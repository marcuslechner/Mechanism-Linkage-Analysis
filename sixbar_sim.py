"""
Six-bar Stephenson linkage simulation.

Computes position/velocity kinematics, motor torque (virtual work),
joint reaction forces (Newton-Euler), and drives an interactive viewer
with synced charts. Can also export animations to GIF.

Usage:
    python sixbar_sim.py mechanism.json --settings settings.json
"""

import argparse
import json
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.patches import Circle
from matplotlib.widgets import Button, Slider
from motiongen_parser import LINEAR_UNIT_TO_M, load_motiongen, Mechanism


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def circle_circle_intersection(c1, r1, c2, r2, prev_point=None):
    """
    Intersect two circles.  Returns the solution closest to *prev_point*
    (or the '+' branch if prev_point is None).  Returns None when the
    circles do not intersect.
    """
    d_vec = c2 - c1
    d = np.linalg.norm(d_vec)

    if d > r1 + r2 + 1e-9 or d < abs(r1 - r2) - 1e-9:
        return None

    a = (r1**2 - r2**2 + d**2) / (2.0 * d)
    h_sq = r1**2 - a**2
    h = np.sqrt(max(h_sq, 0.0))

    unit = d_vec / d
    perp = np.array([-unit[1], unit[0]])
    mid = c1 + a * unit

    p1 = mid + h * perp
    p2 = mid - h * perp

    if prev_point is not None:
        if np.linalg.norm(p1 - prev_point) <= np.linalg.norm(p2 - prev_point):
            return p1
        return p2
    return p1


def cross2d(a, b):
    """2-D scalar cross product:  a x b = ax*by - ay*bx."""
    return a[0] * b[1] - a[1] * b[0]


def signed_angle(vertex, p1, p2):
    """Signed angle at *vertex* from vertex->p1 to vertex->p2 (CCW +)."""
    a1 = np.arctan2(p1[1] - vertex[1], p1[0] - vertex[0])
    a2 = np.arctan2(p2[1] - vertex[1], p2[0] - vertex[0])
    da = (a2 - a1 + np.pi) % (2 * np.pi) - np.pi
    return da


def _strip_json_comments(text: str) -> str:
    """Remove // and /* */ comments from JSON-like text."""
    result = []
    i = 0
    in_string = False
    escape = False

    while i < len(text):
        ch = text[i]
        nxt = text[i + 1] if i + 1 < len(text) else ""

        if in_string:
            result.append(ch)
            if escape:
                escape = False
            elif ch == "\\":
                escape = True
            elif ch == '"':
                in_string = False
            i += 1
            continue

        if ch == '"':
            in_string = True
            result.append(ch)
            i += 1
            continue

        if ch == "/" and nxt == "/":
            i += 2
            while i < len(text) and text[i] not in "\r\n":
                i += 1
            continue

        if ch == "/" and nxt == "*":
            i += 2
            while i + 1 < len(text) and not (text[i] == "*" and text[i + 1] == "/"):
                i += 1
            i += 2
            continue

        result.append(ch)
        i += 1

    return "".join(result)


DEFAULT_METRIC_LINK_MASSES = {
    'L1': 0.045359,
    'L3': 0.226796,
    'L4': 0.090718,
    'L5': 0.136078,
    'L6': 0.068039,
}

FORCE_UNIT_TO_N = {
    'N': 1.0,
    'lbf': 4.4482216152605,
}

TORQUE_UNIT_TO_NM = {
    'N.m': 1.0,
    'N.mm': 1e-3,
    'lbf.in': FORCE_UNIT_TO_N['lbf'] * LINEAR_UNIT_TO_M['in'],
    'lbf.ft': FORCE_UNIT_TO_N['lbf'] * LINEAR_UNIT_TO_M['ft'],
}

ANGLE_UNIT_ALIASES = {
    'deg': 'deg',
    'degree': 'deg',
    'degrees': 'deg',
    'rad': 'rad',
    'radian': 'rad',
    'radians': 'rad',
}


def _normalize_torque_unit(unit: str) -> str:
    normalized = (
        unit.strip()
        .lower()
        .replace(" ", "")
        .replace("*", ".")
        .replace("·", ".")
    )
    aliases = {
        'nm': 'N.m',
        'n.m': 'N.m',
        'nmm': 'N.mm',
        'n.mm': 'N.mm',
        'lbfin': 'lbf.in',
        'lbf.in': 'lbf.in',
        'lbfft': 'lbf.ft',
        'lbf.ft': 'lbf.ft',
    }
    if normalized not in aliases:
        raise ValueError(f"Unsupported torque unit: {unit}")
    return aliases[normalized]


def _normalize_angle_unit(unit: str) -> str:
    normalized = unit.strip().lower()
    if normalized not in ANGLE_UNIT_ALIASES:
        raise ValueError(f"Unsupported angle unit: {unit}")
    return ANGLE_UNIT_ALIASES[normalized]


@dataclass
class SimulationSettings:
    n_steps: int = 720
    gravity: float | None = 9.80665
    start_angle: float | None = None
    end_angle: float | None = None
    length_unit: str = "mm"
    torque_unit: str = "N.m"
    angle_unit: str = "deg"
    length_scale: float = 1.0
    autoscale: bool = False
    autoscale_target_dy: float | None = None
    motor_speed_deg_per_s: float = 10.0
    motor_torque_limit: float | None = 5000.0
    motor_torque_nominal: float | None = None
    export_gif_path: str | None = None
    export_gif_fps: float = 30.0
    payload_mass_kg: float = 22.241108 / 9.80665
    payload_direction: list[float] = field(
        default_factory=lambda: [0.0, -1.0]
    )
    link_masses: dict[str, float] = field(
        default_factory=lambda: DEFAULT_METRIC_LINK_MASSES.copy()
    )

    @classmethod
    def from_dict(cls, data: dict) -> "SimulationSettings":
        payload = data.get('payload', {})
        motor = data.get('motor', {})
        units = data.get('units', {})
        export = data.get('export', {})
        autoscale_cfg = data.get('autoscale', {}) or {}
        payload_mass_kg = payload.get('mass_kg')
        if payload_mass_kg is None:
            legacy_weight_n = payload.get('weight', 22.241108)
            payload_mass_kg = legacy_weight_n / 9.80665
        settings = cls(
            n_steps=data.get('n_steps', 720),
            gravity=data.get('gravity', 9.80665),
            start_angle=data.get('start_angle'),
            end_angle=data.get('end_angle'),
            length_unit=units.get('length', 'mm'),
            torque_unit=units.get('torque', 'N.m'),
            angle_unit=units.get('angle', 'deg'),
            length_scale=data.get('length_scale', 1.0),
            autoscale=bool(autoscale_cfg.get('enabled', False)),
            autoscale_target_dy=autoscale_cfg.get('target_delta_y'),
            motor_speed_deg_per_s=motor.get('speed_deg_per_s', 10.0),
            motor_torque_limit=motor.get('torque_limit', 5000.0),
            motor_torque_nominal=motor.get('torque_nominal'),
            export_gif_path=export.get('gif_path'),
            export_gif_fps=export.get('gif_fps', 30.0),
            payload_mass_kg=payload_mass_kg,
            payload_direction=payload.get('direction', [0.0, -1.0]),
            link_masses=data.get(
                'link_masses', DEFAULT_METRIC_LINK_MASSES.copy()
            ),
        )
        if settings.length_scale <= 0:
            raise ValueError("length_scale must be greater than 0")
        if settings.autoscale:
            if settings.autoscale_target_dy is None:
                raise ValueError(
                    "autoscale.target_delta_y is required when autoscale.enabled is true"
                )
            if settings.autoscale_target_dy <= 0:
                raise ValueError("autoscale.target_delta_y must be greater than 0")
        settings.length_unit = settings.length_unit.lower()
        if settings.length_unit not in LINEAR_UNIT_TO_M:
            raise ValueError(f"Unsupported length unit: {settings.length_unit}")
        settings.torque_unit = _normalize_torque_unit(settings.torque_unit)
        settings.angle_unit = _normalize_angle_unit(settings.angle_unit)
        if settings.export_gif_fps <= 0:
            raise ValueError("export.gif_fps must be greater than 0")
        if settings.export_gif_path is not None and not isinstance(
            settings.export_gif_path, str
        ):
            raise ValueError("export.gif_path must be a string path or null")
        return settings

    @classmethod
    def load_json(cls, filepath: str) -> "SimulationSettings":
        with open(filepath) as f:
            return cls.from_dict(json.loads(_strip_json_comments(f.read())))

    def to_dict(self) -> dict:
        return {
            'n_steps': self.n_steps,
            'gravity': self.gravity,
            'start_angle': self.start_angle,
            'end_angle': self.end_angle,
            'units': {
                'length': self.length_unit,
                'torque': self.torque_unit,
                'angle': self.angle_unit,
            },
            'length_scale': self.length_scale,
            'autoscale': {
                'enabled': self.autoscale,
                'target_delta_y': self.autoscale_target_dy,
            },
            'motor': {
                'speed_deg_per_s': self.motor_speed_deg_per_s,
                'torque_limit': self.motor_torque_limit,
                'torque_nominal': self.motor_torque_nominal,
            },
            'export': {
                'gif_path': self.export_gif_path,
                'gif_fps': self.export_gif_fps,
            },
            'payload': {
                'mass_kg': self.payload_mass_kg,
                'direction': self.payload_direction,
            },
            'link_masses': self.link_masses,
        }

    def payload_force(self, gravity: float) -> np.ndarray:
        direction = np.asarray(self.payload_direction, dtype=float)
        if direction.shape != (2,):
            raise ValueError("payload.direction must be a 2-element vector")

        magnitude = np.linalg.norm(direction)
        if magnitude == 0.0 or self.payload_mass_kg == 0.0:
            return np.zeros(2)
        return (self.payload_mass_kg * gravity) * direction / magnitude

    def cycle_time_s(self) -> float | None:
        if self.motor_speed_deg_per_s == 0:
            return None
        return 360.0 / abs(self.motor_speed_deg_per_s)


# ---------------------------------------------------------------------------
# Main simulation class
# ---------------------------------------------------------------------------

class SixBarSim:
    """
    Quasi-static analysis of a Stephenson six-bar linkage loaded from a
    MotionGen file.

    Topology (labels assigned in MotionGen joint/link order):

        L1  (crank)   : B → A          motor at B
        L2  (ground)  : B – C – D      ternary, fixed
        L3  (coupler) : E – F – G      ternary, E = wheel/output point
        L4            : D → G          binary
        L5            : C – F – H      ternary
        L6            : A → H          binary

    Two loops:
        Loop 1  B – A – H – C   (four-bar, ground segment B–C)
        Loop 2  C – F – G – D   (four-bar, ground segment C–D)
    Connected via ternary link L5 (C – F – H).
    """

    def __init__(
        self,
        input_file: str,
        length_scale: float = 1.0,
        target_length_unit: str | None = None,
        target_angle_unit: str | None = None,
    ):
        if input_file.endswith(".json"):
            self.mech = Mechanism.load_json(input_file)
        else:
            self.mech = load_motiongen(input_file)
        if target_length_unit is not None:
            self.mech = self.mech.to_linear_unit(target_length_unit)
        if target_angle_unit is not None:
            normalized = _normalize_angle_unit(target_angle_unit)
            self.mech.angular_unit = 'degree' if normalized == 'deg' else 'rad'
        self.mech = self.mech.scaled(length_scale)
        self._extract_params()

    # ------------------------------------------------------------------
    # Initialisation helpers
    # ------------------------------------------------------------------

    def _extract_params(self):
        m = self.mech
        act = m.actuators[0]

        # Ground joints
        B_id = act.at_joint_id
        C_id = act.from_joint_id
        ground = m.links[m.ground_link_id]
        D_id = [j for j in ground.joint_ids if j not in (B_id, C_id)][0]

        A_id = act.to_joint_id

        # Build joint → link-ids map
        j2l: dict[str, list[str]] = {}
        for lid, lk in m.links.items():
            for jid in lk.joint_ids:
                j2l.setdefault(jid, []).append(lid)

        # L1 (crank): contains A and B, not ground
        L1_id = next(
            lid for lid in j2l[A_id]
            if B_id in m.links[lid].joint_ids and not m.links[lid].is_ground
        )

        # L6: contains A, not crank, not ground
        L6_id = next(
            lid for lid in j2l[A_id]
            if lid not in (L1_id, m.ground_link_id)
        )
        H_id = [j for j in m.links[L6_id].joint_ids if j != A_id][0]

        # L5: contains C, not ground
        L5_id = next(
            lid for lid in j2l[C_id] if lid != m.ground_link_id
        )
        F_id = [
            j for j in m.links[L5_id].joint_ids if j not in (C_id, H_id)
        ][0]

        # L4: contains D, not ground
        L4_id = next(
            lid for lid in j2l[D_id] if lid != m.ground_link_id
        )
        G_id = [j for j in m.links[L4_id].joint_ids if j != D_id][0]

        # L3: contains F and G (and coupler point E)
        L3_id = next(lid for lid in j2l[F_id] if lid != L5_id)
        E_id = [
            j for j in m.links[L3_id].joint_ids if j not in (F_id, G_id)
        ][0]

        # Store initial positions
        def _p(jid):
            j = m.joints[jid]
            return np.array([j.x, j.y])

        self.B, self.C, self.D = _p(B_id), _p(C_id), _p(D_id)
        self.A0, self.H0, self.F0 = _p(A_id), _p(H_id), _p(F_id)
        self.G0, self.E0 = _p(G_id), _p(E_id)

        # Link lengths
        self.L_BA = np.linalg.norm(self.A0 - self.B)
        self.L_AH = np.linalg.norm(self.H0 - self.A0)
        self.L_CH = np.linalg.norm(self.H0 - self.C)
        self.L_CF = np.linalg.norm(self.F0 - self.C)
        self.L_FH = np.linalg.norm(self.H0 - self.F0)
        self.L_FG = np.linalg.norm(self.G0 - self.F0)
        self.L_DG = np.linalg.norm(self.G0 - self.D)
        self.L_FE = np.linalg.norm(self.E0 - self.F0)
        self.L_GE = np.linalg.norm(self.E0 - self.G0)

        # Internal angles of ternary links (rigid-body offsets)
        # L5: angle at C from C→H to C→F
        self.alpha_L5 = signed_angle(self.C, self.H0, self.F0)
        # L3: angle at F from F→G to F→E
        self.alpha_L3 = signed_angle(self.F0, self.G0, self.E0)

        # Initial crank angle (absolute, B→A)
        self.theta1_init = np.arctan2(
            self.A0[1] - self.B[1], self.A0[0] - self.B[0]
        )

        # Store IDs for reference
        self._ids = dict(
            A=A_id, B=B_id, C=C_id, D=D_id,
            E=E_id, F=F_id, G=G_id, H=H_id,
            L1=L1_id, L2=m.ground_link_id, L3=L3_id,
            L4=L4_id, L5=L5_id, L6=L6_id,
        )

    # ------------------------------------------------------------------
    # Position kinematics
    # ------------------------------------------------------------------

    def solve_position(self, theta1, prev=None):
        """
        Return joint positions {name: ndarray} for the given crank angle,
        or None if the configuration is unreachable.
        """
        if prev is None:
            prev = dict(A=self.A0, H=self.H0, F=self.F0,
                        G=self.G0, E=self.E0)

        pos = dict(B=self.B.copy(), C=self.C.copy(), D=self.D.copy())

        # Crank
        A = self.B + self.L_BA * np.array([np.cos(theta1), np.sin(theta1)])
        pos['A'] = A

        # Loop 1 — find H
        H = circle_circle_intersection(A, self.L_AH, self.C, self.L_CH,
                                        prev['H'])
        if H is None:
            return None
        pos['H'] = H

        # F from ternary L5 (C–H–F rigid)
        theta_CH = np.arctan2(H[1] - self.C[1], H[0] - self.C[0])
        theta_CF = theta_CH + self.alpha_L5
        F = self.C + self.L_CF * np.array([np.cos(theta_CF),
                                            np.sin(theta_CF)])
        pos['F'] = F

        # Loop 2 — find G
        G = circle_circle_intersection(F, self.L_FG, self.D, self.L_DG,
                                        prev['G'])
        if G is None:
            return None
        pos['G'] = G

        # E from ternary L3 (F–G–E rigid)
        theta_FG = np.arctan2(G[1] - F[1], G[0] - F[0])
        theta_FE = theta_FG + self.alpha_L3
        E = F + self.L_FE * np.array([np.cos(theta_FE),
                                       np.sin(theta_FE)])
        pos['E'] = E
        return pos

    # ------------------------------------------------------------------
    # Angle extraction
    # ------------------------------------------------------------------

    @staticmethod
    def _angles(pos):
        """Link direction angles from joint positions."""
        A, B, C, D = pos['A'], pos['B'], pos['C'], pos['D']
        E, F, G, H = pos['E'], pos['F'], pos['G'], pos['H']
        return dict(
            theta1=np.arctan2(A[1] - B[1], A[0] - B[0]),
            theta6=np.arctan2(H[1] - A[1], H[0] - A[0]),
            theta5=np.arctan2(H[1] - C[1], H[0] - C[0]),
            theta4=np.arctan2(G[1] - D[1], G[0] - D[0]),
            theta3=np.arctan2(G[1] - F[1], G[0] - F[0]),
        )

    # ------------------------------------------------------------------
    # Velocity kinematics
    # ------------------------------------------------------------------

    def solve_velocity(self, ang, omega1=1.0):
        """
        Angular velocities of all moving links for a given crank speed.

        Loop 1:  L_BA·ω1·ê⊥(θ1) + L_AH·ω6·ê⊥(θ6) − L_CH·ω5·ê⊥(θ5) = 0
        Loop 2:  L_CF·ω5·ê⊥(θ_CF) + L_FG·ω3·ê⊥(θ3) − L_DG·ω4·ê⊥(θ4) = 0

        Returns dict of angular velocities or None at a singularity.
        """
        t1, t6, t5 = ang['theta1'], ang['theta6'], ang['theta5']
        t3, t4 = ang['theta3'], ang['theta4']
        tCF = t5 + self.alpha_L5

        # Loop 1  →  [ω6, ω5]
        A1 = np.array([
            [-self.L_AH * np.sin(t6),  self.L_CH * np.sin(t5)],
            [ self.L_AH * np.cos(t6), -self.L_CH * np.cos(t5)],
        ])
        b1 = omega1 * np.array([
            self.L_BA * np.sin(t1),
            -self.L_BA * np.cos(t1),
        ])
        try:
            omega6, omega5 = np.linalg.solve(A1, b1)
        except np.linalg.LinAlgError:
            return None

        # Loop 2  →  [ω3, ω4]
        A2 = np.array([
            [-self.L_FG * np.sin(t3),  self.L_DG * np.sin(t4)],
            [ self.L_FG * np.cos(t3), -self.L_DG * np.cos(t4)],
        ])
        b2 = omega5 * np.array([
            self.L_CF * np.sin(tCF),
            -self.L_CF * np.cos(tCF),
        ])
        try:
            omega3, omega4 = np.linalg.solve(A2, b2)
        except np.linalg.LinAlgError:
            return None

        return dict(omega1=omega1, omega6=omega6, omega5=omega5,
                    omega3=omega3, omega4=omega4)

    # ------------------------------------------------------------------
    # Joint linear velocities  (needed for virtual work & Newton-Euler)
    # ------------------------------------------------------------------

    def joint_velocities(self, ang, omg):
        """Linear velocities of every joint (2-D vectors)."""
        t1, t6 = ang['theta1'], ang['theta6']
        t5, t3, t4 = ang['theta5'], ang['theta3'], ang['theta4']
        tCF = t5 + self.alpha_L5
        tFE = t3 + self.alpha_L3

        zero = np.zeros(2)

        def _perp(theta, length, omega):
            return length * omega * np.array([-np.sin(theta), np.cos(theta)])

        vA = _perp(t1, self.L_BA, omg['omega1'])
        vH = vA + _perp(t6, self.L_AH, omg['omega6'])
        vF = _perp(tCF, self.L_CF, omg['omega5'])
        vG = _perp(t4, self.L_DG, omg['omega4'])
        vE = vF + _perp(tFE, self.L_FE, omg['omega3'])

        return dict(B=zero, C=zero, D=zero,
                    A=vA, H=vH, F=vF, G=vG, E=vE)

    # ------------------------------------------------------------------
    # Centre-of-mass helpers (binary → midpoint, ternary → centroid)
    # ------------------------------------------------------------------

    @staticmethod
    def _cm_binary(p1, p2):
        return 0.5 * (p1 + p2)

    @staticmethod
    def _cm_ternary(p1, p2, p3):
        return (p1 + p2 + p3) / 3.0

    def _cm_positions(self, pos):
        """Return CM position for each moving link."""
        return dict(
            L1=self._cm_binary(pos['B'], pos['A']),
            L6=self._cm_binary(pos['A'], pos['H']),
            L5=self._cm_ternary(pos['C'], pos['F'], pos['H']),
            L3=self._cm_ternary(pos['E'], pos['F'], pos['G']),
            L4=self._cm_binary(pos['D'], pos['G']),
        )

    def _cm_velocities(self, vel):
        """Return CM velocity for each moving link."""
        return dict(
            L1=self._cm_binary(vel['B'], vel['A']),
            L6=self._cm_binary(vel['A'], vel['H']),
            L5=self._cm_ternary(vel['C'], vel['F'], vel['H']),
            L3=self._cm_ternary(vel['E'], vel['F'], vel['G']),
            L4=self._cm_binary(vel['D'], vel['G']),
        )

    def _force_unit(self):
        # With mass in kg and payload in N, reaction forces are solved in N.
        return 'N'

    def _native_torque_unit(self):
        return f"{self._force_unit()}·{self.mech.linear_unit}"

    def _native_torque_to_nm_factor(self):
        length_factor = LINEAR_UNIT_TO_M[self.mech.linear_unit.lower()]
        # Native torque unit is N * (mechanism length unit).
        return length_factor

    def _native_to_display_torque_scale(self, display_torque_unit: str):
        target_factor = TORQUE_UNIT_TO_NM[display_torque_unit]
        return self._native_torque_to_nm_factor() / target_factor

    @staticmethod
    def _angle_values(theta_rad, angle_unit: str):
        if angle_unit == 'deg':
            return np.degrees(theta_rad)
        return theta_rad

    @staticmethod
    def _angle_to_radians(angle_value: float, angle_unit: str):
        if angle_unit == 'deg':
            return float(np.radians(angle_value))
        return float(angle_value)

    @staticmethod
    def _angle_axis_label(angle_unit: str):
        if angle_unit == 'deg':
            return '°'
        return 'rad'

    def _default_gravity(self):
        # SI gravity in m/s^2. Keeps mass(kg) and force(N) consistent
        # regardless of geometry length unit.
        return 9.80665

    # ------------------------------------------------------------------
    # Motor torque  (principle of virtual work, quasi-static)
    # ------------------------------------------------------------------

    def motor_torque(self, pos, ang, link_masses, gravity,
                     ext_force_E=None):
        """
        τ · ω₁ + Σ(-mᵢ g v_cm_y_i) + F_ext · v_E = 0
        → τ = Σ(mᵢ g v_cm_y_i) − F_ext · v_E       (with ω₁ = 1)

        *link_masses*: dict  {'L1': m, 'L3': m, …}
        *gravity*:  scalar  (positive, acts in −y)
        *ext_force_E*:  [Fx, Fy] applied at coupler point E
        """
        omg = self.solve_velocity(ang, omega1=1.0)
        if omg is None:
            return np.nan

        vel = self.joint_velocities(ang, omg)
        cm_vel = self._cm_velocities(vel)

        tau = 0.0
        for name, vcm in cm_vel.items():
            mi = link_masses.get(name, 0.0)
            tau += mi * gravity * vcm[1]          # lift against gravity

        if ext_force_E is not None:
            f = np.asarray(ext_force_E, dtype=float)
            tau -= f.dot(vel['E'])

        return tau

    # ------------------------------------------------------------------
    # Joint reaction forces  (Newton-Euler, quasi-static)
    # ------------------------------------------------------------------

    def joint_forces(self, pos, ang, link_masses, gravity,
                     ext_force_E=None):
        """
        Solve for all joint reaction forces and motor torque via
        Newton-Euler static equilibrium on each moving link.

        Returns (forces_dict, tau_motor).
        forces_dict keys: 'F_B', 'F_A', 'F_H', 'F_C5', 'F_F', 'F_G', 'F_D'
        Each value is an ndarray([Fx, Fy]).
        """
        cm = self._cm_positions(pos)
        g_vec = np.array([0.0, -gravity])

        if ext_force_E is None:
            ext_force_E = np.zeros(2)
        else:
            ext_force_E = np.asarray(ext_force_E, dtype=float)

        # Unknown vector (15):
        # [F_Bx, F_By, F_Ax, F_Ay, tau,
        #  F_Hx, F_Hy, F_Cx, F_Cy, F_Fx, F_Fy,
        #  F_Gx, F_Gy, F_Dx, F_Dy]
        #   0      1     2     3    4
        #   5      6     7     8    9    10
        #  11     12    13    14

        M = np.zeros((15, 15))
        rhs = np.zeros(15)

        def _w(name):
            return link_masses.get(name, 0.0) * g_vec

        # ---- L1 (crank):  F_B + F_A + W1 = 0,  moments about CM1 ----
        W1 = _w('L1')
        r_B_cm1 = pos['B'] - cm['L1']
        r_A_cm1 = pos['A'] - cm['L1']

        # Fx: F_Bx + F_Ax = -W1x
        M[0, 0] = 1;  M[0, 2] = 1
        rhs[0] = -W1[0]
        # Fy: F_By + F_Ay = -W1y
        M[1, 1] = 1;  M[1, 3] = 1
        rhs[1] = -W1[1]
        # Mz: r_B×F_B + r_A×F_A + τ = 0
        #   r×F = rx*Fy - ry*Fx
        M[2, 0] = -r_B_cm1[1];  M[2, 1] = r_B_cm1[0]   # r_B × F_B
        M[2, 2] = -r_A_cm1[1];  M[2, 3] = r_A_cm1[0]   # r_A × F_A
        M[2, 4] = 1                                       # τ_motor
        rhs[2] = 0

        # ---- L6:  −F_A + F_H + W6 = 0 ----
        W6 = _w('L6')
        r_A_cm6 = pos['A'] - cm['L6']
        r_H_cm6 = pos['H'] - cm['L6']

        M[3, 2] = -1;  M[3, 5] = 1
        rhs[3] = -W6[0]
        M[4, 3] = -1;  M[4, 6] = 1
        rhs[4] = -W6[1]
        # moment
        M[5, 2] = r_A_cm6[1];   M[5, 3] = -r_A_cm6[0]   # −F_A contrib
        M[5, 5] = -r_H_cm6[1];  M[5, 6] = r_H_cm6[0]    # +F_H contrib
        rhs[5] = 0

        # ---- L5 (ternary):  F_C5 − F_H + F_F + W5 = 0 ----
        W5 = _w('L5')
        r_C_cm5 = pos['C'] - cm['L5']
        r_H_cm5 = pos['H'] - cm['L5']
        r_F_cm5 = pos['F'] - cm['L5']

        M[6, 7] = 1;  M[6, 5] = -1;  M[6, 9] = 1
        rhs[6] = -W5[0]
        M[7, 8] = 1;  M[7, 6] = -1;  M[7, 10] = 1
        rhs[7] = -W5[1]
        # moment
        M[8, 7] = -r_C_cm5[1];  M[8, 8] = r_C_cm5[0]
        M[8, 5] = r_H_cm5[1];   M[8, 6] = -r_H_cm5[0]
        M[8, 9] = -r_F_cm5[1];  M[8, 10] = r_F_cm5[0]
        rhs[8] = 0

        # ---- L3 (coupler):  −F_F + F_G + W3 + F_ext = 0 ----
        W3 = _w('L3')
        r_F_cm3 = pos['F'] - cm['L3']
        r_G_cm3 = pos['G'] - cm['L3']
        r_E_cm3 = pos['E'] - cm['L3']

        M[9, 9] = -1;   M[9, 11] = 1
        rhs[9] = -W3[0] - ext_force_E[0]
        M[10, 10] = -1;  M[10, 12] = 1
        rhs[10] = -W3[1] - ext_force_E[1]
        # moment
        M[11, 9] = r_F_cm3[1];    M[11, 10] = -r_F_cm3[0]
        M[11, 11] = -r_G_cm3[1];  M[11, 12] = r_G_cm3[0]
        rhs[11] = -cross2d(r_E_cm3, ext_force_E)

        # ---- L4:  F_D − F_G + W4 = 0 ----
        W4 = _w('L4')
        r_D_cm4 = pos['D'] - cm['L4']
        r_G_cm4 = pos['G'] - cm['L4']

        M[12, 13] = 1;  M[12, 11] = -1
        rhs[12] = -W4[0]
        M[13, 14] = 1;  M[13, 12] = -1
        rhs[13] = -W4[1]
        # moment
        M[14, 13] = -r_D_cm4[1];  M[14, 14] = r_D_cm4[0]
        M[14, 11] = r_G_cm4[1];   M[14, 12] = -r_G_cm4[0]
        rhs[14] = 0

        x = np.linalg.solve(M, rhs)

        forces = {
            'F_B': x[0:2],
            'F_A': x[2:4],
            'tau':  x[4],
            'F_H': x[5:7],
            'F_C': x[7:9],
            'F_F': x[9:11],
            'F_G': x[11:13],
            'F_D': x[13:15],
        }
        return forces

    # ------------------------------------------------------------------
    # Full sweep
    # ------------------------------------------------------------------

    def measure_E_y_travel(self, n_steps, theta_start=None, theta_end=None):
        """
        Sweep the crank and return the peak-to-peak Y travel of point E,
        in the sim's current linear unit. Only runs position kinematics —
        no force/torque work.
        """
        if theta_start is None:
            theta_start = self.theta1_init
        if theta_end is None:
            theta_end = theta_start + 2 * np.pi
        thetas = np.linspace(theta_start, theta_end, n_steps, endpoint=False)
        ys = []
        prev = None
        for th in thetas:
            pos = self.solve_position(th, prev)
            if pos is None:
                continue
            prev = pos
            ys.append(pos['E'][1])
        if not ys:
            raise RuntimeError(
                "Autoscale probe found no valid configurations across the sweep."
            )
        return float(max(ys) - min(ys))

    def run(
        self,
        n_steps=360,
        link_masses=None,
        gravity=None,
        ext_force_E=None,
        theta_start=None,
        theta_end=None,
    ):
        """
        Sweep the crank through 360° and compute everything.

        Parameters
        ----------
        n_steps : int
            Angular resolution.
        link_masses : dict
            {'L1': mass, 'L3': mass, …}.  Defaults to 0 for missing links.
        gravity : float or None
            Gravitational acceleration in consistent units
            for the mechanism's linear unit. Defaults to an inferred value
            from `self.mech.linear_unit`.
        ext_force_E : array-like or None
            Constant external force [Fx, Fy] at the wheel/coupler point.
        theta_start : float or None
            Start crank angle in radians (absolute). Defaults to initial angle.
        theta_end : float or None
            End crank angle in radians (absolute). Defaults to start + 2*pi.

        Returns
        -------
        dict with arrays keyed by name.
        """
        if link_masses is None:
            link_masses = {}
        if gravity is None:
            gravity = self._default_gravity()
        if theta_start is None:
            theta_start = self.theta1_init
        if theta_end is None:
            theta_end = theta_start + 2 * np.pi

        theta_range = np.linspace(
            theta_start,
            theta_end,
            n_steps,
            endpoint=False,
        )

        # Storage
        all_pos = []
        torques_vw = np.full(n_steps, np.nan)
        torques_ne = np.full(n_steps, np.nan)
        joint_force_mag = {
            k: np.full(n_steps, np.nan)
            for k in ('F_B', 'F_A', 'F_H', 'F_C', 'F_F', 'F_G', 'F_D')
        }

        prev = None
        valid_count = 0

        for i, th in enumerate(theta_range):
            pos = self.solve_position(th, prev)
            if pos is None:
                all_pos.append(None)
                continue

            all_pos.append(pos)
            prev = pos
            valid_count += 1

            ang = self._angles(pos)

            # Virtual-work torque
            torques_vw[i] = self.motor_torque(
                pos, ang, link_masses, gravity, ext_force_E
            )

            # Newton-Euler joint forces
            try:
                frc = self.joint_forces(
                    pos, ang, link_masses, gravity, ext_force_E
                )
                torques_ne[i] = frc['tau']
                for k in joint_force_mag:
                    joint_force_mag[k][i] = np.linalg.norm(frc[k])
            except np.linalg.LinAlgError:
                pass

        print(f"Valid configurations: {valid_count}/{n_steps}")

        return dict(
            theta=theta_range,
            positions=all_pos,
            torque_vw=torques_vw,
            torque_ne=torques_ne,
            joint_force_mag=joint_force_mag,
        )

    def run_with_settings(self, settings: SimulationSettings):
        gravity = settings.gravity
        if gravity is None:
            gravity = self._default_gravity()
        if settings.start_angle is None and settings.end_angle is None:
            theta_start = None
            theta_end = None
        elif settings.start_angle is None or settings.end_angle is None:
            raise ValueError(
                "Set both start_angle and end_angle, or leave both unset."
            )
        else:
            theta_start = self._angle_to_radians(
                settings.start_angle, settings.angle_unit
            )
            theta_end = self._angle_to_radians(
                settings.end_angle, settings.angle_unit
            )

        results = self.run(
            n_steps=settings.n_steps,
            link_masses=settings.link_masses,
            gravity=gravity,
            ext_force_E=settings.payload_force(gravity),
            theta_start=theta_start,
            theta_end=theta_end,
        )
        torque_scale = self._native_to_display_torque_scale(settings.torque_unit)
        results['torque_unit'] = settings.torque_unit
        results['torque_vw_display'] = results['torque_vw'] * torque_scale
        results['torque_ne_display'] = results['torque_ne'] * torque_scale
        results['angle_unit'] = settings.angle_unit
        results['theta_display'] = self._angle_values(
            results['theta'] - self.theta1_init, settings.angle_unit
        )
        results['torque_scale'] = torque_scale
        results['motor_speed_deg_per_s'] = settings.motor_speed_deg_per_s
        results['motor_torque_limit'] = settings.motor_torque_limit
        results['motor_torque_limit_display'] = settings.motor_torque_limit
        if settings.motor_torque_limit is None:
            results['motor_torque_limit_native'] = None
        else:
            results['motor_torque_limit_native'] = (
                settings.motor_torque_limit / torque_scale
            )
        results['motor_torque_nominal'] = settings.motor_torque_nominal
        results['motor_torque_nominal_display'] = settings.motor_torque_nominal
        results['cycle_time_s'] = settings.cycle_time_s()
        results['settings'] = settings.to_dict()
        return results

    # ------------------------------------------------------------------
    # Plotting
    # ------------------------------------------------------------------

    def plot_torque(self, results, title="Motor Torque vs Crank Angle"):
        fig, ax = plt.subplots(figsize=(10, 5))
        angle_unit = results.get('angle_unit', 'deg')
        theta_vals = results.get(
            'theta_display',
            self._angle_values(results['theta'] - self.theta1_init, angle_unit),
        )

        torque_vw = results.get('torque_vw_display', results['torque_vw'])
        torque_ne = results.get('torque_ne_display', results['torque_ne'])
        ax.plot(theta_vals, torque_vw, label='Virtual Work')
        ax.plot(theta_vals, torque_ne, '--', label='Newton-Euler',
                alpha=0.7)

        torque_limit = results.get(
            'motor_torque_limit_display', results.get('motor_torque_limit')
        )
        if torque_limit is not None:
            ax.axhline(torque_limit, color='r', linestyle=':', alpha=0.8,
                       label='Motor Limit')
            ax.axhline(-torque_limit, color='r', linestyle=':', alpha=0.8)

        torque_nominal = results.get(
            'motor_torque_nominal_display', results.get('motor_torque_nominal')
        )
        if torque_nominal is not None:
            ax.axhline(torque_nominal, color='orange', linestyle='--',
                       alpha=0.8, label='Nominal')
            ax.axhline(-torque_nominal, color='orange', linestyle='--',
                       alpha=0.8)

        ax.set_xlabel(f'Crank Angle ({self._angle_axis_label(angle_unit)})')
        ax.set_ylabel(
            f"Motor Torque ({results.get('torque_unit', self._native_torque_unit())})"
        )
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        return fig

    def plot_joint_forces(self, results,
                          title="Joint Reaction Forces vs Crank Angle"):
        fig, ax = plt.subplots(figsize=(10, 5))
        angle_unit = results.get('angle_unit', 'deg')
        theta_vals = results.get(
            'theta_display',
            self._angle_values(results['theta'] - self.theta1_init, angle_unit),
        )

        for name, mag in results['joint_force_mag'].items():
            ax.plot(theta_vals, mag, label=name)

        ax.set_xlabel(f'Crank Angle ({self._angle_axis_label(angle_unit)})')
        ax.set_ylabel(f'Force Magnitude ({self._force_unit()})')
        ax.set_title(title)
        ax.legend()
        ax.grid(True, alpha=0.3)
        fig.tight_layout()
        return fig

    def plot_coupler_curve(self, results, title="Coupler Point (E) Path"):
        fig, ax = plt.subplots(figsize=(7, 10))
        xs, ys = [], []
        for pos in results['positions']:
            if pos is not None:
                xs.append(pos['E'][0])
                ys.append(pos['E'][1])

        ax.plot(xs, ys, 'b-', linewidth=1.5)
        ax.plot(xs[0], ys[0], 'go', markersize=8, label='Start')
        ax.set_xlabel(f'X ({self.mech.linear_unit})')
        ax.set_ylabel(f'Y ({self.mech.linear_unit})')
        ax.set_title(title)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()
        return fig

    def plot_linkage(self, pos, ax=None, title="Linkage Configuration"):
        """Draw the linkage at a single configuration."""
        if ax is None:
            fig, ax = plt.subplots(figsize=(7, 10))
        else:
            fig = ax.figure

        A, B, C, D = pos['A'], pos['B'], pos['C'], pos['D']
        E, F, G, H = pos['E'], pos['F'], pos['G'], pos['H']

        link_style = dict(linewidth=2.5, solid_capstyle='round')
        ground_style = dict(linewidth=3, color='gray', solid_capstyle='round')
        joint_style = dict(marker='o', markersize=6, color='black', zorder=5)

        # Ground triangle
        for p1, p2 in [(B, C), (C, D), (B, D)]:
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], **ground_style)

        # Moving links
        colours = {'L1': '#e74c3c', 'L6': '#3498db', 'L5': '#2ecc71',
                   'L3': '#9b59b6', 'L4': '#e67e22'}
        segments = {
            'L1': [(B, A)],
            'L6': [(A, H)],
            'L5': [(C, H), (C, F), (H, F)],
            'L3': [(F, G), (F, E), (G, E)],
            'L4': [(D, G)],
        }
        for name, segs in segments.items():
            for p1, p2 in segs:
                ax.plot([p1[0], p2[0]], [p1[1], p2[1]],
                        color=colours[name], **link_style)

        # Joints
        points = np.array([pos[name] for name in 'ABCDEFGH'])
        label_offset = max(np.ptp(points[:, 0]), np.ptp(points[:, 1])) * 0.015
        label_offset = max(label_offset, 0.15)

        for name, p in pos.items():
            ax.plot(p[0], p[1], **joint_style)
            ax.annotate(
                name, (p[0] + label_offset, p[1] + label_offset), fontsize=9
            )

        # Ground hatching
        for p in [B, C, D]:
            ax.plot(p[0], p[1], 's', markersize=10, color='gray', zorder=4)

        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_title(title)
        fig.tight_layout()
        return fig

    def animate(self, results, interval=30, save_path=None, fps=30):
        """Animate the mechanism with synced torque, force, and coupler charts."""
        valid_indices = [
            i for i, p in enumerate(results['positions']) if p is not None
        ]
        if not valid_indices:
            print("No valid positions to animate.")
            return None
        if fps <= 0:
            raise ValueError("fps must be greater than 0")

        positions = results['positions']

        # --- Axis data ---
        angle_unit = results.get('angle_unit', 'deg')
        theta_display = results.get(
            'theta_display',
            self._angle_values(
                results['theta'] - self.theta1_init, angle_unit
            ),
        )
        angle_label = self._angle_axis_label(angle_unit)
        torque_vw = results.get('torque_vw_display', results['torque_vw'])
        torque_ne = results.get('torque_ne_display', results['torque_ne'])
        torque_unit = results.get('torque_unit', self._native_torque_unit())
        force_unit = self._force_unit()
        length_unit = self.mech.linear_unit

        # --- Figure layout: linkage left, 3 charts right ---
        fig = plt.figure(figsize=(16, 9))
        gs = fig.add_gridspec(3, 2, width_ratios=[1.15, 1.0], hspace=0.38)
        ax_link = fig.add_subplot(gs[:, 0])
        ax_torque = fig.add_subplot(gs[0, 1])
        ax_force = fig.add_subplot(gs[1, 1], sharex=ax_torque)
        ax_coupler = fig.add_subplot(gs[2, 1], sharex=ax_torque)

        # --- Linkage axes ---
        all_pts = np.array([
            [positions[i][k] for k in 'ABCDEFGH'] for i in valid_indices
        ])
        span = max(np.ptp(all_pts[:, :, 0]), np.ptp(all_pts[:, :, 1]))
        margin = max(span * 0.05, 1.0)
        ax_link.set_xlim(
            all_pts[:, :, 0].min() - margin,
            all_pts[:, :, 0].max() + margin,
        )
        ax_link.set_ylim(
            all_pts[:, :, 1].min() - margin,
            all_pts[:, :, 1].max() + margin,
        )
        ax_link.set_aspect('equal')
        ax_link.grid(True, alpha=0.3)
        ax_link.set_title('Six-Bar Linkage')

        seg_colors = [
            'gray', 'gray',
            '#e74c3c',
            '#3498db',
            '#2ecc71', '#2ecc71', '#2ecc71',
            '#9b59b6', '#9b59b6', '#9b59b6',
            '#e67e22',
        ]
        seg_defs = [
            ('B', 'C'), ('C', 'D'),
            ('B', 'A'),
            ('A', 'H'),
            ('C', 'H'), ('C', 'F'), ('H', 'F'),
            ('F', 'G'), ('F', 'E'), ('G', 'E'),
            ('D', 'G'),
        ]
        seg_lines = []
        for color in seg_colors:
            lw = 3 if color == 'gray' else 2.5
            line, = ax_link.plot(
                [], [], color=color, lw=lw, solid_capstyle='round',
            )
            seg_lines.append(line)

        joints_plot, = ax_link.plot([], [], 'ko', markersize=5, zorder=5)
        coupler_trail, = ax_link.plot(
            [], [], 'b-', linewidth=1.2, alpha=0.45,
        )
        label_texts = {
            name: ax_link.text(0, 0, name, fontsize=9) for name in 'ABCDEFGH'
        }
        trail_x, trail_y = [], []

        # --- Torque chart ---
        ax_torque.plot(theta_display, torque_vw, label='Virtual Work')
        ax_torque.plot(
            theta_display, torque_ne, '--', label='Newton-Euler', alpha=0.7,
        )
        torque_limit = results.get(
            'motor_torque_limit_display', results.get('motor_torque_limit'),
        )
        if torque_limit is not None:
            ax_torque.axhline(
                torque_limit, color='r', ls=':', alpha=0.8, label='Motor Limit',
            )
            ax_torque.axhline(-torque_limit, color='r', ls=':', alpha=0.8)
        torque_nominal = results.get(
            'motor_torque_nominal_display',
            results.get('motor_torque_nominal'),
        )
        if torque_nominal is not None:
            ax_torque.axhline(
                torque_nominal, color='orange', ls='--', alpha=0.8,
                label='Nominal',
            )
            ax_torque.axhline(
                -torque_nominal, color='orange', ls='--', alpha=0.8,
            )
        torque_cursor = ax_torque.axvline(
            theta_display[valid_indices[0]], color='k', lw=1.5, alpha=0.8,
        )
        ax_torque.set_ylabel(f'Torque ({torque_unit})')
        ax_torque.set_title('Motor Torque')
        ax_torque.grid(True, alpha=0.3)
        ax_torque.legend(loc='best', fontsize=8)

        # --- Joint forces chart ---
        for name, mag in results['joint_force_mag'].items():
            ax_force.plot(theta_display, mag, label=name)
        force_cursor = ax_force.axvline(
            theta_display[valid_indices[0]], color='k', lw=1.5, alpha=0.8,
        )
        ax_force.set_ylabel(f'Force ({force_unit})')
        ax_force.set_title('Joint Reaction Forces')
        ax_force.grid(True, alpha=0.3)
        ax_force.legend(loc='best', fontsize=7, ncol=2)

        # --- Coupler position chart ---
        coupler_x = np.array([
            positions[i]['E'][0] if positions[i] is not None else np.nan
            for i in range(len(positions))
        ])
        coupler_y = np.array([
            positions[i]['E'][1] if positions[i] is not None else np.nan
            for i in range(len(positions))
        ])
        ax_coupler.plot(
            theta_display, coupler_x, label=f'E.x ({length_unit})',
        )
        ax_coupler.plot(
            theta_display, coupler_y, label=f'E.y ({length_unit})',
        )
        coupler_cursor = ax_coupler.axvline(
            theta_display[valid_indices[0]], color='k', lw=1.5, alpha=0.8,
        )
        ax_coupler.set_xlabel(f'Crank Angle ({angle_label})')
        ax_coupler.set_ylabel(f'Position ({length_unit})')
        ax_coupler.set_title('Coupler Point E')
        ax_coupler.grid(True, alpha=0.3)
        ax_coupler.legend(loc='best', fontsize=8)

        fig.subplots_adjust(
            left=0.06, right=0.97, top=0.95, bottom=0.07,
            wspace=0.28, hspace=0.38,
        )

        # --- Animation ---
        all_artists = (
            seg_lines + [joints_plot, coupler_trail]
            + list(label_texts.values())
            + [torque_cursor, force_cursor, coupler_cursor]
        )

        def init():
            for l in seg_lines:
                l.set_data([], [])
            joints_plot.set_data([], [])
            coupler_trail.set_data([], [])
            return all_artists

        def update(frame):
            idx = valid_indices[frame]
            pos = positions[idx]
            angle = theta_display[idx]

            # Update linkage segments
            for line, (j1, j2) in zip(seg_lines, seg_defs):
                p1, p2 = pos[j1], pos[j2]
                line.set_data([p1[0], p2[0]], [p1[1], p2[1]])

            # Joints
            pts = np.array([pos[k] for k in 'ABCDEFGH'])
            joints_plot.set_data(pts[:, 0], pts[:, 1])
            offset = max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])) * 0.015
            offset = max(offset, 0.15)
            for name in 'ABCDEFGH':
                p = pos[name]
                label_texts[name].set_position(
                    (p[0] + offset, p[1] + offset),
                )

            # Coupler trail
            trail_x.append(pos['E'][0])
            trail_y.append(pos['E'][1])
            coupler_trail.set_data(trail_x, trail_y)

            # Move cursors
            for cursor in (torque_cursor, force_cursor, coupler_cursor):
                cursor.set_xdata([angle, angle])

            ax_link.set_title(f'Linkage @ {angle:.1f}{angle_label}')
            return all_artists

        anim = animation.FuncAnimation(
            fig, update, init_func=init,
            frames=len(valid_indices), interval=interval, blit=True,
        )

        if save_path:
            save_path = str(save_path)
            if not save_path.lower().endswith('.gif'):
                raise ValueError("GIF export path must end with .gif")
            print(f"Processing GIF animation, please wait... ({save_path})", flush=True)
            anim.save(save_path, writer='pillow', fps=fps)
            print(f"Animation saved to {save_path}")

        return anim


# ---------------------------------------------------------------------------
# Interactive viewer
# ---------------------------------------------------------------------------

_INTERACTIVE_BACKENDS = {
    'gtk3agg', 'gtk3cairo',
    'gtk4agg', 'gtk4cairo',
    'macosx',
    'nbagg', 'notebook',
    'qtagg', 'qtcairo', 'qt5agg', 'qt5cairo',
    'tkagg', 'tkcairo',
    'webagg',
    'wx', 'wxagg', 'wxcairo',
}

_NON_INTERACTIVE_BACKENDS = {
    'agg',
    'cairo',
    'pdf',
    'pgf',
    'ps',
    'svg',
    'template',
    'module://matplotlib_inline.backend_inline',
}


def _is_non_interactive_backend(backend_name: str) -> bool:
    name = backend_name.lower()
    if name in _INTERACTIVE_BACKENDS:
        return False
    return name in _NON_INTERACTIVE_BACKENDS


def _has_qt_binding() -> bool:
    for module_name in ('PyQt6', 'PySide6', 'PyQt5', 'PySide2'):
        try:
            __import__(module_name)
            return True
        except Exception:
            continue
    return False


def _ensure_interactive_backend() -> bool:
    """Switch pyplot to an interactive backend if the current one is not.
    Returns True if an interactive backend is active, False otherwise."""
    if not _is_non_interactive_backend(plt.get_backend()):
        return True

    try:
        import tkinter  # noqa: F401
        plt.switch_backend('TkAgg')
        return True
    except Exception:
        pass

    if _has_qt_binding():
        try:
            plt.switch_backend('QtAgg')
            return True
        except Exception:
            pass

    return not _is_non_interactive_backend(plt.get_backend())


class SixBarInteractiveViewer:
    """Interactive viewer: linkage animation + synced torque/force/coupler charts."""

    def __init__(self, sim: 'SixBarSim', settings: 'SimulationSettings', results: dict):
        self.sim = sim
        self.settings = settings
        self.results = results

        self.positions = results['positions']
        self.valid_indices = np.array(
            [i for i, pos in enumerate(self.positions) if pos is not None], dtype=int
        )
        if self.valid_indices.size == 0:
            raise ValueError("No valid positions available for interactive viewer.")

        self.angle_unit = settings.angle_unit
        self.angle_label = SixBarSim._angle_axis_label(self.angle_unit)
        self.theta_abs = SixBarSim._angle_values(results['theta'], self.angle_unit)
        self.valid_angles = self.theta_abs[self.valid_indices]

        self.torque = results.get('torque_vw_display', results['torque_vw'])
        self.torque_ne = results.get('torque_ne_display', results['torque_ne'])
        self.torque_unit = results.get('torque_unit', sim._native_torque_unit())
        self.force_unit = sim._force_unit()
        self.length_unit = sim.mech.linear_unit

        self.coupler_x = np.full_like(self.theta_abs, np.nan, dtype=float)
        self.coupler_y = np.full_like(self.theta_abs, np.nan, dtype=float)
        for i, pos in enumerate(self.positions):
            if pos is not None:
                self.coupler_x[i] = pos['E'][0]
                self.coupler_y[i] = pos['E'][1]

        self.current_valid_pos = 0
        self.is_playing = False
        self.min_interval_ms = 10
        self.frames_per_tick = 1

        # Real-time base playback: at Speed x = 1.0 the viewer advances at
        # settings.motor_speed_deg_per_s (degrees of crank per real second).
        theta_rad = results['theta']
        valid_theta_rad = theta_rad[self.valid_indices]
        if valid_theta_rad.size >= 2:
            step_deg = float(np.degrees(abs(valid_theta_rad[1] - valid_theta_rad[0])))
        else:
            step_deg = 1.0
        crank_speed = max(abs(settings.motor_speed_deg_per_s), 1e-6)
        self.base_interval_ms = max(
            self.min_interval_ms, int(round(1000.0 * step_deg / crank_speed))
        )

        self._build_figure()
        self._draw_static()
        self._update_by_valid_pos(0)

    def _build_figure(self):
        fig = plt.figure(figsize=(16, 9))
        gs = fig.add_gridspec(
            3, 2, width_ratios=[1.15, 1.0], height_ratios=[1, 1, 1], hspace=0.34
        )

        self.fig = fig
        self.ax_link = fig.add_subplot(gs[:, 0])
        self.ax_torque = fig.add_subplot(gs[0, 1])
        self.ax_force = fig.add_subplot(gs[1, 1], sharex=self.ax_torque)
        self.ax_coupler = fig.add_subplot(gs[2, 1], sharex=self.ax_torque)

        self.ax_slider = fig.add_axes([0.12, 0.02, 0.58, 0.035])
        self.ax_play = fig.add_axes([0.73, 0.02, 0.1, 0.05])
        self.ax_speed = fig.add_axes([0.85, 0.02, 0.12, 0.035])

        self.angle_slider = Slider(
            ax=self.ax_slider,
            label=f'Crank Angle ({self.angle_label})',
            valmin=float(self.valid_angles.min()),
            valmax=float(self.valid_angles.max()),
            valinit=float(self.valid_angles[0]),
        )
        self.angle_slider.on_changed(self._on_slider_changed)

        self.play_button = Button(self.ax_play, 'Play')
        self.play_button.on_clicked(self._on_play_pause)

        self.speed_slider = Slider(
            ax=self.ax_speed,
            label='Speed x',
            valmin=0.25,
            valmax=15.0,
            valinit=5.0,
        )
        self.speed_slider.on_changed(self._on_speed_changed)

        self.timer = self.fig.canvas.new_timer(interval=self.base_interval_ms)
        self.timer.add_callback(self._on_timer_tick)
        self._on_speed_changed(self.speed_slider.val)

    def _draw_static(self):
        all_pts = np.array(
            [[p[k] for k in 'ABCDEFGH'] for p in self.positions if p is not None]
        )
        span = max(np.ptp(all_pts[:, :, 0]), np.ptp(all_pts[:, :, 1]))
        margin = max(1.0, 0.05 * span)
        self.ax_link.set_xlim(all_pts[:, :, 0].min() - margin, all_pts[:, :, 0].max() + margin)
        self.ax_link.set_ylim(all_pts[:, :, 1].min() - margin, all_pts[:, :, 1].max() + margin)
        self.ax_link.set_aspect('equal')
        self.ax_link.grid(True, alpha=0.3)
        self.ax_link.set_title('Linkage Configuration')

        motor_radius_mm = 40.0
        motor_radius = motor_radius_mm * LINEAR_UNIT_TO_M['mm'] / LINEAR_UNIT_TO_M[self.length_unit]
        motor_circle = Circle(
            self.sim.B, motor_radius,
            fill=False, edgecolor='red', linewidth=1.5,
            linestyle='--', alpha=0.7, zorder=3,
            label='Motor (Ø80 mm)',
        )
        self.ax_link.add_patch(motor_circle)
        self.ax_link.legend(handles=[motor_circle], loc='upper right', fontsize=8)

        self.segment_colors = [
            'gray', 'gray',
            '#e74c3c',
            '#3498db',
            '#2ecc71', '#2ecc71', '#2ecc71',
            '#9b59b6', '#9b59b6', '#9b59b6',
            '#e67e22',
        ]
        self.segment_defs = [
            ('B', 'C'), ('C', 'D'),
            ('B', 'A'),
            ('A', 'H'),
            ('C', 'H'), ('C', 'F'), ('H', 'F'),
            ('F', 'G'), ('F', 'E'), ('G', 'E'),
            ('D', 'G'),
        ]
        self.segment_lines = []
        for color in self.segment_colors:
            lw = 3 if color == 'gray' else 2.5
            line, = self.ax_link.plot([], [], color=color, lw=lw, solid_capstyle='round')
            self.segment_lines.append(line)

        self.joints_plot, = self.ax_link.plot([], [], 'ko', markersize=5, zorder=6)
        self.trail_plot, = self.ax_link.plot([], [], 'b-', linewidth=1.2, alpha=0.45)
        self.label_text = {
            name: self.ax_link.text(0, 0, name, fontsize=9)
            for name in 'ABCDEFGH'
        }

        self.ax_torque.plot(self.theta_abs, self.torque, label='Virtual Work')
        self.ax_torque.plot(self.theta_abs, self.torque_ne, '--', label='Newton-Euler', alpha=0.75)
        torque_limit = self.results.get(
            'motor_torque_limit_display', self.results.get('motor_torque_limit')
        )
        if torque_limit is not None:
            self.ax_torque.axhline(torque_limit, color='r', linestyle=':', alpha=0.8, label='Motor Limit')
            self.ax_torque.axhline(-torque_limit, color='r', linestyle=':', alpha=0.8)
        torque_nominal = self.results.get(
            'motor_torque_nominal_display', self.results.get('motor_torque_nominal')
        )
        if torque_nominal is not None:
            self.ax_torque.axhline(torque_nominal, color='orange', linestyle='--', alpha=0.8, label='Nominal')
            self.ax_torque.axhline(-torque_nominal, color='orange', linestyle='--', alpha=0.8)
        self.torque_cursor = self.ax_torque.axvline(self.valid_angles[0], color='k', linewidth=1.5, alpha=0.8)
        self.ax_torque.set_ylabel(f'Torque ({self.torque_unit})')
        self.ax_torque.set_title('Motor Torque')
        self.ax_torque.grid(True, alpha=0.3)
        self.ax_torque.legend(loc='best')

        for name, mag in self.results['joint_force_mag'].items():
            self.ax_force.plot(self.theta_abs, mag, label=name)
        self.force_cursor = self.ax_force.axvline(self.valid_angles[0], color='k', linewidth=1.5, alpha=0.8)
        self.ax_force.set_ylabel(f'Force ({self.force_unit})')
        self.ax_force.set_title('Joint Reaction Forces')
        self.ax_force.grid(True, alpha=0.3)
        self.ax_force.legend(loc='best', fontsize=8, ncol=2)

        self.ax_coupler.plot(self.theta_abs, self.coupler_x, label=f'E.x ({self.length_unit})')
        self.ax_coupler.plot(self.theta_abs, self.coupler_y, label=f'E.y ({self.length_unit})')
        self.coupler_cursor = self.ax_coupler.axvline(self.valid_angles[0], color='k', linewidth=1.5, alpha=0.8)
        self.coupler_x_marker, = self.ax_coupler.plot([], [], 'o', color='#1f77b4', markersize=6)
        self.coupler_y_marker, = self.ax_coupler.plot([], [], 'o', color='#ff7f0e', markersize=6)
        self.coupler_info = self.ax_coupler.text(
            0.02,
            0.95,
            '',
            transform=self.ax_coupler.transAxes,
            va='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.8),
        )
        self.ax_coupler.set_xlabel(f'Crank Angle ({self.angle_label})')
        self.ax_coupler.set_ylabel(f'Position ({self.length_unit})')
        self.ax_coupler.set_title('Coupler Point E')
        self.ax_coupler.grid(True, alpha=0.3)
        self.ax_coupler.legend(loc='best')

    def _nearest_valid_pos(self, slider_angle: float) -> int:
        return int(np.argmin(np.abs(self.valid_angles - slider_angle)))

    def _on_slider_changed(self, slider_angle):
        valid_pos = self._nearest_valid_pos(slider_angle)
        self._update_by_valid_pos(valid_pos)

    def _on_play_pause(self, _event):
        self.is_playing = not self.is_playing
        self.play_button.label.set_text('Pause' if self.is_playing else 'Play')
        if self.is_playing:
            self.timer.start()
        else:
            self.timer.stop()

    def _on_speed_changed(self, speed):
        desired = self.base_interval_ms / max(speed, 1e-6)
        if desired >= self.min_interval_ms:
            self.frames_per_tick = 1
            self.timer.interval = int(round(desired))
        else:
            self.frames_per_tick = max(1, int(round(self.min_interval_ms / desired)))
            self.timer.interval = self.min_interval_ms

    def _on_timer_tick(self):
        if not self.is_playing:
            return
        next_pos = (
            self.current_valid_pos + self.frames_per_tick
        ) % self.valid_indices.size
        next_angle = float(self.valid_angles[next_pos])
        self.angle_slider.set_val(next_angle)

    def _update_by_valid_pos(self, valid_pos: int):
        self.current_valid_pos = valid_pos
        idx = int(self.valid_indices[valid_pos])
        pos = self.positions[idx]
        angle = float(self.theta_abs[idx])

        for line, (j1, j2) in zip(self.segment_lines, self.segment_defs):
            p1, p2 = pos[j1], pos[j2]
            line.set_data([p1[0], p2[0]], [p1[1], p2[1]])

        pts = np.array([pos[k] for k in 'ABCDEFGH'])
        self.joints_plot.set_data(pts[:, 0], pts[:, 1])
        label_offset = max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])) * 0.015
        label_offset = max(label_offset, 0.15)
        for name in 'ABCDEFGH':
            p = pos[name]
            self.label_text[name].set_position((p[0] + label_offset, p[1] + label_offset))

        trail_positions = [self.positions[i]['E'] for i in self.valid_indices[: valid_pos + 1]]
        trail = np.array(trail_positions)
        self.trail_plot.set_data(trail[:, 0], trail[:, 1])

        for cursor in (self.torque_cursor, self.force_cursor, self.coupler_cursor):
            cursor.set_xdata([angle, angle])

        self.coupler_x_marker.set_data([angle], [self.coupler_x[idx]])
        self.coupler_y_marker.set_data([angle], [self.coupler_y[idx]])
        self.coupler_info.set_text(
            f"Angle: {angle:.2f} {self.angle_label}\n"
            f"E.x: {self.coupler_x[idx]:.3f} {self.length_unit}\n"
            f"E.y: {self.coupler_y[idx]:.3f} {self.length_unit}"
        )

        self.ax_link.set_title(f'Linkage Configuration @ {angle:.2f} {self.angle_label}')
        self.fig.canvas.draw_idle()

    def show(self):
        plt.show()


def _print_run_summary(sim: SixBarSim, results: dict) -> None:
    torque_vals = results.get('torque_vw_display', results['torque_vw'])
    finite_torque = torque_vals[np.isfinite(torque_vals)]
    if finite_torque.size == 0:
        print("No valid torque values were computed.")
        return

    positive_torque = finite_torque[finite_torque > 0]
    peak_torque = float(np.max(positive_torque)) if positive_torque.size else 0.0
    torque_unit = results.get('torque_unit', sim._native_torque_unit())
    print(f"Peak motor torque: {peak_torque:.3f} {torque_unit}")

    cycle_time = results.get('cycle_time_s')
    if cycle_time is not None:
        print(f"Cycle time at configured speed: {cycle_time:.3f} s")

    torque_limit = results.get(
        'motor_torque_limit_display', results.get('motor_torque_limit')
    )
    if torque_limit is not None:
        native_vals = results['torque_vw'][np.isfinite(results['torque_vw'])]
        positive_native = native_vals[native_vals > 0]
        peak_native = float(np.max(positive_native)) if positive_native.size else 0.0
        limit_native = results.get('motor_torque_limit_native', torque_limit)
        status = "OK" if peak_native <= limit_native else "EXCEEDS LIMIT"
        print(
            f"Motor torque limit: {torque_limit:.3f} {torque_unit} "
            f"({status})"
        )

    torque_nominal = results.get(
        'motor_torque_nominal_display', results.get('motor_torque_nominal')
    )
    if torque_nominal is not None:
        nom_status = "OK" if peak_torque <= torque_nominal else "EXCEEDS NOMINAL"
        print(
            f"Motor torque nominal: {torque_nominal:.3f} {torque_unit} "
            f"({nom_status})"
        )


def _export_dimensions_md(
    sim: 'SixBarSim',
    mechanism_path: str,
    settings_path: str,
    settings: SimulationSettings,
    effective_length_scale: float,
) -> str:
    mech_stem = Path(mechanism_path).stem
    mech_name = Path(mechanism_path).name
    unit = settings.length_unit

    if settings.autoscale and settings.autoscale_target_dy is not None:
        scale_note = (
            f"`{settings_path}` ("
            f"`autoscale.target_delta_y = {settings.autoscale_target_dy:g} {unit}`, "
            f"applied scale `{effective_length_scale:.6g}\u00d7`)"
        )
    elif effective_length_scale != 1.0:
        scale_note = (
            f"`{settings_path}` (applied scale `{effective_length_scale:.6g}\u00d7`)"
        )
    else:
        scale_note = f"`{settings_path}`"

    L_BC = np.linalg.norm(sim.C - sim.B)
    L_CD = np.linalg.norm(sim.D - sim.C)
    L_BD = np.linalg.norm(sim.D - sim.B)
    angle_unit = settings.angle_unit
    def to_angle(rad):
        return np.degrees(rad) if angle_unit == 'deg' else rad
    angle_label = angle_unit
    ang_BC = to_angle(np.arctan2(sim.C[1] - sim.B[1], sim.C[0] - sim.B[0]))
    ang_CD = to_angle(np.arctan2(sim.D[1] - sim.C[1], sim.D[0] - sim.C[0]))

    def f(v):
        return f"{v:.2f}"

    lines = [
        f"# {mech_stem} Link Dimensions",
        "",
        f"Source: `{mech_name}` with {scale_note}. "
        f"All lengths are centerline distances between joint axes, in {unit}.",
        "",
        "## Link dimensions",
        "",
        f"| Link | Type | Role | Edge | Length ({unit}) |",
        "|---|---|---|---|---|",
        f"| **L1** | binary | Crank (motor at B) | A\u2013B | {f(sim.L_BA)} |",
        f"| **L2** | ternary | Ground frame | B\u2013C | {f(L_BC)} |",
        f"| | | | C\u2013D | {f(L_CD)} |",
        f"| | | | B\u2013D | {f(L_BD)} |",
        f"| **L3** | ternary | Coupler (E = output) | E\u2013F | {f(sim.L_FE)} |",
        f"| | | | F\u2013G | {f(sim.L_FG)} |",
        f"| | | | E\u2013G | {f(sim.L_GE)} |",
        f"| **L4** | binary | Rocker (loop 2) | D\u2013G | {f(sim.L_DG)} |",
        f"| **L5** | ternary | Connects both loops | C\u2013F | {f(sim.L_CF)} |",
        f"| | | | F\u2013H | {f(sim.L_FH)} |",
        f"| | | | C\u2013H | {f(sim.L_CH)} |",
        f"| **L6** | binary | Rocker (loop 1) | A\u2013H | {f(sim.L_AH)} |",
        "",
        "For each ternary link, sketch a triangle using the three side lengths "
        "(SolidWorks solves SSS uniquely). Drill joint holes at the triangle vertices.",
        "",
        "## Ground-frame pivot spacing (for the base plate / chassis)",
        "",
        "L2 is fixed; use these to locate the motor axis (B) and the two idler pivots (C, D).",
        "",
        f"| From | To | Distance ({unit}) | Angle from horizontal ({angle_label}) |",
        "|---|---|---|---|",
        f"| B | C | {f(L_BC)} | {f(ang_BC)} |",
        f"| C | D | {f(L_CD)} | {f(ang_CD)} |",
        f"| B | D | {f(L_BD)} | — |",
        "",
        "## Joint topology reference",
        "",
        "```",
        "L1  (crank)   : B -> A          motor at B",
        "L2  (ground)  : B - C - D      ternary, fixed frame",
        "L3  (coupler) : E - F - G      ternary, E = wheel/output point",
        "L4            : D -> G          binary",
        "L5            : C - F - H      ternary, connects both loops",
        "L6            : A -> H          binary",
        "",
        "Loop 1:  B - A - H - C    (ground segment B-C)",
        "Loop 2:  C - F - G - D    (ground segment C-D)",
        "```",
        "",
        "## Notes",
        "",
        "- These are centerline distances only \u2014 add material around each joint "
        "for bearings/fasteners (typically 1.5\u20132\u00d7 bolt diameter as edge margin).",
        "- Link thickness is a design choice; the simulator treats links as rigid 2D bodies.",
        f"- Regenerate this table by running "
        f"`python sixbar_sim.py inputs/{mech_name} --export-dimensions outputs/{mech_stem}_dimensions.md --no-gui` "
        f"after any geometry or autoscale change.",
        "",
    ]
    return "\n".join(lines)


def _md_to_pdf(md_text: str, pdf_path: str) -> None:
    try:
        from fpdf import FPDF
        from fpdf.enums import XPos, YPos
    except ImportError as exc:
        raise ImportError(
            "fpdf2 is required for PDF export. "
            "Install it with: pip install fpdf2"
        ) from exc

    import re as _re

    def _clean(text):
        return (
            text
            .replace('\u2013', '-')
            .replace('\u2014', '--')
            .replace('\u2192', '->')
            .replace('\u00d7', 'x')
            .replace('\u0394', 'delta')
            .replace('\u2019', "'")
            .replace('\u2018', "'")
        ).encode('latin-1', errors='replace').decode('latin-1')

    H1_COLOR = (10, 40, 100)
    H2_COLOR = (20, 60, 140)
    H3_COLOR = (40, 80, 160)
    HEAD_BG  = (200, 220, 245)
    ROW_ALT  = (235, 242, 252)
    CODE_BG  = (240, 240, 240)

    MARGIN = 20
    CONTENT_W = 210 - 2 * MARGIN
    COL5 = [18, 20, 60, 28, 44]   # Link | Type | Role | Edge | Length
    COL3 = [50, 50, 70]            # From | To | Distance

    pdf = FPDF()
    pdf.add_page()
    pdf.set_margins(MARGIN, MARGIN, MARGIN)
    pdf.set_auto_page_break(auto=True, margin=MARGIN)

    def _parse_table_rows(raw_rows):
        rows = []
        for r in raw_rows:
            if _re.match(r'^\|[-| ]+\|$', r.strip()):
                continue
            cells = [c.strip() for c in r.strip().strip('|').split('|')]
            rows.append(cells)
        return rows

    def _draw_table(raw_rows):
        rows = _parse_table_rows(raw_rows)
        if not rows:
            return
        ncols = len(rows[0])
        cols = COL5 if ncols >= 5 else COL3
        if len(cols) < ncols:
            cols = [CONTENT_W // ncols] * ncols

        for ri, row in enumerate(rows):
            is_header = ri == 0
            pdf.set_font('Helvetica', 'B' if is_header else '', 9)
            pdf.set_fill_color(*(HEAD_BG if is_header else (ROW_ALT if ri % 2 == 0 else (255, 255, 255))))
            for ci, cell in enumerate(row):
                w = cols[ci] if ci < len(cols) else 20
                cell_text = _re.sub(r'\*\*([^*]+)\*\*', r'\1', cell)
                align = 'R' if ci == len(row) - 1 and ri > 0 else 'L'
                pdf.cell(w, 6, _clean(cell_text), border=1, align=align, fill=True,
                         new_x=XPos.RIGHT, new_y=YPos.TOP)
            pdf.ln()
        pdf.ln(3)

    def _render_inline(text):
        parts = _re.split(r'(\*\*[^*]+\*\*|`[^`]+`)', text)
        for part in parts:
            if part.startswith('**') and part.endswith('**'):
                pdf.set_font('Helvetica', 'B', 10)
                pdf.write(5, _clean(part[2:-2]))
                pdf.set_font('Helvetica', '', 10)
            elif part.startswith('`') and part.endswith('`'):
                pdf.set_font('Courier', '', 9)
                pdf.write(5, _clean(part[1:-1]))
                pdf.set_font('Helvetica', '', 10)
            else:
                pdf.write(5, _clean(part))

    table_buf: list[str] = []
    code_buf: list[str] = []
    in_code = False

    def _flush_table():
        if table_buf:
            _draw_table(table_buf)
            table_buf.clear()

    def _flush_code():
        if code_buf:
            pdf.set_font('Courier', '', 8.5)
            pdf.set_fill_color(*CODE_BG)
            for cl in code_buf:
                pdf.cell(CONTENT_W, 5, _clean(cl), fill=True,
                         new_x=XPos.LMARGIN, new_y=YPos.NEXT)
            code_buf.clear()
            pdf.ln(2)

    for line in md_text.splitlines():
        s = line.rstrip()

        if s.startswith('```'):
            if in_code:
                _flush_code()
                in_code = False
            else:
                _flush_table()
                in_code = True
            continue

        if in_code:
            code_buf.append(s)
            continue

        if s.startswith('|'):
            table_buf.append(s)
            continue
        else:
            _flush_table()

        if s.startswith('# '):
            pdf.set_font('Helvetica', 'B', 16)
            pdf.set_text_color(*H1_COLOR)
            pdf.cell(0, 10, _clean(s[2:]), new_x=XPos.LMARGIN, new_y=YPos.NEXT)
            pdf.set_line_width(0.8)
            pdf.set_draw_color(*H1_COLOR)
            pdf.line(pdf.l_margin, pdf.get_y(), pdf.w - pdf.r_margin, pdf.get_y())
            pdf.ln(2)
            pdf.set_text_color(0, 0, 0)
            pdf.set_draw_color(0, 0, 0)
        elif s.startswith('## '):
            pdf.ln(2)
            pdf.set_font('Helvetica', 'B', 13)
            pdf.set_text_color(*H2_COLOR)
            pdf.cell(0, 8, _clean(s[3:]), new_x=XPos.LMARGIN, new_y=YPos.NEXT)
            pdf.set_line_width(0.4)
            pdf.set_draw_color(*H2_COLOR)
            pdf.line(pdf.l_margin, pdf.get_y(), pdf.w - pdf.r_margin, pdf.get_y())
            pdf.ln(2)
            pdf.set_text_color(0, 0, 0)
            pdf.set_draw_color(0, 0, 0)
        elif s.startswith('### '):
            pdf.ln(1)
            pdf.set_font('Helvetica', 'B', 11)
            pdf.set_text_color(*H3_COLOR)
            pdf.cell(0, 7, _clean(s[4:]), new_x=XPos.LMARGIN, new_y=YPos.NEXT)
            pdf.set_text_color(0, 0, 0)
        elif s.startswith('- '):
            pdf.set_font('Helvetica', '', 10)
            pdf.set_x(pdf.l_margin + 4)
            pdf.write(5, _clean('- '))
            _render_inline(s[2:])
            pdf.ln()
        elif s == '' or s == '---':
            pdf.ln(3)
        else:
            pdf.set_font('Helvetica', '', 10)
            _render_inline(s)
            pdf.ln()

    _flush_table()
    _flush_code()
    pdf.output(pdf_path)


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'mechanism',
        nargs='?',
        default='mechanism.json',
        help='Path to a .motiongen or mechanism .json file.',
    )
    parser.add_argument(
        '-s', '--settings',
        default='settings.json',
        help='Path to the simulation settings JSON file.',
    )
    parser.add_argument(
        '--export-gif',
        help='Export an animation GIF to this path (must end with .gif). '
             'Overrides settings.export_gif_path.',
    )
    parser.add_argument(
        '--gif-fps',
        type=float,
        help='GIF frame rate override (frames per second).',
    )
    parser.add_argument(
        '--no-gui',
        action='store_true',
        help='Run without opening the interactive viewer window.',
    )
    parser.add_argument(
        '--export-dimensions',
        metavar='PATH',
        nargs='?',
        const='',
        help='Export link dimensions to a Markdown file. '
             'Omit PATH to auto-name as <mechanism_stem>_dimensions.md.',
    )
    return parser


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    args = _build_arg_parser().parse_args()
    if not Path(args.mechanism).exists():
        candidate = Path("inputs") / Path(args.mechanism).name
        if candidate.exists():
            args.mechanism = str(candidate)
    settings_path = Path(args.settings)
    if settings_path.exists():
        settings = SimulationSettings.load_json(str(settings_path))
        print(f"Loaded settings: {settings_path}")
    else:
        settings = SimulationSettings()
        print(f"Settings file not found, using built-in defaults: {settings_path}")

    if settings.start_angle is None and settings.end_angle is None:
        theta_start_override = None
        theta_end_override = None
    elif settings.start_angle is None or settings.end_angle is None:
        raise ValueError(
            "Set both start_angle and end_angle, or leave both unset."
        )
    else:
        theta_start_override = SixBarSim._angle_to_radians(
            settings.start_angle, settings.angle_unit
        )
        theta_end_override = SixBarSim._angle_to_radians(
            settings.end_angle, settings.angle_unit
        )

    effective_length_scale = settings.length_scale
    if settings.autoscale:
        probe = SixBarSim(
            args.mechanism,
            length_scale=1.0,
            target_length_unit=settings.length_unit,
            target_angle_unit=settings.angle_unit,
        )
        dy_unit = probe.measure_E_y_travel(
            settings.n_steps, theta_start_override, theta_end_override
        )
        effective_length_scale = settings.autoscale_target_dy / dy_unit
        print(
            f"Autoscale: dY at scale 1.0 = {dy_unit:.4f} {settings.length_unit}, "
            f"target = {settings.autoscale_target_dy:g} {settings.length_unit}, "
            f"length_scale = {effective_length_scale:.6g}"
        )

    sim = SixBarSim(
        args.mechanism,
        length_scale=effective_length_scale,
        target_length_unit=settings.length_unit,
        target_angle_unit=settings.angle_unit,
    )
    print(sim.mech.summary())
    initial_angle = SixBarSim._angle_values(sim.theta1_init, settings.angle_unit)
    angle_label = SixBarSim._angle_axis_label(settings.angle_unit)
    print(f"\nInitial crank angle: {initial_angle:.1f} {angle_label}")
    print(f"Grashof check (Loop 1): "
          f"shortest + longest = {sim.L_BA + sim.L_CH:.4f}, "
          f"other two = {sim.L_AH + np.linalg.norm(sim.C - sim.B):.4f}")
    if effective_length_scale != 1.0:
        print(f"Applied length scale: {effective_length_scale:.6g}x")

    results = sim.run_with_settings(settings)
    _print_run_summary(sim, results)

    if args.export_dimensions is not None:
        out_path = Path(args.export_dimensions) if args.export_dimensions else (
            Path("outputs") / (Path(args.mechanism).stem + "_dimensions.md")
        )
        out_path.parent.mkdir(parents=True, exist_ok=True)
        md = _export_dimensions_md(
            sim, args.mechanism, args.settings, settings, effective_length_scale
        )
        out_path.write_text(md, encoding='utf-8')
        print(f"Exported dimensions: {out_path}")
        pdf_path = str(Path(out_path).with_suffix('.pdf'))
        _md_to_pdf(md, pdf_path)
        print(f"Exported dimensions PDF: {pdf_path}")

    gif_path = args.export_gif or settings.export_gif_path
    gif_fps = args.gif_fps if args.gif_fps is not None else settings.export_gif_fps
    if gif_fps <= 0:
        raise ValueError("GIF fps must be greater than 0")
    if gif_path:
        sim.animate(results, save_path=gif_path, fps=gif_fps)

    if args.no_gui:
        sys.exit(0)

    if not _ensure_interactive_backend():
        active_backend = plt.get_backend()
        qt_msg = (
            "Qt bindings detected, but GUI cannot start in this headless session.\n"
            if _has_qt_binding()
            else ""
        )
        print(
            "Interactive viewer needs a GUI Matplotlib backend.\n"
            f"Current backend: {active_backend}\n"
            f"{qt_msg}"
            "Install one of:\n"
            "  - python3-tk (TkAgg)\n"
            "  - PyQt5/PyQt6/PySide6 (QtAgg)\n"
            "And run from a desktop session (X11/Wayland), not headless.\n"
            "Or use --no-gui --export-gif <path.gif> for headless export."
        )
        sys.exit(1)

    viewer = SixBarInteractiveViewer(sim, settings, results)
    viewer.show()
