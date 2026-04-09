"""
Parser for MotionGen .motiongen JSON files.

Extracts joint positions, link connectivity, ground links, actuator
definitions, and units into a clean data structure for downstream
simulation (PyDy, etc.).
"""

import json
import math
from dataclasses import dataclass, field


@dataclass
class Joint:
    id: str
    label: str
    x: float
    y: float


@dataclass
class Link:
    id: str
    label: str
    joint_ids: list[str]
    is_ground: bool


@dataclass
class Actuator:
    id: str
    type: str
    at_joint_id: str      # pivot joint (where motor sits)
    from_joint_id: str    # reference joint for angle start
    to_joint_id: str      # driven joint
    min_angle: float      # rad or deg depending on angular_unit
    max_angle: float
    velocity: float


@dataclass
class LinkLength:
    joint_a_id: str
    joint_b_id: str
    length: float


@dataclass
class Mechanism:
    joints: dict[str, Joint]
    links: dict[str, Link]
    actuators: list[Actuator]
    link_lengths: list[LinkLength]
    linear_unit: str
    angular_unit: str
    ground_link_id: str | None

    def get_joint_by_label(self, label: str) -> Joint | None:
        for j in self.joints.values():
            if j.label == label:
                return j
        return None

    def get_link_by_label(self, label: str) -> Link | None:
        for l in self.links.values():
            if l.label == label:
                return l
        return None

    def get_joint_position(self, joint_id: str) -> tuple[float, float]:
        j = self.joints[joint_id]
        return (j.x, j.y)

    def compute_link_length(self, joint_id_a: str, joint_id_b: str) -> float:
        ax, ay = self.get_joint_position(joint_id_a)
        bx, by = self.get_joint_position(joint_id_b)
        return math.sqrt((bx - ax) ** 2 + (by - ay) ** 2)

    def summary(self) -> str:
        lines = []
        lines.append(f"Mechanism: {len(self.joints)} joints, {len(self.links)} links")
        lines.append(f"Units: {self.linear_unit} (linear), {self.angular_unit} (angular)")
        lines.append("")

        lines.append("Joints:")
        for j in self.joints.values():
            lines.append(f"  {j.label}: ({j.x:.4f}, {j.y:.4f})")
        lines.append("")

        lines.append("Links:")
        for l in self.links.values():
            joint_labels = [self.joints[jid].label for jid in l.joint_ids]
            ground_str = " [GROUND]" if l.is_ground else ""
            lines.append(f"  {l.label}: {' - '.join(joint_labels)}{ground_str}")
        lines.append("")

        lines.append("Link lengths:")
        for ll in self.link_lengths:
            la = self.joints[ll.joint_a_id].label
            lb = self.joints[ll.joint_b_id].label
            lines.append(f"  {la}-{lb}: {ll.length:.4f} {self.linear_unit}")
        lines.append("")

        for a in self.actuators:
            at_label = self.joints[a.at_joint_id].label
            to_label = self.joints[a.to_joint_id].label
            from_label = self.joints[a.from_joint_id].label
            lines.append(
                f"Actuator: {a.type} at {at_label}, "
                f"drives {at_label}-{to_label} (ref: {from_label})"
            )

        return "\n".join(lines)


def load_motiongen(filepath: str) -> Mechanism:
    """Load a .motiongen file and return a Mechanism dataclass."""
    with open(filepath) as f:
        data = json.load(f)

    settings = data.get("settings", {})
    linear_unit = settings.get("linearUnit", "unknown")
    angular_unit = settings.get("angularUnit", "unknown")

    mech = data["mechanism"]

    # Parse joints — assign labels A, B, C, ... in order of appearance
    joints = {}
    for i, jdata in enumerate(mech["joints"]):
        label = chr(ord("A") + i)
        joints[jdata["id"]] = Joint(
            id=jdata["id"],
            label=label,
            x=jdata["x"],
            y=jdata["y"],
        )

    # Parse links — assign labels L1, L2, ... in order of appearance
    links = {}
    ground_link_id = None
    for i, ldata in enumerate(mech["links"]):
        label = f"L{i + 1}"
        link = Link(
            id=ldata["id"],
            label=label,
            joint_ids=ldata["jointIds"],
            is_ground=ldata["isGround"],
        )
        links[ldata["id"]] = link
        if ldata["isGround"]:
            ground_link_id = ldata["id"]

    # Parse actuators
    actuators = []
    for adata in mech["actuators"]:
        actuators.append(Actuator(
            id=adata["id"],
            type=adata["type"],
            at_joint_id=adata["at"],
            from_joint_id=adata["from"],
            to_joint_id=adata["to"],
            min_angle=adata["min"],
            max_angle=adata["max"],
            velocity=adata["velocity"],
        ))

    # Compute all link segment lengths
    link_lengths = []
    for link in links.values():
        jids = link.joint_ids
        for k in range(len(jids)):
            for m in range(k + 1, len(jids)):
                ja, jb = joints[jids[k]], joints[jids[m]]
                length = math.sqrt((jb.x - ja.x) ** 2 + (jb.y - ja.y) ** 2)
                link_lengths.append(LinkLength(
                    joint_a_id=jids[k],
                    joint_b_id=jids[m],
                    length=length,
                ))

    return Mechanism(
        joints=joints,
        links=links,
        actuators=actuators,
        link_lengths=link_lengths,
        linear_unit=linear_unit,
        angular_unit=angular_unit,
        ground_link_id=ground_link_id,
    )


if __name__ == "__main__":
    import sys
    filepath = sys.argv[1] if len(sys.argv) > 1 else "/home/marcus/Downloads/6bar.motiongen"
    mech = load_motiongen(filepath)
    print(mech.summary())
