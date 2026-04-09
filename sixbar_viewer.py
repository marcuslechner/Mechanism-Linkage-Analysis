"""
Interactive viewer for six-bar simulation results.

Usage:
    python sixbar_viewer.py mechanism.json --settings settings.json
"""

import argparse
import sys
from pathlib import Path

import matplotlib
import numpy as np
from matplotlib.widgets import Button, Slider

from sixbar_sim import SixBarSim, SimulationSettings

plt = None


def _is_non_interactive_backend(backend_name: str) -> bool:
    name = backend_name.lower()
    interactive_backends = {
        'gtk3agg', 'gtk3cairo',
        'gtk4agg', 'gtk4cairo',
        'macosx',
        'nbagg', 'notebook',
        'qtagg', 'qtcairo', 'qt5agg', 'qt5cairo',
        'tkagg', 'tkcairo',
        'webagg',
        'wx', 'wxagg', 'wxcairo',
    }
    if name in interactive_backends:
        return False

    return name in {
        'agg',
        'cairo',
        'pdf',
        'pgf',
        'ps',
        'svg',
        'template',
        'module://matplotlib_inline.backend_inline',
    }


def _configure_interactive_backend():
    # Respect an explicit backend set by user env/rc unless it's non-interactive.
    backend = matplotlib.get_backend()
    if not _is_non_interactive_backend(backend):
        return

    # Try Tk first (usually simplest on Linux).
    try:
        import tkinter  # noqa: F401
        matplotlib.use('TkAgg', force=True)
        return
    except Exception:
        pass

    # Try Qt if bindings are available.
    for module_name in ('PyQt6', 'PySide6', 'PyQt5', 'PySide2'):
        try:
            __import__(module_name)
            matplotlib.use('QtAgg', force=True)
            return
        except Exception:
            continue


def _has_qt_binding() -> bool:
    for module_name in ('PyQt6', 'PySide6', 'PyQt5', 'PySide2'):
        try:
            __import__(module_name)
            return True
        except Exception:
            continue
    return False


class SixBarInteractiveViewer:
    def __init__(self, sim: SixBarSim, settings: SimulationSettings, results: dict):
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
        self.base_interval_ms = 45

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
            valmax=4.0,
            valinit=1.0,
        )
        self.speed_slider.on_changed(self._on_speed_changed)

        self.timer = self.fig.canvas.new_timer(interval=self.base_interval_ms)
        self.timer.add_callback(self._on_timer_tick)

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
        interval = max(10, int(self.base_interval_ms / speed))
        self.timer.interval = interval

    def _on_timer_tick(self):
        if not self.is_playing:
            return
        next_pos = (self.current_valid_pos + 1) % self.valid_indices.size
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


def _build_arg_parser():
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
        help='Export an animation GIF to this path (must end with .gif).',
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
    return parser


if __name__ == '__main__':
    args = _build_arg_parser().parse_args()
    settings_path = Path(args.settings)
    if settings_path.exists():
        settings = SimulationSettings.load_json(str(settings_path))
        print(f"Loaded settings: {settings_path}")
    else:
        settings = SimulationSettings()
        print(f"Settings file not found, using built-in defaults: {settings_path}")

    sim = SixBarSim(
        args.mechanism,
        target_length_unit=settings.length_unit,
        target_angle_unit=settings.angle_unit,
        length_scale=settings.length_scale,
    )
    results = sim.run_with_settings(settings)

    gif_path = args.export_gif or settings.export_gif_path
    gif_fps = args.gif_fps if args.gif_fps is not None else settings.export_gif_fps
    if gif_fps <= 0:
        raise ValueError("GIF fps must be greater than 0")
    if gif_path:
        sim.animate(results, save_path=gif_path, fps=gif_fps)

    if args.no_gui:
        sys.exit(0)

    _configure_interactive_backend()
    active_backend = matplotlib.get_backend()
    if _is_non_interactive_backend(active_backend):
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
            "Or use --no-gui --export-gif <path.gif> for headless export.\n"
            "Then rerun sixbar_viewer.py."
        )
        sys.exit(1)

    import matplotlib.pyplot as plt  # noqa: PLW0603

    viewer = SixBarInteractiveViewer(sim, settings, results)
    viewer.show()
