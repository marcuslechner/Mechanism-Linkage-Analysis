# Six-Bar Linkage Simulation

Quasi-static kinematic and force analysis for a Stephenson six-bar leg linkage. Loads mechanism geometry from [MotionGen](https://motiongen.io/) export files or from an intermediate JSON format that can be hand-edited.

## What it computes

- **Position kinematics** — all joint positions vs crank angle (loop-closure via circle-circle intersection)
- **Velocity kinematics** — angular velocities of all links
- **Motor torque** — via principle of virtual work (validated against Newton-Euler)
- **Joint reaction forces** — full Newton-Euler static equilibrium on each link
- **Coupler curve** — path traced by the wheel/output point
- **Animation** — animated linkage motion through full crank rotation

## Setup

```bash
python3 -m venv venv
source venv/bin/activate
pip install pydy sympy numpy scipy matplotlib
```

## Usage

The workflow has two stages that can be run together or separately:

1. **Parse** a `.motiongen` file into a human-readable JSON
2. **Simulate** from the JSON (or directly from the `.motiongen`)

This lets you export from MotionGen once, then tweak joint positions, link lengths, or actuator parameters in the JSON without re-exporting.

### Parse to JSON

```bash
python motiongen_parser.py path/to/mechanism.motiongen -o mechanism.json
```

Edit `mechanism.json` in any text editor to adjust geometry, then run the simulation from it.

### Run the simulation

```bash
# Interactive linkage + synced charts (slider, play/pause, speed)
python sixbar_sim.py mechanism.json

# Works directly from a .motiongen file too
python sixbar_sim.py path/to/mechanism.motiongen

# Use a custom settings file
python sixbar_sim.py mechanism.json --settings path/to/settings.json

# Headless GIF export (no GUI required)
python sixbar_sim.py mechanism.json --no-gui --export-gif linkage.gif
```

This reads `settings.json` by default for masses, payload, motor limits, and other run settings, runs the full kinematics/dynamics, and opens an interactive viewer.
The mechanism is converted to the configured output units even if the input `.motiongen` was authored in a different unit system.
If `export.gif_path` is set in settings (or `--export-gif` is passed), the run also writes an animation GIF.

The viewer includes:

- Absolute crank-angle slider (drag to inspect a configuration)
- Linkage motion panel with coupler trail
- Synchronized cursors across torque, joint-force, and coupler charts
- Play/Pause and playback speed control
- Optional GIF export via `--export-gif` (or `settings.json` `export.gif_path`)

If the run prints `Current backend: agg`, install a GUI backend first:

- Ubuntu/Debian: `sudo apt install python3-tk` (TkAgg)
- Or install Qt bindings in your venv: `pip install PyQt5`

### Settings file

`settings.json` drives the simulation inputs that were previously hardcoded in the script:

- `units.length`, `units.torque`, and `units.angle` for output units
- `length_scale` for uniformly scaling imported mechanism geometry
- `payload.mass_kg` and `payload.direction`
- `motor.speed_deg_per_s` and `motor.torque_limit`
- `export.gif_path` and `export.gif_fps`
- `link_masses`
- `n_steps`, `start_angle`, `end_angle`, and `gravity`

The included default file is:

```json
{
  "units": {
    "length": "mm",
    "torque": "N.m",
    "angle": "deg"
  },
  "n_steps": 720,
  "start_angle": null,
  "end_angle": null,
  "gravity": 9.80665,
  "length_scale": 1.0,
  "motor": {
    "speed_deg_per_s": 10.0,
    "torque_limit": 5.0
  },
  "export": {
    "gif_path": null,
    "gif_fps": 30.0
  },
  "payload": {
    "mass_kg": 2.267962,
    "direction": [0.0, -1.0]
  },
  "link_masses": {
    "L1": 0.045359,
    "L3": 0.226796,
    "L4": 0.090718,
    "L5": 0.136078,
    "L6": 0.068039
  }
}
```

`start_angle` and `end_angle` are in `units.angle`. Set both to simulate a partial sweep.

For backward compatibility, `payload.weight` (in N) is still accepted if `payload.mass_kg` is omitted.

Supported unit values:

- `units.length`: `mm`, `cm`, `m`, `in`, `ft`
- `units.torque`: `N.m`, `N.mm`, `lbf.in`, `lbf.ft`
- `units.angle`: `deg`, `rad`

### As a library

```python
from sixbar_sim import SixBarSim, SimulationSettings

# Accepts either a .motiongen or .json file
settings = SimulationSettings.load_json("settings.json")
sim = SixBarSim(
    "mechanism.json",
    target_length_unit=settings.length_unit,
    target_angle_unit=settings.angle_unit,
    length_scale=settings.length_scale,
)

results = sim.run_with_settings(settings)

sim.plot_torque(results)
sim.plot_joint_forces(results)
sim.plot_coupler_curve(results)
sim.animate(results, save_path="linkage.gif")
```

### MotionGen parser standalone

```python
from motiongen_parser import load_motiongen, Mechanism

# Parse and export
mech = load_motiongen("mechanism.motiongen")
mech.save_json("mechanism.json")

# Load back from JSON
mech = Mechanism.load_json("mechanism.json")
print(mech.summary())
```

## Mechanism topology

```
L1  (crank)   : B → A          motor at B
L2  (ground)  : B – C – D      ternary, fixed frame
L3  (coupler) : E – F – G      ternary, E = wheel/output point
L4            : D → G          binary
L5            : C – F – H      ternary, connects both loops
L6            : A → H          binary

Loop 1:  B – A – H – C    (ground segment B–C)
Loop 2:  C – F – G – D    (ground segment C–D)
```

## File structure

```
settings.json         # Simulation inputs: payload, motor, masses, sweep
motiongen_parser.py   # MotionGen → intermediate JSON → Python data structures
sixbar_sim.py         # Kinematics, force analysis, interactive viewer, GIF export
mechanism.json        # Intermediate JSON (generated, editable)
```
