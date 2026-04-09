# Six-Bar Linkage Simulation

Quasi-static kinematic and force analysis for a Stephenson six-bar leg linkage. Loads mechanism geometry directly from [MotionGen](https://motiongen.io/) export files.

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

```bash
python sixbar_sim.py path/to/mechanism.motiongen
```

This runs the simulation with placeholder masses and loads, generates plots, and displays them.

### As a library

```python
from sixbar_sim import SixBarSim
import numpy as np

sim = SixBarSim("path/to/mechanism.motiongen")

# Define link masses (lbm) and external load at wheel point (lbf)
masses = {"L1": 0.1, "L3": 0.5, "L4": 0.2, "L5": 0.3, "L6": 0.15}
F_ext = np.array([0.0, -5.0])

results = sim.run(
    n_steps=720,
    link_masses=masses,
    gravity=386.1,        # in/s^2 (use 9810 for mm)
    ext_force_E=F_ext,
)

sim.plot_torque(results)
sim.plot_joint_forces(results)
sim.plot_coupler_curve(results)
sim.animate(results, save_path="linkage.gif")
```

### MotionGen parser standalone

```python
from motiongen_parser import load_motiongen

mech = load_motiongen("mechanism.motiongen")
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
motiongen_parser.py   # MotionGen JSON → Python data structures
sixbar_sim.py         # Kinematics, force analysis, plotting, animation
```
