# 6bar6_1 Link Dimensions

Source: `6bar6_1.motiongen` with `settings.json` (`autoscale.target_delta_y = 250 mm`, applied scale `0.827011×`). All lengths are centerline distances between joint axes, in millimeters.

## Link dimensions

| Link | Type | Role | Edge | Length (mm) |
|---|---|---|---|---|
| **L1** | binary | Crank (motor at B) | A–B | 36.20 |
| **L2** | ternary | Ground frame | B–C | 54.70 |
| | | | C–D | 30.18 |
| | | | B–D | 82.97 |
| **L3** | ternary | Coupler (E = output) | E–F | 141.80 |
| | | | F–G | 26.07 |
| | | | E–G | 126.15 |
| **L4** | binary | Rocker (loop 2) | D–G | 168.89 |
| **L5** | ternary | Connects both loops | C–F | 160.58 |
| | | | F–H | 87.27 |
| | | | C–H | 73.32 |
| **L6** | binary | Rocker (loop 1) | A–H | 64.47 |

For each ternary link, sketch a triangle using the three side lengths (SolidWorks solves SSS uniquely). Drill joint holes at the triangle vertices.

## Ground-frame pivot spacing (for the base plate / chassis)

L2 is fixed; use these to locate the motor axis (B) and the two idler pivots (C, D).

| From | To | Distance (mm) |
|---|---|---|
| B | C | 54.70 |
| C | D | 30.18 |
| B | D | 82.97 |

## Joint topology reference

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

## Notes

- These are centerline distances only — add material around each joint for bearings/fasteners (typically 1.5–2× bolt diameter as edge margin).
- Link thickness is a design choice; the simulator treats links as rigid 2D bodies.
- Regenerate this table by running `python sixbar_sim.py 6bar6_1.motiongen --no-gui` after any geometry or autoscale change.
