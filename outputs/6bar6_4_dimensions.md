# 6bar6_4 Link Dimensions

Source: `6bar6_4.motiongen` with `settings.json` (`autoscale.target_delta_y = 250 mm`, applied scale `0.835713×`). All lengths are centerline distances between joint axes, in millimeters.

## Link dimensions

| Link | Type | Role | Edge | Length (mm) |
|---|---|---|---|---|
| **L1** | binary | Crank (motor at B) | A–B | 36.58 |
| **L2** | ternary | Ground frame | B–C | 55.28 |
| | | | C–D | 30.49 |
| | | | B–D | 83.84 |
| **L3** | ternary | Coupler (E = output) | E–F | 174.55 |
| | | | F–G | 26.35 |
| | | | E–G | 160.91 |
| **L4** | binary | Rocker (loop 2) | D–G | 160.08 |
| **L5** | ternary | Connects both loops | C–F | 151.67 |
| | | | F–H | 77.62 |
| | | | C–H | 74.09 |
| **L6** | binary | Rocker (loop 1) | A–H | 65.15 |

For each ternary link, sketch a triangle using the three side lengths (SolidWorks solves SSS uniquely). Drill joint holes at the triangle vertices.

## Ground-frame pivot spacing (for the base plate / chassis)

L2 is fixed; use these to locate the motor axis (B) and the two idler pivots (C, D).

| From | To | Distance (mm) |
|---|---|---|
| B | C | 55.28 |
| C | D | 30.49 |
| B | D | 83.84 |

## Joint topology reference

```
L1  (crank)   : B -> A          motor at B
L2  (ground)  : B - C - D      ternary, fixed frame
L3  (coupler) : E - F - G      ternary, E = wheel/output point
L4            : D -> G          binary
L5            : C - F - H      ternary, connects both loops
L6            : A -> H          binary

Loop 1:  B - A - H - C    (ground segment B-C)
Loop 2:  C - F - G - D    (ground segment C-D)
```

## Notes

- These are centerline distances only — add material around each joint for bearings/fasteners (typically 1.5–2× bolt diameter as edge margin).
- Link thickness is a design choice; the simulator treats links as rigid 2D bodies.
- Regenerate this table by running `python sixbar_sim.py 6bar6_4.motiongen --no-gui` after any geometry or autoscale change.
