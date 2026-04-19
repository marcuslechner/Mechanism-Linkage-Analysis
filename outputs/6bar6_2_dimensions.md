# 6bar6_2 Link Dimensions

Source: `6bar6_2.motiongen` with `settings.json` (`autoscale.target_delta_y = 250 mm`, applied scale `0.754375×`). All lengths are centerline distances between joint axes, in millimeters.

## Link dimensions

| Link | Type | Role | Edge | Length (mm) |
|---|---|---|---|---|
| **L1** | binary | Crank (motor at B) | A–B | 33.02 |
| **L2** | ternary | Ground frame | B–C | 49.90 |
| | | | C–D | 27.53 |
| | | | B–D | 75.68 |
| **L3** | ternary | Coupler (E = output) | E–F | 169.21 |
| | | | F–G | 23.78 |
| | | | E–G | 157.65 |
| **L4** | binary | Rocker (loop 2) | D–G | 154.05 |
| **L5** | ternary | Connects both loops | C–F | 146.48 |
| | | | F–H | 79.61 |
| | | | C–H | 66.88 |
| **L6** | binary | Rocker (loop 1) | A–H | 58.81 |

For each ternary link, sketch a triangle using the three side lengths (SolidWorks solves SSS uniquely). Drill joint holes at the triangle vertices.

## Ground-frame pivot spacing (for the base plate / chassis)

L2 is fixed; use these to locate the motor axis (B) and the two idler pivots (C, D).

| From | To | Distance (mm) |
|---|---|---|
| B | C | 49.90 |
| C | D | 27.53 |
| B | D | 75.68 |

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
- Regenerate this table by running `python sixbar_sim.py 6bar6_2.motiongen --no-gui` after any geometry or autoscale change.
