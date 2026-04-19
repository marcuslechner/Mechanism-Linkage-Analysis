# 6bar6_5 Link Dimensions

Source: `6bar6_5.motiongen` with `settings.json` (`autoscale.target_delta_y = 250 mm`, applied scale `0.777566×`). All lengths are centerline distances between joint axes, in mm.

## Link dimensions

| Link | Type | Role | Edge | Length (mm) |
|---|---|---|---|---|
| **L1** | binary | Crank (motor at B) | A–B | 34.04 |
| **L2** | ternary | Ground frame | B–C | 51.15 |
| | | | C–D | 32.05 |
| | | | B–D | 78.01 |
| **L3** | ternary | Coupler (E = output) | E–F | 195.02 |
| | | | F–G | 31.37 |
| | | | E–G | 184.22 |
| **L4** | binary | Rocker (loop 2) | D–G | 173.30 |
| **L5** | ternary | Connects both loops | C–F | 165.49 |
| | | | F–H | 96.56 |
| | | | C–H | 68.93 |
| **L6** | binary | Rocker (loop 1) | A–H | 54.36 |

For each ternary link, sketch a triangle using the three side lengths (SolidWorks solves SSS uniquely). Drill joint holes at the triangle vertices.

## Ground-frame pivot spacing (for the base plate / chassis)

L2 is fixed; use these to locate the motor axis (B) and the two idler pivots (C, D).

| From | To | Distance (mm) |
|---|---|---|
| B | C | 51.15 |
| C | D | 32.05 |
| B | D | 78.01 |

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
- Regenerate this table by running `python sixbar_sim.py inputs/6bar6_5.motiongen --export-dimensions outputs/6bar6_5_dimensions.md --no-gui` after any geometry or autoscale change.
