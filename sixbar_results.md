# Six-Bar Linkage Simulation Results

Settings: `settings.json` — payload 22.24 N, autoscale target ΔY = 250 mm, motor limit 10 N·m, nominal 5 N·m, 720 steps/rev, 10 °/s.

---

## Motor Torque Summary

| File   | Peak Torque (N·m) | vs Limit (10 N·m) | vs Nominal (5 N·m) | Length Scale |
|--------|:-----------------:|:-----------------:|:------------------:|:------------:|
| 6bar2  | 9.562             | ✅ OK              | ⚠️ EXCEEDS          | 0.780×       |
| 6bar3  | 10.940            | ❌ EXCEEDS         | ⚠️ EXCEEDS          | 1.209×       |
| 6bar4  | 9.317             | ✅ OK              | ⚠️ EXCEEDS          | 1.097×       |
| 6bar5  | 8.959             | ✅ OK              | ⚠️ EXCEEDS          | 0.979×       |
| 6bar6  | 8.920             | ✅ OK              | ⚠️ EXCEEDS          | 0.838×       |
| 6bar7  | 9.176             | ✅ OK              | ⚠️ EXCEEDS          | 0.739×       |
| 6bar8  | 8.699             | ✅ OK              | ⚠️ EXCEEDS          | 0.719×       |
| 6bar6_1 | 8.861            | ✅ OK              | ⚠️ EXCEEDS          | 0.827×       |
| 6bar6_2 | 8.860            | ✅ OK              | ⚠️ EXCEEDS          | 0.754×       |
| 6bar6_3 | 9.038            | ✅ OK              | ⚠️ EXCEEDS          | 0.887×       |
| 6bar6_4 | 9.019            | ✅ OK              | ⚠️ EXCEEDS          | 0.836×       |
| 6bar6_5 | 8.779            | ✅ OK              | ⚠️ EXCEEDS          | 0.778×       |

> **6bar3** is the only design that exceeds the 10 N·m motor limit.  
> All designs exceed the 5 N·m nominal torque. Cycle time is 36 s for all (10 °/s).

---

## Link Dimensions (after autoscaling to ΔY = 250 mm)

All values in **mm**.

### 6bar2 — scale 0.780×, peak torque 9.562 N·m

| Link segment | Length (mm) |
|---|---:|
| L1: A–B (crank) | 38.53 |
| L2: B–C (ground) | 51.99 |
| L2: B–D (ground) | 75.42 |
| L2: C–D (ground) | 28.47 |
| L3: E–F | 133.87 |
| L3: E–G | 113.67 |
| L3: F–G | 24.60 |
| L4: D–G | 157.55 |
| L5: C–F | 151.72 |
| L5: C–H | 62.61 |
| L5: F–H | 95.23 |
| L6: A–H | 55.68 |

---

### 6bar3 — scale 1.209×, peak torque 10.940 N·m ❌

| Link segment | Length (mm) |
|---|---:|
| L1: A–B (crank) | 43.20 |
| L2: B–C (ground) | 82.73 |
| L2: B–D (ground) | 116.88 |
| L2: C–D (ground) | 45.60 |
| L3: E–F | 159.73 |
| L3: E–G | 131.60 |
| L3: F–G | 32.58 |
| L4: D–G | 188.47 |
| L5: C–F | 188.02 |
| L5: C–H | 83.02 |
| L5: F–H | 105.34 |
| L6: A–H | 66.14 |

---

### 6bar4 — scale 1.097×, peak torque 9.317 N·m

| Link segment | Length (mm) |
|---|---:|
| L1: A–B (crank) | 43.14 |
| L2: B–C (ground) | 72.54 |
| L2: B–D (ground) | 110.02 |
| L2: C–D (ground) | 40.02 |
| L3: E–F | 161.23 |
| L3: E–G | 138.15 |
| L3: F–G | 34.57 |
| L4: D–G | 191.46 |
| L5: C–F | 180.27 |
| L5: C–H | 92.60 |
| L5: F–H | 87.71 |
| L6: A–H | 86.39 |

---

### 6bar5 — scale 0.979×, peak torque 8.959 N·m

| Link segment | Length (mm) |
|---|---:|
| L1: A–B (crank) | 43.51 |
| L2: B–C (ground) | 64.73 |
| L2: B–D (ground) | 98.17 |
| L2: C–D (ground) | 35.71 |
| L3: E–F | 153.39 |
| L3: E–G | 132.26 |
| L3: F–G | 30.85 |
| L4: D–G | 170.84 |
| L5: C–F | 160.86 |
| L5: C–H | 86.76 |
| L5: F–H | 74.14 |
| L6: A–H | 77.08 |

---

### 6bar6 — scale 0.838×, peak torque 8.920 N·m

| Link segment | Length (mm) |
|---|---:|
| L1: A–B (crank) | 36.69 |
| L2: B–C (ground) | 55.45 |
| L2: B–D (ground) | 84.10 |
| L2: C–D (ground) | 30.59 |
| L3: E–F | 152.28 |
| L3: E–G | 134.31 |
| L3: F–G | 26.43 |
| L4: D–G | 171.19 |
| L5: C–F | 162.77 |
| L5: C–H | 74.32 |
| L5: F–H | 88.46 |
| L6: A–H | 65.35 |

---

### 6bar7 — scale 0.739×, peak torque 9.176 N·m

| Link segment | Length (mm) |
|---|---:|
| L1: A–B (crank) | 32.36 |
| L2: B–C (ground) | 48.90 |
| L2: B–D (ground) | 74.17 |
| L2: C–D (ground) | 26.98 |
| L3: E–F | 153.40 |
| L3: E–G | 138.15 |
| L3: F–G | 23.31 |
| L4: D–G | 172.44 |
| L5: C–F | 164.35 |
| L5: C–H | 65.54 |
| L5: F–H | 99.11 |
| L6: A–H | 57.64 |

---

### 6bar8 — scale 0.719×, peak torque 8.699 N·m

| Link segment | Length (mm) |
|---|---:|
| L1: A–B (crank) | 31.48 |
| L2: B–C (ground) | 47.58 |
| L2: B–D (ground) | 72.16 |
| L2: C–D (ground) | 26.25 |
| L3: E–F | 149.24 |
| L3: E–G | 134.40 |
| L3: F–G | 22.68 |
| L4: D–G | 167.76 |
| L5: C–F | 159.89 |
| L5: C–H | 62.91 |
| L5: F–H | 98.41 |
| L6: A–H | 47.26 |

---

### 6bar6_1 — scale 0.827×, peak torque 8.861 N·m

| Link segment | Length (mm) |
|---|---:|
| L1: A–B (crank) | 36.20 |
| L2: B–C (ground) | 54.70 |
| L2: B–D (ground) | 82.97 |
| L2: C–D (ground) | 30.18 |
| L3: E–F | 141.80 |
| L3: E–G | 126.15 |
| L3: F–G | 26.07 |
| L4: D–G | 168.89 |
| L5: C–F | 160.58 |
| L5: C–H | 73.32 |
| L5: F–H | 87.27 |
| L6: A–H | 64.47 |

---

### 6bar6_2 — scale 0.754×, peak torque 8.860 N·m

| Link segment | Length (mm) |
|---|---:|
| L1: A–B (crank) | 33.02 |
| L2: B–C (ground) | 49.90 |
| L2: B–D (ground) | 75.68 |
| L2: C–D (ground) | 27.53 |
| L3: E–F | 169.21 |
| L3: E–G | 157.65 |
| L3: F–G | 23.78 |
| L4: D–G | 154.05 |
| L5: C–F | 146.48 |
| L5: C–H | 66.88 |
| L5: F–H | 79.61 |
| L6: A–H | 58.81 |

---

### 6bar6_3 — scale 0.887×, peak torque 9.038 N·m

| Link segment | Length (mm) |
|---|---:|
| L1: A–B (crank) | 38.83 |
| L2: B–C (ground) | 58.68 |
| L2: B–D (ground) | 89.00 |
| L2: C–D (ground) | 32.37 |
| L3: E–F | 182.27 |
| L3: E–G | 166.12 |
| L3: F–G | 27.97 |
| L4: D–G | 162.09 |
| L5: C–F | 153.16 |
| L5: C–H | 78.64 |
| L5: F–H | 74.56 |
| L6: A–H | 69.16 |

---

### 6bar6_4 — scale 0.836×, peak torque 9.019 N·m

| Link segment | Length (mm) |
|---|---:|
| L1: A–B (crank) | 36.58 |
| L2: B–C (ground) | 55.28 |
| L2: B–D (ground) | 83.84 |
| L2: C–D (ground) | 30.49 |
| L3: E–F | 174.55 |
| L3: E–G | 160.91 |
| L3: F–G | 26.35 |
| L4: D–G | 160.08 |
| L5: C–F | 151.67 |
| L5: C–H | 74.09 |
| L5: F–H | 77.62 |
| L6: A–H | 65.15 |

---

### 6bar6_5 — scale 0.778×, peak torque 8.779 N·m

| Link segment | Length (mm) |
|---|---:|
| L1: A–B (crank) | 34.04 |
| L2: B–C (ground) | 51.15 |
| L2: B–D (ground) | 78.01 |
| L2: C–D (ground) | 32.05 |
| L3: E–F | 195.02 |
| L3: E–G | 184.22 |
| L3: F–G | 31.37 |
| L4: D–G | 173.30 |
| L5: C–F | 165.49 |
| L5: C–H | 68.93 |
| L5: F–H | 96.56 |
| L6: A–H | 54.36 |
