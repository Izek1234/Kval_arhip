---
name: solar-farm-gen
description: Collision-free placement of solar panels, indicator platforms, and contamination spots in Gazebo world generation — avoiding ArUco marker grid positions
source: auto-skill
extracted_at: '2026-06-20T10:20:38.403Z'
---

# Solar Farm World Generation — Collision-Free Placement

## Context
The project uses `scripts/gen_solar_farm.py` to generate a Gazebo world with solar panels, indicator platforms, and contamination spots. ArUco markers occupy a regular 10×10 grid at integer coordinates (0–9 on both axes). All spawned objects must avoid overlapping ArUco markers.

## Key layout facts
- **ArUco grid**: 100 markers at positions `(i×1.0, j×1.0)` for `i,j ∈ [0..9]`
- **Panel**: 1×1 box collision, mesh visual, tilted (rot_x=1.55). Panel center must be ≥0.4m from any ArUco center.
- **Indicator platform**: 0.3×0.02 box, placed to the RIGHT of the panel (positive Y offset). Must also avoid ArUco markers with margin 0.4+0.15 (half-platform size).
- **Contamination spots**: small boxes on panel surface. Offset ±0.35m from panel center, size 0.08×0.08×0.01. Z coordinate accounts for panel tilt: `z = (cx - panel_x) * sin(0.8) + 0.06`.

## ArUco avoidance pattern

### For panels (`generate_solar_panels`)
After checking inter-panel distance, loop through all 100 ArUco positions and reject any candidate `(x, y)` where `sqrt((x-ax)^2 + (y-ay)^2) < aruco_margin` (default 0.4m).

### For indicator platforms (`_indicator_pos_right`)
Priority placement is RIGHT of panel (+Y). Try a grid of offsets:
- **Y offsets** (right): `[0.55, 0.45, 0.65, 0.75, 0.35, 0.85, 0.95, 1.05]`
- **X offsets** (wiggle room): `[0.0, -0.15, 0.15, -0.3, 0.3, -0.4, 0.4]`
- **Fallback** (left side, -Y): `[-0.55, -0.45, -0.65]` with same X offsets

For each `(dx, dy)`, check if `sqrt((ind_x-ax)^2 + (ind_y-ay)^2) < aruco_margin + ind_half` (0.55 total). Return first collision-free position.

### For contamination spots (`create_contamination_sdf`)
- Offset range ±0.35 from panel center (within 1×1 panel: `0.35 + 0.04 = 0.39 < 0.5`).
- Size reduced to 0.08×0.08 to keep contamination clearly within panel bounds.

## Tips for modifications
- If adding more objects near panels, always check against `aruco_positions` list.
- The `aruco_margin` (0.4) can be adjusted — increasing it makes placement harder but gives more clearance for drone camera detection of ArUco markers.
- If panels fail to find positions (attempts exceed 1000), consider shrinking `min_edge_dist` or `aruco_margin`, or expanding the spawn area.
- Indicator fallback to left (-Y) means visually the indicator may occasionally appear on the opposite side — acceptable for ArUco avoidance but may need consideration for inspection logic.
