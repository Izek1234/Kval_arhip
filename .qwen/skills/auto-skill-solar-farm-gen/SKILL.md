---
name: solar-farm-gen
description: Collision-free placement of solar panels, indicator platforms, and contamination spots in Gazebo world generation — avoiding ArUco marker grid and inter-object overlap
source: auto-skill
extracted_at: '2026-06-20T10:20:38.403Z'
---

# Solar Farm World Generation — Collision-Free Placement

## Context
The project uses `scripts/gen_solar_farm.py` to generate a Gazebo world with solar panels, indicator platforms, and contamination spots. ArUco markers occupy a regular 10×10 grid at integer coordinates (0–9 on both axes). All spawned objects must avoid overlapping ArUco markers **and each other**.

## Key layout facts
- **ArUco grid**: 100 markers at positions `(i×1.0, j×1.0)` for `i,j ∈ [0..9]`
- **Panel**: 1×1 box collision, mesh visual, tilted (rot_x=1.55). Panel center must be ≥0.4m from any ArUco center.
- **Indicator platform**: 0.3×0.02 box. Currently uses simplified placement `(px-0.25, py+0.1)` — slightly left and above panel. **The `_indicator_pos_right` function accepts `aruco_positions` but doesn't use them yet** — a placeholder for future ArUco-aware placement. If ArUco avoidance is needed for indicators, the offset-grid approach from the previous version should be restored (see below).
- **Contamination spots**: 0.08×0.08×0.01 boxes on panel surface, Z=0.6 (accounts for near-vertical panel tilt). Offset: ±0.2m from panel center **plus** a +0.15 X shift. Placed via `_place_contamination()` with collision avoidance.

## ArUco avoidance pattern

### For panels (`generate_solar_panels`)
After checking inter-panel distance, loop through all 100 ArUco positions and reject any candidate `(x, y)` where `sqrt((x-ax)^2 + (y-ay)^2) < aruco_margin` (default 0.4m).

### For indicator platforms (`_indicator_pos_right`)
**Current state**: returns hardcoded `(px-0.25, py+0.1)` — no ArUco checking.

**Previous ArUco-aware version** (restore if indicators overlap ArUco):
- Priority placement is RIGHT of panel (+Y). Try a grid of offsets:
  - Y offsets: `[0.55, 0.45, 0.65, 0.75, 0.35, 0.85, 0.95, 1.05]`
  - X offsets: `[0.0, -0.15, 0.15, -0.3, 0.3, -0.4, 0.4]`
  - Fallback (left, -Y): `[-0.55, -0.45, -0.65]` with same X offsets
- For each `(dx, dy)`, check `sqrt((ind_x-ax)^2 + (ind_y-ay)^2) < aruco_margin + ind_half` (0.55 total). Return first collision-free position.

### For contamination spots (`_place_contamination` + `create_contamination_sdf`)
Contaminations are placed with **inter-contamination collision avoidance**:
- `_place_contamination(panel_x, panel_y, existing, cont_size=0.08, min_gap=0.02, max_attempts=200)`
- For each attempt: generate `(cx, cy)` with `cx = panel_x + uniform(-0.2, 0.2) + 0.15`, `cy = panel_y + uniform(-0.2, 0.2)`
- Check distance to all `existing` spots: reject if `sqrt((cx-ex)^2 + (cy-ey)^2) < cont_size + min_gap` (= 0.10m)
- This ensures contaminations never overlap (min edge-to-edge gap = 0.02m)
- `create_contamination_sdf` only renders SDF; coordinates `(cx, cy)` come from `_place_contamination`

In `generate_world`, contaminations are placed in a loop tracking `existing_cont` list per panel.

## Tips for modifications
- If adding more objects near panels, always check against `aruco_positions` list.
- The `aruco_margin` (0.4) can be adjusted — increasing it makes placement harder but gives more clearance for drone camera detection of ArUco markers.
- If panels fail to find positions (attempts exceed 1000), consider shrinking `min_edge_dist` or `aruco_margin`, or expanding the spawn area.
- The contamination `+0.15` X shift accounts for the near-vertical panel tilt (rot_x=1.55). If panel pose changes, recalculate shift and Z accordingly.
- `_indicator_pos_right` currently ignores ArUco — if indicators start overlapping markers, restore the offset-grid search version.
- `generate_world` now accepts `aruco_positions=None` parameter for passing custom ArUco grids.
