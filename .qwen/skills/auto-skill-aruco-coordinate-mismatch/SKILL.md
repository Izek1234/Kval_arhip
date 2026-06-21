---
name: aruco-coordinate-mismatch
description: Critical bug pattern — aruco_cmit_txt Gazebo model uses inverted Y for marker IDs vs naive aruco_map definition, causing drone navigation to fly in wrong direction
source: auto-skill
extracted_at: '2026-06-20T19:37:56.801Z'
---

# ArUco Coordinate Mismatch — Inverted Y-Axis

## The Bug
The `aruco_cmit_txt` Gazebo model (bundled with the Clover platform) places ArUco markers with **inverted Y-axis** relative to the naive mapping `y = ID // 10`. The physical positions follow `y = 9 - (ID // 10)`.

This means if your `SolarFarm.txt` (or any aruco_map file) defines marker positions with `y = ID // 10`, the aruco_map node will compute **inverted drone position** — when the drone is physically at y=0, aruco_map reports y=9, and `navigate` commands fly the drone in the opposite direction, causing it to fly off the edge of the map.

## Comparison

| ID | Naive (WRONG) | Physical (aruco_cmit_txt) |
|---|---|---|
| 0 | (0, 0) | (0, **9**) |
| 9 | (9, 0) | (9, **9**) |
| 90 | (0, 9) | (0, **0**) |
| 99 | (9, 9) | (9, **0**) |

**Correct formula**: `x = ID % 10`, `y = 9 - (ID // 10)`

**Also**: ID 17 is **missing** from the `aruco_cmit_txt` model — it must be excluded from the map file.

## Detection
If the drone reports `y≈9` at spawn (should be near y=0), or flies in the wrong direction when given `navigate` commands in `aruco_map` frame, the Y-axis is likely inverted in the map file.

## Fix
1. Update the ARUCO_MAP definition to use `y = 9 - (ID // 10)` formula
2. Regenerate `SolarFarm.txt` with corrected Y values
3. On the VM: run `gen_solar_farm.py` to write the corrected map to `aruco_pose/map/SolarFarm.txt` in the catkin workspace
4. Restart the simulator + aruco nodes

## Why it happens
The `aruco_cmit_txt` model was designed with marker ID layout matching typical image coordinate convention (origin top-left, y increasing downward in image space = decreasing in world Y). The naive formula `y = ID // 10` assumes origin bottom-left. This mismatch is easy to miss because the grid positions still cover all 100 integer points — just with swapped marker IDs on each position.
