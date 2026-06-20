---
name: web-ui-layout-fix
description: Fix mission control web interfaces where buttons overflow upward beyond viewport — use min-height instead of fixed height, proper flex/grid sizing, and overflow control
source: auto-skill
extracted_at: '2026-06-20T11:02:32.310Z'
---

# Web UI Layout Fix — Preventing Content Overflow

## Problem
In this project, mission control web interfaces (`solar_farm_frontend.html`, `Front-end_drone_v4`) had buttons and UI elements "sliding up" beyond the viewport boundary. This happens when:

1. `body` uses `height: 100vh` + `display: flex; align-items: center` — forces container into exact viewport height, and vertical centering can push content above viewport top edge
2. `body` uses `overflow: hidden` — clips overflowing content instead of allowing scroll
3. Fixed-width buttons with large padding (`padding: 20px 50px`) combined with `margin-left/margin-right` create width conflicts

## Solution pattern

### For flex-based layouts (like solar_farm_frontend.html)
- Replace `body { height: 100vh }` → `body { min-height: 100vh }` — content can grow beyond viewport
- Remove `display: flex; justify-content: center; align-items: center` from body — let container flow naturally
- Use `padding` on `.container` instead of `margin-left/margin-right` on child panels
- Set `align-items: stretch` on container so panels fill height naturally
- Give side panels explicit `flex-shrink: 0; width: Xpx` to prevent squishing
- Buttons: `width: 100%` inside their panel, reasonable padding (`14px 24px` instead of `20px 50px`)
- Replace `position: fixed` footer → inline `text-align: center` block (fixed positioning can cause overlap)

### For grid-based layouts (like Front-end_drone_v4)
- Replace `.container { height: 100vh }` → `.container { min-height: 100vh }`
- Replace `body { overflow: hidden }` → `body { overflow-y: auto }`
- Remove `overflow: hidden` from header (was clipping scanline animation — keep animation but let content flow)
- Add `max-height` to console/log panels so they don't grow unbounded

### General principle
**`min-height` over `height`** — fixed `height: 100vh` assumes content always fits. When it doesn't, elements overflow upward (negative direction) because flex/grid tries to distribute space and clips the top. `min-height` allows the page to scroll when content exceeds viewport.

## Project-specific files
- `scripts/solar_farm_frontend.html` — flex layout, 3-column (controls | map+report | image)
- `scripts/Front-end_drone_v4` — grid layout, 2-column (map | controls) + console row

Both connect to ROS via rosbridge WebSocket (`ws://localhost:9090`) and publish/subscribe to `/mission/start`, `/mission/land`, `/mission/kill`, `/mission/status`, `/buildings`, `/solar`.
