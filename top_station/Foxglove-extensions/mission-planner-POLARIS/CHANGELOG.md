# Changelog

## 1.6.0

- Waypoint actions: **Pass** (no stopping, default), **Stop & wait** (hold time) and
  **Surface** (rise to the surface / ice, optional hold at the top)
- Click a waypoint to open its card: pick the action, set the time (± 5 s or type it),
  "Apply to all waypoints", Delete
- Map badges on waypoints (⏸ 30s / ⤒ 10s); status bar counts waits / surfaces
- Actions survive drag, undo, save, export/import (`path[i].action`, `path[i].holdS`)

## 1.5.1

- Survey area edges show their length (violet labels), also while drawing
- Live length label on the segment being drawn (safe area, survey area and path)
- Path and survey area both stay visible and editable when switching between Waypoints and Survey;
  the type switch only selects the drawing tool. Status bar and mission list show both.

## 1.5.0

- Mission type switch in Plan mode: **Waypoints** (path planner) or **Survey**
- Survey: draw the area to be surveyed like the safe area (≥ 3 vertices, close on first vertex /
  Enter / double-click), violet fill; drag / "+" insert / select + Delete / right-click to edit
- Survey area size in the status bar; warning (red dashed outline) when it reaches outside the safe area
- Switching type keeps both the path and the survey area (undoable); key `4` toggles the type
- Mission files and saved missions carry `type`, `survey`, `surveyClosed` (older files load as waypoints)

## 1.4.0

- Export / import missions as JSON files (independent of the Foxglove layout, e.g. for git):
  Export current, per-mission export, Export all (library file), Import… (multiple files,
  validation, duplicate names get " (2)", a single imported mission opens right away)
- Plan / Watch modes: Watch is view only (editing tools hidden, points locked, map clicks and
  edit shortcuts ignored) but you can still switch between saved missions and export
- Mode is also available in the panel settings and saved with the layout

## 1.3.0

- Saved missions: mission chip at the top of the map with name field, Save (⌘/Ctrl S),
  Save as copy, New mission, and a list of saved missions to switch between or delete
- Unsaved-changes indicator; loading another mission or starting a new one asks for
  confirmation when there are unsaved changes; loading is undoable
- Saved missions are stored in the panel's layout state

## 1.2.0

- Vehicle trail history: new "All points" mode (default) keeps the whole trail; "Last N points" still available
- Min. point spacing (default 0.2 m) so GNSS jitter while stationary doesn't pile up trail points
- "Clear trail" action in the Vehicle position settings
- Follow vehicle no longer re-centres on every fix: it centres once, then only when the vehicle leaves the middle half of the view (map and planned path stop shaking)
- Heading arrow only updates after >= 1.5 m of real movement (no twitching while stationary)
- Trail is appended incrementally (scales to long missions); eslint ignores dist/

## 1.1.0

- Click a point (safe-area vertex, waypoint or start) to select it, then press Delete/Backspace to remove it; Esc deselects
- Path tool snaps to the start: clicking on or near START (with ≥ 2 waypoints) closes the loop and finishes the path

## 1.0.0

- Initial mission planner panel (frontend only, mission stored in the layout):
  - Safe-area polygon (≥ 3 vertices) with translucent red outside mask; drag / insert / delete vertices
  - Start point via map click (cross) or current GNSS fix
  - Click-to-add path from the start point with segment lengths, close-loop toggle, safe-area violation highlighting
  - Vehicle position + trail (last N points) from a NavSatFix / LocationFix topic, follow mode
  - Street / satellite / custom tile basemaps, light & dark theme, undo/redo, keyboard shortcuts
