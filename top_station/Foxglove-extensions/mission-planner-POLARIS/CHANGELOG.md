# Changelog

## 1.0.0

- Initial mission planner panel (frontend only, mission stored in the layout):
  - Safe-area polygon (≥ 3 vertices) with translucent red outside mask; drag / insert / delete vertices
  - Start point via map click (cross) or current GNSS fix
  - Click-to-add path from the start point with segment lengths, close-loop toggle, safe-area violation highlighting
  - Vehicle position + trail (last N points) from a NavSatFix / LocationFix topic, follow mode
  - Street / satellite / custom tile basemaps, light & dark theme, undo/redo, keyboard shortcuts
