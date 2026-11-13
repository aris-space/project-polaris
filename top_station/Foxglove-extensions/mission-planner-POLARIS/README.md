# Mission Planner (POLARIS)

A Foxglove panel for planning a mission on a map. It works like the built-in Map panel and adds a safe area, a start point and a path planner.

> **Frontend only (v1).** The mission is saved in the Foxglove layout and is **not published to ROS 2 yet**. That will be added once the high-level autonomy interface is settled.

## Features

| Tool | Key | What it does |
|---|---|---|
| Pan | `Esc` | Move the map. Existing points can still be dragged. |
| Safe area | `1` | Click to place vertices (≥ 3). Finish by clicking the first vertex, pressing `Enter`, or double-clicking. Everything **outside** the area is tinted translucent red. Once finished: drag vertices to move them, drag or click a **+** midpoint to insert one, right-click to delete one. |
| Start point | `2` | Click to place the start cross. You can drag it afterwards. |
| Start from GNSS | `G` | Sets the start point to the latest fix on the GNSS topic (default `/fix`). The dot shows fix health: green = OK, amber = more than 2 m uncertainty, grey = no fix or older than 5 s. |
| Path | `3` | Click to add waypoints. They connect automatically, starting from the start point. Each segment shows its length. `Enter` or double-click finishes. |
| Close loop | `L` | Adds a dashed return leg from the last waypoint back to the start. Needs ≥ 2 waypoints. |
| Undo / Redo | `⌘/Ctrl Z`, `⌘/Ctrl Shift Z` | Up to 50 steps. |
| Remove last point | `Backspace` | While drawing a safe area or a path. |
| Fit mission | `F` | Zooms to the mission and the vehicle. |

**Safe-area check:** segments or waypoints outside the safe area turn red. The status bar shows a ⚠ warning, or ✓ when the whole path is inside.

**Workflow guidance:** finishing the safe area switches to the Start tool, and setting the start switches to the Path tool. The status bar always shows what to do next.

## Vehicle position

This works like the stock Map panel. Under **Vehicle position** in the panel settings you set:

- the **Topic**: any `sensor_msgs/NavSatFix` or `foxglove.LocationFix`. The default is `/gps/filtered/global`, the global EKF estimate.
- **History**: how many past positions to keep as a trail. 0 shows only the marker.
- the **Color**.
- **Follow vehicle**: also available as a button. Dragging the map turns follow off.

The marker becomes a heading arrow (course over ground) once the vehicle moves. The trail is cleared when you seek in a recording.

## Map

You can choose a street map (OpenStreetMap), satellite imagery (Esri), or a **custom XYZ tile URL**, e.g. a local tile server for offline use at the lake. The panel UI follows Foxglove's light or dark theme.

## Development

```bash
npm install
npm run lint:ci && npm run build
npm run local-install        # install into local Foxglove desktop
npm run package              # -> polaris.ros-mission-planner-polaris-<version>.foxe
```

`config.ts` turns off `url()` resolution in css-loader so that `leaflet.css` bundles without image loaders. All markers are SVG `divIcon`s.

### Source layout
- `src/MissionPlannerPanel.tsx`: Foxglove wiring (render loop, subscriptions, settings), mission state with undo, and keyboard shortcuts.
- `src/map/MapController.ts`: imperative Leaflet wrapper (mask, route, handles, vehicle, GNSS).
- `src/geo.ts`: distances, area, point-in-polygon, and the safe-area segment check.
- `src/ui/`: toolbar, status bar and icons. `src/styles.css`: theme tokens and styles.
