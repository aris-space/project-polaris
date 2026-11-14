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
| Path | `3` | Click to add waypoints. They connect automatically, starting from the start point. Each segment shows its length. `Enter` or double-click finishes. Clicking **on or near the START cross** (once there are ≥ 2 waypoints) snaps to it and closes the loop. |
| Close loop | `L` | Adds a dashed return leg from the last waypoint back to the start. Needs ≥ 2 waypoints. |
| Undo / Redo | `⌘/Ctrl Z`, `⌘/Ctrl Shift Z` | Up to 50 steps. |
| Delete a point | click it, then `Delete`/`Backspace` | Works for safe-area vertices, waypoints and the start point. `Esc` deselects. Right-click also deletes. |
| Remove last point | `Backspace` | While drawing with nothing selected. |
| Fit mission | `F` | Zooms to the mission and the vehicle. |

## Saved missions

The chip at the top centre of the map shows the current mission's name. An amber dot means it has unsaved changes. Click the chip to:

- **Name** the mission and **Save** it (or press `⌘/Ctrl S` anywhere in the panel). The first save creates a new saved mission; later saves update it.
- **Save as copy**: store the current state as a new mission.
- **New mission**: start from an empty map.
- **Switch** between saved missions by clicking one in the list. Each entry shows its waypoints, length and when it was saved. If you have unsaved changes, the first click warns you and the second click loads.
- **Delete** a saved mission with the trash icon (click twice to confirm).

Saved missions are stored in the **Foxglove layout** (panel state). They survive restarts and travel with the layout if you export or share it.

## Waypoint actions

Click a waypoint to open its card (bottom left) and choose what the vehicle does there:

| Action | Meaning | Time |
|---|---|---|
| **Pass** (default) | Drive through, no stopping | – |
| **Stop & wait** | Stop and hold position | Wait time (s) |
| **Surface** | Stop and rise to the surface / ice | Hold at the top (s, 0 = none) |

Set the time with **−/+** (5 s steps) or type it and press Enter. **Apply to all waypoints** copies the action to every waypoint. Waypoints with an action get a badge on the map (⏸ 30s amber for wait, ⤒ 10s blue for surface), and the status bar counts them. Actions are kept when you drag a waypoint, and are included in undo, saved missions and exported files (`"action": "wait" | "surface"`, `"holdS": seconds`; pass waypoints have no extra fields).

## Mission type: Waypoints or Survey

In Plan mode the switch at the top of the tool palette chooses what the mission is (key `4` toggles):

- **Waypoints**: the path planner. Click waypoints from the start, close the loop, and so on.
- **Survey**: mark the **area to be surveyed**. Press `3` (Survey area), click ≥ 3 vertices and close the polygon like the safe area. It's drawn in violet and edited the same way (drag, **+** to insert, select + `Delete`, right-click). The status bar shows its size. If it reaches outside the safe area, the outline turns red and dashed and a ⚠ warning appears.

Edge lengths are shown on the survey area (and live on the segment you're drawing, for every drawing tool).

Switching type never hides or deletes anything: the path and the survey area both stay on the map, stay editable and are both checked against the safe area. The switch only chooses which drawing tool the palette offers (Path or Survey area) and what the mission's primary type is. It can be undone.

## Plan / Watch mode

The switch at the top left toggles between:

- **Plan**: the full editor described above.
- **Watch**: view only. The editing tools are hidden and points can't be dragged, selected or deleted. You still see the vehicle and trail, can switch between saved missions, and can export. Use this on the operator screen during a run.

The mode is saved with the layout and can also be set in the panel settings (Mission → Mode).

## Mission files (JSON)

Missions can be exported and imported as files, independent of the Foxglove layout, so you can keep them in git next to the autonomy mission CSVs. Use the buttons at the bottom of the mission menu:

- **Export current**: the mission on the map → `<name>.mission.json`
- **↓ on a saved mission**: that mission
- **Export all**: every saved mission in one library file → `polaris-missions-YYYY-MM-DD.json`
- **Import…** (Plan mode): one or more mission or library files. They're added to the saved missions; duplicate names get " (2)". A single imported mission opens straight away unless you have unsaved changes.

Desktop Foxglove shows a native *Save as* dialog; elsewhere the file is downloaded.

```json
{
  "format": "polaris-mission",
  "version": 1,
  "name": "Goldbach line",
  "exportedAt": "2026-11-14T00:47:12.000Z",
  "mission": {
    "safeArea": [{ "lat": 47.2582, "lon": 8.6891 }, …],
    "safeClosed": true,
    "start": { "lat": 47.2561, "lon": 8.6903 },
    "path": [{ "lat": 47.2572, "lon": 8.6909 }, …],
    "closed": true,
    "type": "waypoints",
    "survey": [],
    "surveyClosed": false
  }
}
```

A library file has `"format": "polaris-mission-library"` and a `missions` array of `{ name, savedAt, mission }`. Coordinates are WGS84 degrees.

**Safe-area check:** segments or waypoints outside the safe area turn red. The status bar shows a ⚠ warning, or ✓ when the whole path is inside.

**Workflow guidance:** finishing the safe area switches to the Start tool, and setting the start switches to the Path tool. The status bar always shows what to do next.

## Vehicle position

This works like the stock Map panel. Under **Vehicle position** in the panel settings you set:

- the **Topic**: any `sensor_msgs/NavSatFix` or `foxglove.LocationFix`. The default is `/gps/filtered/global`, the global EKF estimate.
- **History**: **All points** (default) keeps the whole trail; **Last N points** keeps a sliding window (0 = marker only). A **Clear trail** action resets it.
- **Min. point spacing**: new trail points closer than this to the previous one are skipped (default 0.2 m), so GNSS jitter while stationary doesn't scribble.
- the **Color**.
- **Follow vehicle**: also available as a button. It centres once, then only re-centres when the vehicle leaves the middle half of the view, so the map doesn't shake with GNSS noise. Dragging the map turns follow off.

The marker becomes a heading arrow (course over ground) once the vehicle has moved 1.5 m; it holds its heading while stationary. The trail is cleared when you seek in a recording.

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
