# SBL-API tools — pool validation (2026-10-06)

Validation of the Waterlinked UGPS G2 HTTP-API calibration tools
(`src/sensors/uwgpsg2_ros2_interface/`, branch `thesis_gleb/test_lake/sbl_api`), run from a Windows
laptop in the x86 devcontainer, Ethernet to the topstation, box at `http://192.168.194.94`, box in
**static** mode, `environment: reflective` (pool).

## Result summary

| Tool | Status | Notes |
|---|---|---|
| **Heading** `sbl_heading_set` / `sbl_heading_calibration` | ✅ works | wrote `static_orientation` 270 → 123.4, verified on config + `position/master` |
| **Topside position** `sbl_position_calibration` (simulated) | ✅ works | wrote `static_lat`/`static_lon`, verified on `position/master` (moved 1670 m to the sim point) |
| **Sound speed** `sbl_sound_speed_set` | ❌ not settable (tool removed) | `speed_of_sound` is **read-only on this firmware** (GUI can't set it either) — handle offline, see below |
| **External depth** `sbl_external_depth` | ⚠️ no-op on this hardware (tool removed) | PUTs accepted (HTTP 200 @ 4 Hz) but **ignored**: the **Locator-U1 has its own depth sensor**, so the box uses that and discards external depth. External depth is **A1-only**. Handle depth in our own filter/post-processing. |

> The `sbl_sound_speed_set` and `sbl_external_depth` tools were **removed from the repo** (both are
> no-ops on this hardware — see below). This document preserves the findings. The freshwater
> sound-speed helper `sbl_water.sound_speed_freshwater` is kept for the offline range correction.

## Networking gotcha (laptop → topstation)

The USB-Ethernet adapter to the topstation does **not** get DHCP — it sits on a link-local
`169.254.x.x` and can't reach the box. Fix: give it a **static IP in the box's /24**, e.g.
```
New-NetIPAddress -InterfaceAlias "Ethernet 4" -IPAddress 192.168.194.50 -PrefixLength 24   # elevated PowerShell
Test-Connection 192.168.194.94
```
Once the Windows host can ping `.94`, the devcontainer reaches it via NAT (we run the container
without `--network=host`).

## Box config snapshot — `GET /api/v1/config/generic`

```json
{"antenna_enabled": true, "channel": 5020, "compass": "static", "environment": "reflective",
 "external_pps_enabled": false, "gps": "static", "imu_vehicle_enabled": false, "locator_type": "u1",
 "range_max_x": 0, "range_max_y": 0, "range_max_z": 0, "range_min_x": 0, "range_min_y": 0,
 "search_direction": 3, "search_radius": 5, "search_sector": 360, "speed_of_sound": 1475,
 "static_lat": 47.328921, "static_lon": 8.572832, "static_orientation": 270}
```

## Firmware / API facts learned (operational)

- **`config/generic` PUT is a full-object replace** (goa framework). The **required** writable attributes
  are: `channel, locator_type, gps, compass, static_lat, static_lon, static_orientation`. A partial PUT
  → `vnd.goa.error` "attribute … missing and required".
- **Verify any write via `GET /api/v1/position/master`** → `lat`, `lon`, `orientation`.
- **Field types matter:**
  - `speed_of_sound` is an **integer** (a float body → `cannot unmarshal number 1441.2 … speed_of_sound`).
  - `static_orientation` accepts a number incl. **float** (123.4 worked).
  - `static_lat`/`static_lon` are floats.
- **`speed_of_sound` is read-only / not applyable.** A full-object PUT that *only* changes
  `speed_of_sound` (all required fields present, correct int type) passes schema validation but the
  Waterlinked apply handler refuses it: `HTTP 400`, `Content-Type: vnd.waterlinked.operation_response+json`,
  body `null`. Heading/position changes go through the exact same full-object path and apply fine, so the
  refusal is specific to `speed_of_sound`. This matches the GUI also being unable to set it. **The box
  stays at `speed_of_sound = 1475` (constant).**

## Decision: sound speed is corrected OFFLINE (not on the box)

Sound speed only **scales ranges linearly**: the box reports `range = c_box · t` (time-of-flight `t`).
With a better `c_true` (from water temperature), the corrected range is
```
range_true = range_box · (c_true / c_box),   c_box = 1475 m/s  (box constant, not settable)
```
So we **do not** write sound speed to the box. We record the raw `sbl_raw.receiver_distance` (and the
solved `x/y/z`) and apply the factor `c_true / c_box` in **post-processing**. At the pool (~8.5 °C,
freshwater `c_true ≈ 1441 m/s`) the factor is `1441/1475 = 0.977` (ranges ~2.3 % too long as reported).
Compute `c_true` from temperature with the freshwater Marczak formula in `sbl_water.sound_speed_freshwater`
(the helper is kept; the `sbl_sound_speed_set` node was **removed** — it can't write the box anyway).

## External depth — ignored by the box (Locator-U1)

`sbl_external_depth` PUTs `/api/v1/external/depth {depth, temp}` fine (HTTP 200 @ 4 Hz), but the box
does **not** use it: the verify `z` from `/position/acoustic/filtered` stays ~0.0–0.24 m (the box's own
value, ~0.14 m in the GUI), never the fed 2.5 m. Cause: **the Locator-U1 has a built-in depth sensor**,
so the G2 uses the locator's own depth and ignores external depth. External depth is **only for the
Locator-A1** (no depth sensor). This unit is `locator_type: u1` → external depth is a **no-op**.
Decision: same as sound speed — we use our own (Bar30/Keller) depth in **our filter / post-processing**,
not as a box input. The `sbl_external_depth` node was **removed from the repo** (no-op here). For the
record, it applied a locator-frame offset (`base_link→sbl_link` = 0.197 m via TF) so the fed depth was the
locator's, not the COM's — that correction logic belongs in the PF/offline depth handling when needed.

## Open items

- Heading currently holds the **test value 123.4** — run the real `sbl_heading_calibration` (or set a
  known heading) before trusting it.
- For the lake, set `environment` from `reflective` → `openwater`.
