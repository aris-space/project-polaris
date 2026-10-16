# Mission: SBL raw-range data collection (first-week test)

**Branch:** `thesis_gleb/feature/sbl_driver_change`
**Goal:** verify the additive UGPS driver change and record the first `.mcap` bag of **raw
per-receiver SBL ranges** — the input the tightly-coupled particle filter will fuse. This test runs
entirely on a **Windows laptop in the devcontainer**, talking to the Waterlinked UGPS G2 box over
HTTP. No onboard computer, no ublox, no vehicle.

---

## What the driver change adds (additive — the deployed stack is unchanged)

- New opt-in parameter **`publish_sbl_raw`** (default `False`). When `False`, the driver behaves
  exactly as before.
- When `True`, the driver additionally:
  - GETs `…/api/v1/position/acoustic/raw` each cycle and publishes **`sbl_raw`**
    (`custom_msgs/SblRaw`): per-receiver `receiver_distance` / `receiver_nsd` / `receiver_rssi` /
    `receiver_valid`, plus the solved `x/y/z`, `std`, `position_valid`, and a `poll_unix_time`.
    Successive identical payloads are **de-duplicated** (box pings ~2–4 Hz).
  - GETs `…/api/v1/config/receivers` and `…/api/v1/config/generic` once at startup and publishes
    **`sbl_receiver_config`** (`custom_msgs/SblReceiverConfig`), **latched** (TRANSIENT_LOCAL): the
    fixed receiver geometry + sound speed, needed to interpret the ranges.

Topics (after remap) land under `/waterlinked_ugps/…`.

---

## Prerequisites

1. **Network:** the laptop is on the UGPS subnet and can reach the box:
   - `http://192.168.194.94` opens the Waterlinked web UI in the laptop's browser, **and**
   - the box is pinging a locator (a locator is powered and in the water / on the bench-test rig).
2. **Devcontainer networking:** this branch's `.devcontainer/devcontainer.json` runs **without**
   `--network=host` (which does not reach a physical LAN device on Docker Desktop/WSL2). The
   container reaches `192.168.194.94` by NAT through the Windows host, so the host itself must be on
   that subnet. Sanity check inside the container:
   ```bash
   curl -s http://192.168.194.94/api/v1/position/acoustic/raw | head -c 300
   ```
   If that returns JSON with `receiver_distance`, networking is good.

---

## Build & launch

Inside the devcontainer (`/ros2_ws`):

```bash
# 1. Build the new messages first, then the driver
colcon build --symlink-install --packages-select custom_msgs
colcon build --symlink-install --packages-select uwgpsg2_ros2_interface
source install/setup.bash

# 2. Launch driver (raw mode) + rosbag record
ros2 launch uwgpsg2_ros2_interface sbl_laptop_record.launch.py
#   optional args:
#     bag_dir:=/ros2_ws/recordings   # where the .mcap goes (timestamped subfolder). default ./recordings
#     record:=false                  # run driver only, no recording
#     bag_all:=true                  # record ALL topics instead of just the SBL ones
```

The bag is written to `…/sbl_raw_<YYYY_MM_DD-HH_MM_SS>/` as MCAP. `recordings/` is gitignored.

---

## Verification checklist (first-week acceptance)

Open a second terminal (`source install/setup.bash` first).

1. **Topics exist:**
   ```bash
   ros2 topic list | grep -E 'sbl_raw|sbl_receiver_config'
   ```
   Expect `/waterlinked_ugps/sbl_raw` and `/waterlinked_ugps/sbl_receiver_config`.

2. **Latched config arrives immediately** (even though it was published once at startup):
   ```bash
   ros2 topic echo --once /waterlinked_ugps/sbl_receiver_config
   ```
   Check `num_receivers` and that `receiver_x/y/z` match the physical topside layout. `speed_of_sound`
   is `-1.0` if the box does not report it (fine — note it and hard-code the site value later).

3. **Raw ranges stream:**
   ```bash
   ros2 topic echo /waterlinked_ugps/sbl_raw
   ```
   - `receiver_distance` length == `num_receivers` from step 2.
   - Distances are plausible (metres, > 0) and change as the locator moves.
   - `receiver_valid` is 1 for receivers with a clean return.

4. **Rate & de-dup:**
   ```bash
   ros2 topic hz /waterlinked_ugps/sbl_raw
   ```
   Expect roughly the box ping rate (~2–4 Hz), **not** the 4 Hz poll rate — duplicates are dropped.
   The driver also logs `SBL raw: N receivers (M valid), K new fixes so far` about once per second.

5. **Bag recorded:**
   ```bash
   ros2 bag info recordings/sbl_raw_*/    # newest folder
   ```
   Confirm `/waterlinked_ugps/sbl_raw` and `/waterlinked_ugps/sbl_receiver_config` have messages.

6. **Collect a real segment:** with a locator moving through a known pattern (e.g. a straight leg or
   a box), record 5–10 min. Note start/stop times and rough geometry in `research_thesis/` (offline
   branch) or a mission log for later cross-checking against the solved position.

---

## Troubleshooting

- **`curl` to the box fails / `sbl_raw` never publishes** → networking. Confirm the Windows host
  reaches `192.168.194.94`, and that the devcontainer does **not** use `--network=host`. Rebuild the
  container after editing `.devcontainer/devcontainer.json`.
- **`sbl_raw` publishes but arrays are empty / all `receiver_valid`=0** → the box has no acoustic
  lock (locator off, out of water, or out of range). Check the Waterlinked web UI.
- **`sbl_receiver_config` warns "unavailable"** → `…/api/v1/config/receivers` returned nothing;
  check firmware/endpoint via `curl`.
- **Repeated red "Transformation … lacking arguments!" lines** → expected and benign on the laptop
  (no topside NavigationStatus). They do not affect `sbl_raw` or the recording.
- **`ImportError: custom_msgs`** → build `custom_msgs` before the driver and re-`source install/setup.bash`.

---

## Success criteria

A `.mcap` bag containing a latched `sbl_receiver_config` and a continuous `sbl_raw` stream whose
`receiver_distance` arrays track a moving locator, at the box ping rate, recorded on the laptop with
the deployed driver behaviour otherwise untouched. That bag is the first input for developing the
tightly-coupled SBL particle filter offline.

---

## Validation log

**2026-09-23 — pipeline confirmed end-to-end on the real box (pool test).**
- Ran the driver on a Windows-laptop devcontainer against a UGPS G2 with the integrated 4-receiver
  antenna + Locator-U1. All message field names match the box's API exactly (no parsing changes
  needed). The antenna **does** expose per-receiver data — resolved the earlier concern.
- `sbl_receiver_config` latched correctly: **4 receivers, sound_speed = 1475 m/s**.
- `sbl_raw` populated correctly: length-4 `receiver_distance / nsd / rssi / valid`; invalid ranges
  reported as `-1.0` with `receiver_valid = 0`.
- Recorded a ~10.7 min bag (`recordings/sbl_raw_2026_09_23-13_46_41/`): **985 `sbl_raw`** messages +
  **1 latched `sbl_receiver_config`** (rosbag2 captured the TRANSIENT_LOCAL message — no QoS race in
  practice). `locator_position_global/_topside_ned/_acoustic_quality` were 0 (expected: no topside
  NavigationStatus feed on the laptop).
- **Data quality caveat:** test was in a **pool** → heavy multipath + short antenna baseline meant
  mostly only 1 of 4 receivers valid per ping, `position_valid=false`, and the box's solved `x/y/z`
  frozen with `std` capped at 5.0. Pipeline is proven; **usable data needs open water / a lake** (expect
  3–4 valid receivers there).

### Notes for the particle filter (from the real data)
- **`receiver_distance == -1.0` means "no measurement this ping"** — gate these out; never feed `-1`
  as a range.
- **Watch for a receiver holding a constant value** across many pings — verify it's a genuine static
  range (stationary locator) and not a stale/held value while the locator is actually moving; don't
  treat a held range as an independent new measurement.
- The box's **solved `x/y/z` freezes when `position_valid=false`** — don't trust the solved fix in
  those windows; that stale black-box behaviour is exactly what fusing the raw ranges is meant to beat.
