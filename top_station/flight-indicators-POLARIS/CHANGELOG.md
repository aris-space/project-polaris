# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Added

- **Tachometer** panel — single-needle RPM gauge; 0–8000 RPM range with a 320° sweep, configurable via a single message path field
- **Dual Gauge** panel — generic dual-needle gauge with fully configurable data pipeline and display; all gauge-type panels are thin wrappers supplying default config to the shared `createDualGaugePanel` factory
- **Dual Tachometer** panel — helicopter-style split tachometer (engine % RPM left, rotor % RPM right); defaults: 5800 RPM engine / 600 RPM rotor, 50–110% display range
- **Dual Fuel Gauge** panel — left/right tank quantity gauge; normalizes raw kg values (0–50) to 0–1 display range with E / ½ / F tick labels and red/yellow/green zones
- **Dual Oil Gauge** panel — oil temperature (°C, left) and oil pressure (PSI, right) with custom tick positions and color zones matching typical limits

#### Dual Gauge configurable features
- **Data** sub-node per side: optional expression transform (`x`-variable, powered by `expr-eval`), optional normalization with configurable input and output range, always-on clamping with configurable min/max (clamp range also defines the display range)
- **Display** sub-node per side: tick count, tick precision, custom tick positions (csv, must match tick count), custom tick labels (csv, must match tick count), sub-tick toggle, up to 5 color arc zones
- **Labels**: top, bottom (csv — 1 value centered, 2 values split left/right), left/right labels with optional vertical character stacking and length-aware font scaling

### Changed

- `ensureCss` in `BundledFlightIndicators.ts` is now exported so new instrument components can share the single CSS injection guard

---

## [1.0.0] - 2026-04-03

Initial public release.

### Added

- **Airspeed Indicator** panel — displays indicated airspeed (0–160 KIAS)
- **Altimeter** panel — three-pointer altimeter with QNH Kollsman window (altitude in ft, pressure in hPa)
- **Attitude Indicator** panel — gyroscopic horizon with pitch and roll inputs
- **Heading Indicator** panel — directional gyro compass rose with automatic [0°, 360°) normalization
- **Turn Coordinator** panel — turn-and-bank needle clamped to ±20° standard-rate deflection
- **Variometer** panel — vertical speed indicator, ±1950 ft/min range
- **Message path inputs** — each panel field accepts a Foxglove-native message path string (`/topic.field`, `/topic[0].field`, etc.) with Studio autocomplete; works with any message type that resolves to a finite number
- **Auto-sizing** — instruments resize to fill the panel at any window dimension via `ResizeObserver`
- SVG instrument graphics vendored as inline data URLs — no static file server required in the bundled extension environment
- `npm run win-install` script for one-step build and install to Foxglove Studio on Windows from WSL

### Technical

- Rendering powered by [flight-indicators-js](https://github.com/teocci/js-module-flight-indicators) (MIT), replacing an earlier GPL-3.0 dependency
- No custom ROS message types required — compatible with `std_msgs/Float64` and any other schema
- Message path parsing supports dot notation and bracket array indexing (`field.nested`, `arr[0].value`)