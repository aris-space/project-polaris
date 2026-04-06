# Flight Indicators Panel Style Guide

---

## 1. Architecture

Two files per instrument, strict separation:

| Layer | Location | Role |
|---|---|---|
| Instrument component | `src/instruments/Xxx.tsx` | Renders gauge DOM; no Foxglove imports |
| Panel component | `src/panels/XxxPanel.tsx` | Foxglove subscriptions, config state, settings |

---

## 2. Instrument component

### Props
```ts
type Props = { value: number | undefined; size: string };
```
- All data props are `number | undefined` — instrument renders at default position when undefined.
- `size` is a `string` pixel value produced by `useInstrumentPanel` (e.g. `"234px"`).
- No required non-nullable data props.

### Refs
```ts
const containerRef = useRef<HTMLDivElement>(null);        // root mount point
const needleRef    = useRef<HTMLDivElement | null>(null); // one per moving part
```

### Three-effect pattern

**Effect 1 — Mount (`[]`):** build DOM, store moving-part refs, return cleanup.
```ts
useEffect(() => {
  const el = containerRef.current;
  if (!el) return;
  ensureCss();
  const instrument = document.createElement("div");
  instrument.className = "instrument my-gauge";
  // append face img, needle wrappers, bezel div …
  el.appendChild(instrument);
  needleRef.current = needleDiv;
  return () => { el.innerHTML = ""; needleRef.current = null; };
}, []);
```

**Effect 2 — Resize (`[size]`):** parse and apply size to `.instrument`.
```ts
useEffect(() => {
  const n = parseInt(size, 10);
  if (!isNaN(n)) {
    const inst = containerRef.current?.querySelector<HTMLElement>(".instrument");
    if (inst) { inst.style.width = `${n}px`; inst.style.height = `${n}px`; }
  }
}, [size]);
```

**Effect 3 — Value (`[value]`):** update needle transform; guard against undefined.
```ts
useEffect(() => {
  if (value == null || !needleRef.current) return;
  const clamped = Math.max(MIN, Math.min(MAX, value));
  needleRef.current.style.transform = `rotate(${BASE_DEG + (clamped / MAX) * SWEEP_DEG}deg)`;
}, [value]);
```
Multi-value instruments have one value effect per independent moving part.

### Rotation constants
Document the formula above the constants:
```ts
// 0–8000 RPM → 320° sweep. rotate(90 + (rpm/8000)*320)deg
//   0 RPM → rotate(90deg), 8000 RPM → rotate(410deg)
const MAX_RPM = 8000; const BASE_DEG = 90; const SWEEP_DEG = 320;
```

### Return value
Always `<div ref={containerRef} />`. No JSX beyond the mount container.

---

## 3. DOM / CSS rules

### Shared CSS (injected by `ensureCss()`)
```css
.instrument { width: 250px; height: 250px; position: relative; display: inline-block; overflow: hidden; }
.instrument .box { position: absolute; top: 0; left: 0; width: 100%; height: 100%; }
```
- Every full-size element (face img, needle wrapper div, needle img, bezel div) gets class `box`.
- **Never** add `border-radius: 50%` to the instrument div — all existing instruments rely on the SVG background circle to define the visual boundary.
- **Never** apply a CSS `transform` to an inner `<img>` element. Only the wrapper `<div>` rotates.

### SVG asset loading
```ts
import myFaceRaw from "../assets/instruments/my_face.svg?raw";

function toDataUrl(svgRaw: string): string {
  return `data:image/svg+xml;charset=utf-8,${encodeURIComponent(svgRaw)}`;
}
const FACE_URL = toDataUrl(myFaceRaw); // convert at module level, not inside effects
```

To produce a horizontally mirrored variant from a single SVG source (e.g. R needle from L needle):
```ts
function mirrorSvgX(svgRaw: string): string {
  // Wraps SVG body in <g transform="scale(-1,1) translate(-400,0)"> → maps x → 400−x
  return svgRaw.replace(
    /(<svg[^>]*>)([\s\S]*)(<\/svg>\s*$)/,
    (_, open, body, close) =>
      `${open}<g transform="scale(-1,1) translate(-400,0)">${body}</g>${close}`,
  );
}
const NEEDLE_R_URL = toDataUrl(mirrorSvgX(needleRaw));
```

### DOM layer order (always this order)
```
.instrument
  └── face     <img class="box">              — static SVG background
  └── [needle  <div class="box">              — rotating wrapper (transform-origin here)
                 └── <img class="box">]       — needle SVG, no transform on img
  └── [repeat for each moving part]
  └── bezel    <div class="mechanics box">   — always last, renders on top
                 └── <img class="box">        — fi_circle.svg or equivalent
```

### Needle pivot origins

| Pivot | `transform-origin` | Derivation |
|---|---|---|
| Centre (standard) | `50% 50%` (default) | — |
| Left-edge (side pivot E) | `12.5% 50%` | fi_circle inner ring r=150 in 400px SVG → 150/400 = 12.5% inset |
| Right-edge (side pivot R) | `87.5% 50%` | mirror of E: 100% − 12.5% |

---

## 4. Panel component

### Config type
```ts
type Config = { speedPath: string; maxRpm: number };          // flat, no nested types
const defaultConfig: Config = { speedPath: "", maxRpm: 5800 }; // paths → "", numbers → meaningful default
```

### State initialization
```ts
const [config, setConfig] = useState<Config>(() => ({
  ...defaultConfig,
  ...(context.initialState as Partial<Config>),
}));
```

### Hook usage
```ts
const { getValue, containerRef, size } = useInstrumentPanel(context, [config.path1, config.path2]);
```
Pass all subscribed paths; hook deduplicates topics. `getValue(path)` returns `number | undefined`.

### Value preprocessing
Done between `getValue()` and render, in component body (not in effects):

| Need | Pattern |
|---|---|
| Clamp | `Math.max(min, Math.min(max, raw))` |
| Normalize angle | `((raw % 360) + 360) % 360` |
| Unit conversion | `raw / scale` |
| Scale to % | `(raw / maxRpm) * 100` |

Always guard: `rawValue != null ? transform(rawValue) : undefined`.

### Settings editor
```ts
useEffect(() => {
  context.updatePanelSettingsEditor({
    actionHandler: (action: SettingsTreeAction) => {
      if (action.action === "update") {
        const { path, value } = action.payload;
        const key = path[1] as keyof Config;
        setConfig((prev) => {
          const next = { ...prev, [key]: isNumeric(key) ? Number(value) : (value as string) };
          context.saveState(next);
          return next;
        });
      }
    },
    nodes: {
      general: {
        label: "General",
        fields: {
          speedPath: { label: "Speed (knots)", input: "messagepath", value: config.speedPath },
        },
      },
      scaling: {                   // add only when numeric calibration values exist
        label: "Scaling (unit = 100%)",
        fields: {
          maxRpm: { label: "Max RPM", input: "number", value: config.maxRpm, min: 1, step: 100 },
        },
      },
    },
  });
}, [context, config]);
```
Rules:
- `general` first; holds all `messagepath` fields. `scaling` holds only `number` fields.
- Field labels always include units: `"Speed (knots)"`, `"Altitude (ft)"`.
- `context.saveState(next)` always called inside the updater.

### Wrapper JSX
```tsx
return (
  <div ref={containerRef} style={{ display:"flex", alignItems:"center", justifyContent:"center", width:"100%", height:"100%" }}>
    <MyInstrument value={value} size={size} />
  </div>
);
```

### init function
```ts
export function initMyPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<MyPanel context={context} />);
  return () => { root.unmount(); };
}
```

---

## 5. Naming & registration

| Artefact | Pattern | Example |
|---|---|---|
| Instrument component | `src/instruments/Xxx.tsx` | `Tachometer.tsx` |
| Panel component | `src/panels/XxxPanel.tsx` | `TachometerPanel.tsx` |
| SVG assets | `src/assets/instruments/snake_case.svg` | `tachometer_mechanics.svg` |
| Panel name (registerPanel) | PascalCase, no spaces | `"DualTachometer"` |
| init export | `initXxxPanel` | `initTachometerPanel` |

Three files to touch for every new panel:
1. `src/instruments/Xxx.tsx`
2. `src/panels/XxxPanel.tsx`
3. `src/index.ts` — import + `extensionContext.registerPanel(...)`

---

## 6. SVG design system

### Canvas
All SVGs: `width="400px" height="400px" viewBox="0 0 400 400"`, centre at **(200, 200)**.

Required header:
```xml
<?xml version="1.0" encoding="utf-8"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg version="1.1" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink"
    x="0px" y="0px" width="400px" height="400px" viewBox="0 0 400 400"
    enable-background="new 0 0 400 400" xml:space="preserve">
```

Background circle radii across existing gauges:

| Instrument | r |
|---|---|
| Airspeed, Tachometer | 160 |
| Dual Tachometer | 163 |
| Heading Indicator | 161 |
| Altimeter | 153 |
| Attitude / Turn Coordinator horizon | 150 |

Tick ring and bezel inner edge both sit at **r = 150**, so they are always flush.

---

### Color palette

#### Backgrounds & structure

| Role | Value |
|---|---|
| Instrument face fill | `#232323` |
| Horizon sky (gradient) | `#558EBB` |
| Horizon ground (gradient) | `#503723` |
| Bezel ring fill | `#232323` |
| Bezel ring border stroke | `#353535` (stroke-width 1.3 inner / 3 outer) |

#### Needle

| Part | `fill` | `stroke` | `stroke-width` |
|---|---|---|---|
| White arm | `#FFFFFF` | `#B2B2B2` | 0.5 |
| Gray hub | `#232323` | `#353535` | 0.5 |

#### Ticks & arcs

| Element | Value |
|---|---|
| All tick lines | `stroke: white` (CSS keyword) |
| Normal arc | `stroke: green` |
| Caution arc | `stroke: yellow` |
| Critical arc | `stroke: red` |
| All color arcs | `stroke-width: 10` |

#### Accent colors

| Role | Hex |
|---|---|
| Compass cardinal markers (N/S/E/W) | `#FF2A00` |
| Attitude indicator aim/crosshair | `#FF4A24` |

---

### `fi_circle.svg` — shared bezel overlay

- Inner ring r = 150, outer ring r = 166 (even-odd fill path)
- Inner edge is 12.5% inset from each side of the 400 px square → basis for side-pivot `transform-origin`
- Use as the topmost layer on all standard round instruments

---

### Drop shadow filter

Copy verbatim; `stdDeviation` must be **5**, offset must be **0**:
```xml
<filter filterUnits="objectBoundingBox" id="filter-shadow">
  <feGaussianBlur stdDeviation="5" result="blur" in="SourceAlpha"/>
  <feOffset dx="0" dy="0" result="offsetBlurredAlpha" in="blur"/>
  <feMerge>
    <feMergeNode in="offsetBlurredAlpha"/>
    <feMergeNode in="SourceGraphic"/>
  </feMerge>
</filter>
```

---

### Color zone arcs

```css
.color-marks .level          { --level-color: green;  fill: none; stroke: var(--level-color); stroke-width: 10; stroke-miterlimit: 10; }
.color-marks .level.high     { --level-color: yellow; }
.color-marks .level.critical { --level-color: red;    }
```
```xml
<g class="color-marks">
  <path class="level normal"   d="M … A …" />
  <path class="level high"     d="M … A …" />
  <path class="level critical" d="M … A …" />
</g>
```
Arc order is green → yellow → red in the needle's sweep direction. Place after the background circle, before tick marks.

---

### Tick mark system

**CSS pattern** — group translated to centre, each line rotated via custom property:
```css
.marks { transform: translate(50%, 50%); }
.marks line {
  --base-angle: 270deg;
  transform: rotate(var(--base-angle));
  stroke: white; fill: none; stroke-miterlimit: 10; stroke-width: 2;
}
.marks .wide-ticks  line { stroke-width: 4; }
.marks .wide-ticks  line:nth-child(2) { --base-angle: 310deg; }
/* … one override per tick … */
```

**Tick tier dimensions** (all terminate at x2 = 150, flush with bezel inner edge):

| Class | `x1` | `x2` | Length | `stroke-width` | Purpose |
|---|---|---|---|---|---|
| `.wide-ticks` | 130 | 150 | 20 px | 4 | Major scale labels |
| `.long-ticks` | 135 | 150 | 15 px | 2 | Mid-scale labels |
| `.short-ticks` | 140 | 150 | 10 px | 2 | Minor divisions |

---

### Typography

| Property | Value |
|---|---|
| `font-family` | `Noto Sans KR, sans-serif` |
| Numerical labels `font-size` | 18–22 px |
| Title / unit labels `font-size` | 14–18 px |
| `font-weight` | `bold` (numbers), normal or 600 (titles) |
| `fill` | `white` |
| `text-anchor` | `middle` |
| `dominant-baseline` | `middle` |

**Label placement** — all text elements sit at `(0,0)`; CSS custom properties move them:
```css
.mark-labels { transform: translate(50%, 50%); }
.mark-labels .numerical { --base-x: 0%; --base-y: 0%; transform: translate(var(--base-x), var(--base-y)); }
.mark-labels .numerical:nth-child(1) { --base-x: 0;   --base-y: -28%; }
.mark-labels .numerical:nth-child(2) { --base-x: 16%; --base-y: -22%; }
/* … clockwise from 12 o'clock, matching major-tick sweep … */
```

---

### Asset inventory

| File | Role | Used by |
|---|---|---|
| `fi_circle.svg` | Bezel overlay ring (r 150/166) | All standard round gauges |
| `fi_needle.svg` | Centre-pivot needle (white arm + gray hub) | Airspeed, Heading, TurnCoordinator, Tachometer |
| `fi_needle_small.svg` | Upward-pointing small needle | Altimeter |
| `fi_tc_airplane.svg` | Turn coordinator airplane silhouette | TurnCoordinator |
| `speed_mechanics.svg` | Airspeed face (0–160 KIAS) | Airspeed |
| `vertical_mechanics.svg` | Variometer face (±2 km/h) | Variometer |
| `altitude_ticks.svg` | Altimeter outer tick ring | Altimeter |
| `altitude_pressure.svg` | Altimeter pressure sub-dial | Altimeter |
| `heading_mechanics.svg` | Heading compass rose | HeadingIndicator |
| `heading_yaw.svg` | Rotating yaw disc | HeadingIndicator |
| `horizon_back.svg` | Horizon ball (sky/ground gradient) | AttitudeIndicator |
| `horizon_ball.svg` | Pitch lines on horizon ball | AttitudeIndicator |
| `horizon_circle.svg` | Bank-angle donut ring | AttitudeIndicator, TurnCoordinator |
| `horizon_mechanics.svg` | Static overlay (crosshair, bank cap) | AttitudeIndicator |
| `turn_coordinator.svg` | Turn coordinator background | TurnCoordinator |
| `tachometer_mechanics.svg` | Tachometer face (0–8000 RPM) | Tachometer |
| `dual_tachometer_mechanics.svg` | Dual tachometer face (50–110% RPM) | DualTachometer |
| `dual_tachometer_needle.svg` | Side-pivot needle, arm points right from x=50 (E side); R side generated at runtime via `mirrorSvgX()` | DualTachometer |
