import { Joy } from "../types";

interface JoyDataDisplayProps {
  joy: Joy | undefined;
  kbMapping?: Record<string, { button: number; axis: number; direction?: string | null }>;
  uiScale?: number;
}

// Map axis indices to their names based on PS4 controller
const axisNames: Record<number, string> = {
  0: "L Stick X",
  1: "L Stick Y",
  2: "R Stick X",
  3: "R Stick Y",
  4: "L2",
  5: "R2",
};

const axisBaseline: Record<number, number> = {
  4: -1,
  5: -1,
};

// Map button indices to their names based on PS4 controller
const buttonNames: Record<number, string> = {
  0: "X",
  1: "O",
  2: "□",
  3: "△",
  4: "L1",
  5: "R1",
  6: "L2",
  7: "R2",
  8: "Share",
  9: "Options",
  10: "L3",
  11: "R3",
  12: "D-Pad ↑",
  13: "D-Pad ↓",
  14: "D-Pad ←",
  15: "D-Pad →",
  16: "PS",
  17: "Touchpad",
};

export function JoyDataDisplay({ joy, kbMapping, uiScale = 1 }: JoyDataDisplayProps): JSX.Element {
  if (!joy) {
    return <div style={{ padding: `${16 * uiScale}px`, color: "#999", fontSize: `${14 * uiScale}px` }}>No joy data</div>;
  }

  const kbButtonLabels = new Map<number, string[]>();
  const kbAxisLabels = new Map<number, string[]>();
  if (kbMapping) {
    for (const [key, config] of Object.entries(kbMapping)) {
      const displayKey = key === " " ? "Space" : key;
      if (config.button >= 0) {
        const existing = kbButtonLabels.get(config.button) ?? [];
        existing.push(displayKey);
        kbButtonLabels.set(config.button, existing);
      }

      if (config.axis >= 0) {
        const dir = config.direction ? ` ${config.direction}` : "";
        const existing = kbAxisLabels.get(config.axis) ?? [];
        existing.push(`${displayKey}${dir}`.trim());
        kbAxisLabels.set(config.axis, existing);
      }
    }
  }

  const maxButtonIndex = Math.max(
    -1,
    ...Object.keys(buttonNames).map((key) => Number(key)).filter((num) => !Number.isNaN(num)),
  );
  const maxAxisIndex = Math.max(
    -1,
    ...Object.keys(axisNames).map((key) => Number(key)).filter((num) => !Number.isNaN(num)),
  );
  const buttonCount = Math.max(joy.buttons.length, maxButtonIndex + 1);
  const axisCount = Math.max(joy.axes.length, maxAxisIndex + 1);
  const buttons = Array.from({ length: buttonCount }, (_, idx) => joy.buttons[idx] ?? 0);
  const axes = Array.from({ length: axisCount }, (_, idx) => joy.axes[idx] ?? 0);

  return (
    <div style={{ padding: `${16 * uiScale}px`, fontFamily: "monospace", fontSize: `${12 * uiScale}px`, maxWidth: "1200px" }}>
      <h3 style={{ margin: `0 0 ${16 * uiScale}px 0`, fontSize: `${16 * uiScale}px`, fontWeight: "600" }}>Raw Joy Data</h3>
      
      <div style={{ marginBottom: `${24 * uiScale}px` }}>
        <h4 style={{ margin: `0 0 ${12 * uiScale}px 0`, fontSize: `${14 * uiScale}px`, fontWeight: "500", color: "#bbb" }}>
          Buttons <span style={{ color: "#666" }}>({buttons.length})</span>
        </h4>
        <div style={{ display: "grid", gridTemplateColumns: `repeat(auto-fill, minmax(${Math.max(50, 80 * uiScale)}px, 1fr))`, gap: `${8 * uiScale}px` }}>
          {buttons.map((value, idx) => (
            <div
              key={idx}
              style={{
                padding: `${10 * uiScale}px ${Math.max(2, 4 * uiScale)}px`,
                border: "1px solid #555",
                backgroundColor: value === 1 ? "#4a4" : "#2a2a2a",
                textAlign: "center",
                color: value === 1 ? "#000" : "#ccc",
                borderRadius: `${6 * uiScale}px`,
                transition: "all 0.15s ease",
              }}
            >
              <div
                style={{
                  fontSize: `${12 * uiScale}px`,
                  fontWeight: "700",
                  color: value !== 0 ? "#4af" : "#ccc",
                  marginBottom: `${6 * uiScale}px`,
                }}
              >
                [{idx}] = {value}
              </div>
              <div style={{ fontSize: `${16 * uiScale}px`, fontWeight: "600", color: value === 1 ? "#000" : "#fff" }}>
                {buttonNames[idx] || `B${idx}`}
              </div>
              {kbButtonLabels.has(idx) ? (
                <div style={{ fontSize: `${10 * uiScale}px`, fontWeight: "500", color: "#bbb", marginTop: `${6 * uiScale}px` }}>
                  {(kbButtonLabels.get(idx) ?? []).join(", ")}
                </div>
              ) : null}
            </div>
          ))}
        </div>
      </div>

      <div>
        <h4 style={{ margin: `0 0 ${12 * uiScale}px 0`, fontSize: `${14 * uiScale}px`, fontWeight: "500", color: "#bbb" }}>
          Axes <span style={{ color: "#666" }}>({axes.length})</span>
        </h4>
        <div style={{ display: "grid", gridTemplateColumns: `repeat(auto-fill, minmax(${Math.max(55, 100 * uiScale)}px, 1fr))`, gap: `${8 * uiScale}px` }}>
          {axes.map((value, idx) => (
            <div
              key={idx}
              style={{
                padding: `${10 * uiScale}px ${Math.max(2, 4 * uiScale)}px`,
                border: "1px solid #555",
                backgroundColor: "#2a2a2a",
                borderRadius: `${6 * uiScale}px`,
                textAlign: "center",
              }}
            >
              <div
                style={{
                  fontSize: `${12 * uiScale}px`,
                  fontWeight: "700",
                  color: value !== (axisBaseline[idx] ?? 0) ? "#4af" : "#ccc",
                  marginBottom: `${6 * uiScale}px`,
                }}
              >
                [{idx}] = {value.toFixed(2)}
              </div>
              <div style={{ fontSize: `${13 * uiScale}px`, fontWeight: "600", color: "#fff" }}>
                {axisNames[idx] || `Axis ${idx}`}
              </div>
              {kbAxisLabels.has(idx) ? (
                <div style={{ fontSize: `${10 * uiScale}px`, fontWeight: "500", color: "#bbb", marginTop: `${6 * uiScale}px` }}>
                  {(kbAxisLabels.get(idx) ?? []).join(", ")}
                </div>
              ) : null}
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
