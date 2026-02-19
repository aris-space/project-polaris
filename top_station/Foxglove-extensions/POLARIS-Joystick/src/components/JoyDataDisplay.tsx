import { Joy } from "../types";

interface JoyDataDisplayProps {
  joy: Joy | undefined;
}

// Map axis indices to their names based on PS4 controller
const axisNames: Record<number, string> = {
  0: "L Stick X",
  1: "L Stick Y",
  2: "L2",
  3: "R Stick X",
  4: "R Stick Y",
  5: "R2",
  6: "D-Pad X",
  7: "D-Pad Y",
};

// Map button indices to their names based on PS4 controller
const buttonNames: Record<number, string> = {
  0: "X",
  1: "O",
  2: "□",
  3: "△",
  4: "L1",
  5: "R1",
  8: "Share",
  9: "Options",
  10: "L3",
  11: "R3",
  12: "PS",
  13: "Touchpad",
};

export function JoyDataDisplay({ joy }: JoyDataDisplayProps): JSX.Element {
  if (!joy) {
    return <div style={{ padding: "16px", color: "#999" }}>No joy data</div>;
  }

  return (
    <div style={{ padding: "16px", fontFamily: "monospace", fontSize: "12px", maxWidth: "1200px" }}>
      <h3 style={{ margin: "0 0 16px 0", fontSize: "16px", fontWeight: "600" }}>Raw Joy Data</h3>
      
      <div style={{ marginBottom: "24px" }}>
        <h4 style={{ margin: "0 0 12px 0", fontSize: "14px", fontWeight: "500", color: "#bbb" }}>
          Buttons <span style={{ color: "#666" }}>({joy.buttons.length})</span>
        </h4>
        <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fill, minmax(80px, 1fr))", gap: "8px" }}>
          {joy.buttons.map((value, idx) => (
            <div
              key={idx}
              style={{
                padding: "10px 8px",
                border: "1px solid #555",
                backgroundColor: value === 1 ? "#4a4" : "#2a2a2a",
                textAlign: "center",
                color: value === 1 ? "#000" : "#ccc",
                borderRadius: "6px",
                transition: "all 0.15s ease",
              }}
            >
              <div style={{ fontSize: "16px", fontWeight: "600", color: value === 1 ? "#000" : "#fff", marginBottom: "4px" }}>
                {buttonNames[idx] || `B${idx}`}
              </div>
              <div style={{ fontSize: "11px", fontWeight: "500", color: value === 1 ? "#333" : "#888", marginBottom: "6px" }}>
                [{idx}]
              </div>
              <div style={{ fontWeight: "bold", fontSize: "14px" }}>{value}</div>
            </div>
          ))}
        </div>
      </div>

      <div>
        <h4 style={{ margin: "0 0 12px 0", fontSize: "14px", fontWeight: "500", color: "#bbb" }}>
          Axes <span style={{ color: "#666" }}>({joy.axes.length})</span>
        </h4>
        <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fill, minmax(100px, 1fr))", gap: "8px" }}>
          {joy.axes.map((value, idx) => (
            <div
              key={idx}
              style={{
                padding: "10px 8px",
                border: "1px solid #555",
                backgroundColor: "#2a2a2a",
                borderRadius: "6px",
                textAlign: "center",
              }}
            >
              <div style={{ fontSize: "13px", fontWeight: "600", color: "#fff", marginBottom: "4px" }}>
                {axisNames[idx] || `Axis ${idx}`}
              </div>
              <div style={{ fontSize: "11px", fontWeight: "500", color: "#888", marginBottom: "6px" }}>
                [{idx}]
              </div>
              <div style={{ fontWeight: "bold", fontSize: "14px", color: Math.abs(value) > 0.1 ? "#4af" : "#888" }}>
                {value.toFixed(2)}
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
