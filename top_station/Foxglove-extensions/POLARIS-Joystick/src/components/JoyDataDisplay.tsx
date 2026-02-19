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
    return <div style={{ padding: "10px", color: "#999" }}>No joy data</div>;
  }

  return (
    <div style={{ padding: "10px", fontFamily: "monospace", fontSize: "12px" }}>
      <h3 style={{ marginTop: "20px", marginBottom: "10px" }}>Raw Joy Data</h3>
      
      <div style={{ marginBottom: "20px" }}>
        <h4 style={{ marginBottom: "8px" }}>Buttons ({joy.buttons.length})</h4>
        <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fit, minmax(80px, 1fr))", gap: "4px" }}>
          {joy.buttons.map((value, idx) => (
            <div
              key={idx}
              style={{
                padding: "6px",
                border: "1px solid #666",
                backgroundColor: value === 1 ? "#4a4" : "#333",
                textAlign: "center",
                color: value === 1 ? "#000" : "#aaa",
                borderRadius: "4px",
              }}
            >
              <div style={{ fontSize: "10px", color: "#888" }}>{buttonNames[idx] || `B${idx}`}</div>
              <div style={{ fontWeight: "bold" }}>{value}</div>
            </div>
          ))}
        </div>
      </div>

      <div>
        <h4 style={{ marginBottom: "8px" }}>Axes ({joy.axes.length})</h4>
        <div style={{ display: "grid", gridTemplateColumns: "repeat(auto-fit, minmax(100px, 1fr))", gap: "4px" }}>
          {joy.axes.map((value, idx) => (
            <div
              key={idx}
              style={{
                padding: "8px",
                border: "1px solid #666",
                backgroundColor: "#333",
                borderRadius: "4px",
              }}
            >
              <div style={{ fontSize: "10px", color: "#888" }}>{axisNames[idx] || `A${idx}`}</div>
              <div style={{ fontWeight: "bold", color: Math.abs(value) > 0.1 ? "#f88" : "#aaa" }}>
                {value.toFixed(3)}
              </div>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}
