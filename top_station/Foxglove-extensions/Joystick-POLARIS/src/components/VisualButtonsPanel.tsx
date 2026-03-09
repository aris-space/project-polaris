import { Button } from "@mui/material";

import { VisualButtonMapping } from "../panelSettings";

interface VisualButtonsPanelProps {
  mappings: VisualButtonMapping[];
  activeIndices: Set<number>;
  onPress: (index: number) => void;
  onRelease: (index: number) => void;
}

export function VisualButtonsPanel({
  mappings,
  activeIndices,
  onPress,
  onRelease,
}: VisualButtonsPanelProps): JSX.Element {
  return (
    <div
      style={{
        display: "grid",
        gridTemplateColumns: "repeat(auto-fit, minmax(120px, 1fr))",
        gap: "12px",
        width: "100%",
      }}
    >
      {mappings.map((mapping, idx) => {
        const isActive = activeIndices.has(idx);
        return (
          <Button
            key={`vb-${idx}`}
            variant={isActive ? "contained" : "outlined"}
            color={isActive ? "success" : "primary"}
            size="large"
            onPointerDown={(e) => {
              e.preventDefault();
              onPress(idx);
            }}
            onPointerUp={(e) => {
              e.preventDefault();
              onRelease(idx);
            }}
            onPointerCancel={(e) => {
              e.preventDefault();
              onRelease(idx);
            }}
            onPointerLeave={(e) => {
              if (e.buttons === 0) {
                onRelease(idx);
              }
            }}
            sx={{
              minHeight: "56px",
              fontWeight: 700,
              textTransform: "none",
            }}
          >
            {mapping.label || `B${idx + 1}`}
          </Button>
        );
      })}
    </div>
  );
}
