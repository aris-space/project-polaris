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
  const buttonCount = mappings.length;
  const minButtonWidth = buttonCount > 10 ? 82 : 96;

  return (
    <div
      style={{
        display: "grid",
        gridTemplateColumns: `repeat(auto-fill, minmax(${minButtonWidth}px, 1fr))`,
        gap: "8px",
        width: "100%",
      }}
    >
      {mappings.map((mapping, idx) => {
        const isActive = activeIndices.has(idx);
        const buttonColor = mapping.color as "primary" | "secondary" | "success" | "error" | "info" | "warning";
        return (
          <Button
            key={`vb-${idx}`}
            variant={isActive ? "contained" : "outlined"}
            color={buttonColor}
            size="small"
            disableRipple
            disableTouchRipple
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
              minHeight: "40px",
              px: 1,
              fontWeight: 700,
              fontSize: "0.78rem",
              lineHeight: 1.2,
              textTransform: "none",
              transition: "none",
            }}
          >
            {mapping.label || `B${idx + 1}`}
          </Button>
        );
      })}
    </div>
  );
}
