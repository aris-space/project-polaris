import { Button, Typography } from "@mui/material";

import { VisualButtonMapping, ButtonSection } from "../panelSettings";

interface VisualButtonsPanelProps {
  mappings: VisualButtonMapping[];
  sections: ButtonSection[];
  activeIndices: Set<number>;
  onPress: (index: number) => void;
  onRelease: (index: number) => void;
}

export function VisualButtonsPanel({
  mappings,
  sections,
  activeIndices,
  onPress,
  onRelease,
}: VisualButtonsPanelProps): JSX.Element {
  const buttonCount = mappings.length;
  const minButtonWidth = buttonCount > 10 ? 76 : 88;

  return (
    <div style={{ width: "100%" }}>
      {sections.map((section, sectionIdx) => (
        <div key={`section-${sectionIdx}`} style={{ marginBottom: "8px" }}>
          <Typography
            variant="subtitle2"
            sx={{
              fontWeight: 600,
              marginBottom: "4px",
              color: "white",
              textTransform: "uppercase",
              fontSize: "0.68rem",
              letterSpacing: "0.35px",
            }}
          >
            {section.title}
          </Typography>
          <div
            style={{
              display: "grid",
              gridTemplateColumns: `repeat(auto-fill, minmax(${minButtonWidth}px, 1fr))`,
              gap: "6px",
              width: "100%",
            }}
          >
            {section.buttonIndices.map((btnIdx) => {
              const mapping = mappings[btnIdx];
              if (!mapping) {
                return null;
              }

              const isActive = activeIndices.has(btnIdx);
              return (
                <Button
                  key={`vb-${btnIdx}`}
                  variant={isActive ? "contained" : "outlined"}
                  color="primary"
                  size="small"
                  disableRipple
                  disableTouchRipple
                  onPointerDown={(e) => {
                    e.preventDefault();
                    onPress(btnIdx);
                  }}
                  onPointerUp={(e) => {
                    e.preventDefault();
                    onRelease(btnIdx);
                  }}
                  onPointerCancel={(e) => {
                    e.preventDefault();
                    onRelease(btnIdx);
                  }}
                  onPointerLeave={(e) => {
                    if (e.buttons === 0) {
                      onRelease(btnIdx);
                    }
                  }}
                  sx={{
                    minHeight: "34px",
                    px: 0.75,
                    fontWeight: 700,
                    fontSize: "0.72rem",
                    lineHeight: 1.2,
                    textTransform: "none",
                    transition: "none",
                  }}
                >
                  {mapping.label || `B${btnIdx + 1}`}
                </Button>
              );
            })}
          </div>
        </div>
      ))}
    </div>
  );
}
