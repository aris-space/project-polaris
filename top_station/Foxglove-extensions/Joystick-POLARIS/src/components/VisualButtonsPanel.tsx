import { Button, Typography } from "@mui/material";

import { VisualButtonMapping, ButtonSection } from "../panelSettings";

interface VisualButtonsPanelProps {
  mappings: VisualButtonMapping[];
  sections: ButtonSection[];
  activeIndices: Set<number>;
  onPress: (index: number) => void;
  onRelease: (index: number) => void;
  inputEnabled?: boolean;
}

export function VisualButtonsPanel({
  mappings,
  sections,
  activeIndices,
  onPress,
  onRelease,
  inputEnabled = true,
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
                  disabled={!inputEnabled}
                  disableRipple
                  disableTouchRipple
                  onPointerDown={(e) => {
                    if (!inputEnabled) return;
                    e.preventDefault();
                    onPress(btnIdx);
                  }}
                  onPointerUp={(e) => {
                    if (!inputEnabled) return;
                    e.preventDefault();
                    onRelease(btnIdx);
                  }}
                  onPointerCancel={(e) => {
                    if (!inputEnabled) return;
                    e.preventDefault();
                    onRelease(btnIdx);
                  }}
                  onPointerLeave={(e) => {
                    if (!inputEnabled) return;
                    e.preventDefault();
                    onRelease(btnIdx);
                  }}
                  sx={{
                    minHeight: "34px",
                    px: 0.75,
                    fontWeight: 400,
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
