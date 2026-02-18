import { Button, LinearProgress } from "@mui/material";

import { Joy } from "../types";

// eslint-disable-next-line no-warning-comments
// TODO copy theming from another extension

interface SimpleButtonViewProps {
  joy?: Joy;
}

export function SimpleButtonView({ joy }: SimpleButtonViewProps): JSX.Element {
  const buttons = joy
    ? joy.buttons.map((item, index) => (
        <Button
          key={`btn-${index}`}
          variant={item > 0 ? "contained" : "outlined"}
          size="large"
          color={item > 0 ? "error" : "primary"}
        >
          {index}
        </Button>
      ))
    : [];

  const axes = joy
    ? joy.axes.map((item, index) => (
        <LinearProgress
          key={`axis-${index}`}
          variant="determinate"
          value={item * 50 + 50}
          sx={{ transition: "none" }}
        />
      ))
    : [];

  return (
    <div>
      {joy ? null : "Waiting for first data..."}
      {buttons}
      {axes}
      {/* {JSON.stringify(joy)} */}
    </div>
  );
}
