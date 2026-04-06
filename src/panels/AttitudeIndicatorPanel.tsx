import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";
import { AttitudeIndicator } from "../instruments/AttitudeIndicator";

import { useInstrumentPanel } from "../useInstrumentPanel";

type Config = {
  pitchPath: string;
  rollPath: string;
};

const defaultConfig: Config = {
  pitchPath: "",
  rollPath: "",
};

function AttitudeIndicatorPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config, setConfig] = useState<Config>(() => ({
    ...defaultConfig,
    ...(context.initialState as Partial<Config>),
  }));

  const { getValue, containerRef, size } = useInstrumentPanel(context, [
    config.pitchPath,
    config.rollPath,
  ]);
  const pitch = getValue(config.pitchPath);
  const roll = getValue(config.rollPath);

  useEffect(() => {
    context.updatePanelSettingsEditor({
      actionHandler: (action: SettingsTreeAction) => {
        if (action.action === "update") {
          const { path, value } = action.payload;
          setConfig((prev) => {
            const next = { ...prev, [path[1] as keyof Config]: value as string };
            context.saveState(next);
            return next;
          });
        }
      },
      nodes: {
        general: {
          label: "General",
          fields: {
            pitchPath: { label: "Pitch (deg)", input: "messagepath", value: config.pitchPath },
            rollPath: { label: "Roll (deg)", input: "messagepath", value: config.rollPath },
          },
        },
      },
    });
  }, [context, config]);

  return (
    <div
      ref={containerRef}
      style={{ display: "flex", alignItems: "center", justifyContent: "center", width: "100%", height: "100%" }}
    >
      <AttitudeIndicator pitch={pitch} roll={roll} size={size} />
    </div>
  );
}

export function initAttitudeIndicatorPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<AttitudeIndicatorPanel context={context} />);
  return () => { root.unmount(); };
}
