import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";
import { Airspeed } from "../instruments/Airspeed";

import { useInstrumentPanel } from "../useInstrumentPanel";

type Config = {
  speedPath: string;
};

const defaultConfig: Config = {
  speedPath: "",
};

function AirspeedPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config, setConfig] = useState<Config>(() => ({
    ...defaultConfig,
    ...(context.initialState as Partial<Config>),
  }));

  const { getValue, containerRef, size } = useInstrumentPanel(context, [config.speedPath]);
  const speed = getValue(config.speedPath);

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
            speedPath: { label: "Speed (knots)", input: "messagepath", value: config.speedPath },
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
      <Airspeed speed={speed} size={size} />
    </div>
  );
}

export function initAirspeedPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<AirspeedPanel context={context} />);
  return () => { root.unmount(); };
}
