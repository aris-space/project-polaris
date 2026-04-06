import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";
import { Altimeter } from "../instruments/Altimeter";

import { useInstrumentPanel } from "../useInstrumentPanel";

type Config = {
  altitudePath: string;
  pressurePath: string;
};

const defaultConfig: Config = {
  altitudePath: "",
  pressurePath: "",
};

function AltimeterPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config, setConfig] = useState<Config>(() => ({
    ...defaultConfig,
    ...(context.initialState as Partial<Config>),
  }));

  const { getValue, containerRef, size } = useInstrumentPanel(context, [
    config.altitudePath,
    config.pressurePath,
  ]);
  const altitude = getValue(config.altitudePath);
  const rawPressure = getValue(config.pressurePath);
  // Component has no internal clamping — clamp to full atmospheric range (hPa)
  const pressure = rawPressure != null ? Math.max(870, Math.min(1084, rawPressure)) : undefined;

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
            altitudePath: { label: "Altitude (ft)", input: "messagepath", value: config.altitudePath },
            pressurePath: { label: "Pressure (hPa)", input: "messagepath", value: config.pressurePath },
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
      <Altimeter altitude={altitude} pressure={pressure} size={size} />
    </div>
  );
}

export function initAltimeterPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<AltimeterPanel context={context} />);
  return () => { root.unmount(); };
}
