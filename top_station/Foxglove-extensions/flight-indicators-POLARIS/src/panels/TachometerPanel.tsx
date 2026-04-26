import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";

import { Tachometer } from "../instruments/Tachometer";
import { useInstrumentPanel } from "../useInstrumentPanel";

type Config = {
  rpmPath: string;
};

const defaultConfig: Config = {
  rpmPath: "",
};

function TachometerPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config, setConfig] = useState<Config>(() => ({
    ...defaultConfig,
    ...(context.initialState as Partial<Config>),
  }));

  const { getValue, containerRef, size } = useInstrumentPanel(context, [config.rpmPath]);
  const rpm = getValue(config.rpmPath);

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
            rpmPath: { label: "RPM", input: "messagepath", value: config.rpmPath },
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
      <Tachometer rpm={rpm} size={size} />
    </div>
  );
}

export function initTachometerPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<TachometerPanel context={context} />);
  return () => { root.unmount(); };
}
