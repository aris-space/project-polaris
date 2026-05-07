import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";
import { Variometer } from "../instruments/Variometer";

import { useInstrumentPanel } from "../useInstrumentPanel";

type Config = {
  varioPath: string;
};

const defaultConfig: Config = {
  varioPath: "",
};

function VariometerPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config, setConfig] = useState<Config>(() => ({
    ...defaultConfig,
    ...(context.initialState as Partial<Config>),
  }));

  const { getValue, containerRef, size } = useInstrumentPanel(context, [config.varioPath]);
  const vario = getValue(config.varioPath);

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
            varioPath: { label: "Vario (ft/min)", input: "messagepath", value: config.varioPath },
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
      <Variometer vario={vario} size={size} />
    </div>
  );
}

export function initVariometerPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<VariometerPanel context={context} />);
  return () => { root.unmount(); };
}
