import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";
import { TurnCoordinator } from "../instruments/TurnCoordinator";

import { useInstrumentPanel } from "../useInstrumentPanel";

type Config = {
  turnPath: string;
};

const defaultConfig: Config = {
  turnPath: "",
};

function TurnCoordinatorPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config, setConfig] = useState<Config>(() => ({
    ...defaultConfig,
    ...(context.initialState as Partial<Config>),
  }));

  const { getValue, containerRef, size } = useInstrumentPanel(context, [config.turnPath]);
  const rawTurn = getValue(config.turnPath);
  // Component has no internal clamping — clamp to ±20° (standard rate turn marks)
  const turn = rawTurn != null ? Math.max(-20, Math.min(20, rawTurn)) : undefined;

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
            turnPath: { label: "Turn (deg)", input: "messagepath", value: config.turnPath },
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
      <TurnCoordinator turn={turn} size={size} />
    </div>
  );
}

export function initTurnCoordinatorPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<TurnCoordinatorPanel context={context} />);
  return () => { root.unmount(); };
}
