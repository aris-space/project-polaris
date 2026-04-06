import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";
import { HeadingIndicator } from "../instruments/HeadingIndicator";

import { useInstrumentPanel } from "../useInstrumentPanel";

type Config = {
  headingPath: string;
};

const defaultConfig: Config = {
  headingPath: "",
};

function HeadingIndicatorPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config, setConfig] = useState<Config>(() => ({
    ...defaultConfig,
    ...(context.initialState as Partial<Config>),
  }));

  const { getValue, containerRef, size } = useInstrumentPanel(context, [config.headingPath]);
  const rawHeading = getValue(config.headingPath);
  // Normalize to [0, 360) so values like 400° or -90° map correctly on the dial
  const heading = rawHeading != null ? ((rawHeading % 360) + 360) % 360 : undefined;

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
            headingPath: { label: "Heading (deg)", input: "messagepath", value: config.headingPath },
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
      <HeadingIndicator heading={heading} size={size} />
    </div>
  );
}

export function initHeadingIndicatorPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<HeadingIndicatorPanel context={context} />);
  return () => { root.unmount(); };
}
