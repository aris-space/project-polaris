import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { ReactElement, useEffect, useState } from "react";
import { createRoot } from "react-dom/client";
import { HeadingIndicator } from "../instruments/HeadingIndicator";

import { useInstrumentPanel } from "../useInstrumentPanel";
import { quaternionToHeadingDegrees } from "../utils";

type OrientationMode = "degrees" | "quaternion";

type Config = {
  orientationMode: OrientationMode;
  headingPath: string;
  quaternionWPath: string;
  quaternionXPath: string;
  quaternionYPath: string;
  quaternionZPath: string;
};

const defaultConfig: Config = {
  orientationMode: "degrees",
  headingPath: "",
  quaternionWPath: "",
  quaternionXPath: "",
  quaternionYPath: "",
  quaternionZPath: "",
};

function HeadingIndicatorPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [config, setConfig] = useState<Config>(() => ({
    ...defaultConfig,
    ...(context.initialState as Partial<Config>),
  }));

  const messagePaths = [
    config.headingPath,
    config.quaternionWPath,
    config.quaternionXPath,
    config.quaternionYPath,
    config.quaternionZPath,
  ].filter(Boolean);
  const { getValue, containerRef, size } = useInstrumentPanel(context, messagePaths);

  let heading: number | undefined;

  if (config.orientationMode === "degrees") {
    const rawHeading = getValue(config.headingPath);
    // Normalize to [0, 360) so values like 400° or -90° map correctly on the dial
    heading = rawHeading != null ? ((rawHeading % 360) + 360) % 360 : undefined;
  } else {
    // Use separate quaternion component paths (w/x/y/z).
    const qw = getValue(config.quaternionWPath);
    const qx = getValue(config.quaternionXPath);
    const qy = getValue(config.quaternionYPath);
    const qz = getValue(config.quaternionZPath);
    const hasSeparateQuaternion = [qw, qx, qy, qz].every((value) => value != undefined);
    const quaternion = hasSeparateQuaternion
      ? { w: qw as number, x: qx as number, y: qy as number, z: qz as number }
      : undefined;

    if (quaternion) {
      heading = quaternionToHeadingDegrees(quaternion);
    }
  }

  useEffect(() => {
    const fields: Record<string, any> = {
      orientationMode: {
        label: "Orientation Mode",
        input: "select",
        options: [
          { label: "Degrees (deg)", value: "degrees" },
          { label: "Quaternion (wxyz)", value: "quaternion" },
        ],
        value: config.orientationMode,
      },
    };

    if (config.orientationMode === "degrees") {
      fields.headingPath = {
        label: "Heading (deg)",
        input: "messagepath",
        value: config.headingPath,
      };
    } else {
      fields.quaternionWPath = {
        label: "Quaternion w",
        input: "messagepath",
        value: config.quaternionWPath,
      };
      fields.quaternionXPath = {
        label: "Quaternion x",
        input: "messagepath",
        value: config.quaternionXPath,
      };
      fields.quaternionYPath = {
        label: "Quaternion y",
        input: "messagepath",
        value: config.quaternionYPath,
      };
      fields.quaternionZPath = {
        label: "Quaternion z",
        input: "messagepath",
        value: config.quaternionZPath,
      };
    }

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
          fields,
        },
      },
    });
  }, [
    context,
    config.orientationMode,
    config.headingPath,
    config.quaternionWPath,
    config.quaternionXPath,
    config.quaternionYPath,
    config.quaternionZPath,
  ]);

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
  return () => {
    root.unmount();
  };
}
