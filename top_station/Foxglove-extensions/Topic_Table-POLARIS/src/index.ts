import { ExtensionContext } from "@foxglove/extension";

import { initExamplePanel } from "./Main-panel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "ROS topics table (POLARIS)",
    initPanel: initExamplePanel,
  });
}
