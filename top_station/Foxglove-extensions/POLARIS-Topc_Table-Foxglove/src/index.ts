import { ExtensionContext } from "@foxglove/extension";

import { initExamplePanel } from "./Main-panel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "polaris-ros-topic-tables",
    initPanel: initExamplePanel,
  });
}
