import { ExtensionContext } from "@foxglove/extension";
import { initValueDisplayPanel } from "./ValueDisplayPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "Value Display POLARIS", initPanel: initValueDisplayPanel });
}
