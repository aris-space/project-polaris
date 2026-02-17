import { ExtensionContext } from "@foxglove/studio";

import { initJoyPanel } from "./JoyPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "POLARIS Joystick", initPanel: initJoyPanel });
}
