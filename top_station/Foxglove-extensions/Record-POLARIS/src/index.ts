import { ExtensionContext } from "@foxglove/extension";

import { initRecorderActivePanel } from "./RecorderPanel";

export function activate(extensionContext: ExtensionContext): void {
	extensionContext.registerPanel({
		name: "Recorder Active (POLARIS)",
		initPanel: initRecorderActivePanel,
	});
}
