import { ExtensionContext } from "@foxglove/extension";

import { initMissionPlannerPanel } from "./MissionPlannerPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Mission Planner (POLARIS)",
    initPanel: initMissionPlannerPanel,
  });
}
