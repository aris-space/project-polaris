import { ExtensionContext } from "@foxglove/extension";

import { initAirspeedPanel } from "./panels/AirspeedPanel";
import { initAltimeterPanel } from "./panels/AltimeterPanelImpl";
import { initAttitudeIndicatorPanel } from "./panels/AttitudeIndicatorPanel";
import { initDualFuelGaugePanel } from "./panels/DualFuelGaugePanel";
import { initDualOilGaugePanel } from "./panels/DualOilGaugePanel";
import { initDualGaugePanel } from "./panels/DualGaugePanel";
import { initDualTachometerPanel } from "./panels/DualTachometerPanel";
import { initHeadingIndicatorPanel } from "./panels/HeadingIndicatorPanel";
import { initTachometerPanel } from "./panels/TachometerPanel";
import { initTurnCoordinatorPanel } from "./panels/TurnCoordinatorPanel";
import { initVariometerPanel } from "./panels/VariometerPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "ROS airspeed POLARIS", initPanel: initAirspeedPanel });
  extensionContext.registerPanel({ name: "ROS altimeter POLARIS", initPanel: initAltimeterPanel });
  extensionContext.registerPanel({ name: "ROS attitude indicator POLARIS", initPanel: initAttitudeIndicatorPanel });
  extensionContext.registerPanel({ name: "ROS dual fuel gauge POLARIS", initPanel: initDualFuelGaugePanel });
  extensionContext.registerPanel({ name: "ROS dual oil gauge POLARIS", initPanel: initDualOilGaugePanel });
  extensionContext.registerPanel({ name: "ROS dual gauge POLARIS", initPanel: initDualGaugePanel });
  extensionContext.registerPanel({ name: "ROS dual tachometer POLARIS", initPanel: initDualTachometerPanel });
  extensionContext.registerPanel({ name: "ROS heading indicator POLARIS", initPanel: initHeadingIndicatorPanel });
  extensionContext.registerPanel({ name: "ROS tachometer POLARIS", initPanel: initTachometerPanel });
  extensionContext.registerPanel({ name: "ROS turn coordinator POLARIS", initPanel: initTurnCoordinatorPanel });
  extensionContext.registerPanel({ name: "ROS variometer POLARIS", initPanel: initVariometerPanel });
}
