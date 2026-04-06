import { ExtensionContext } from "@foxglove/extension";

import { initAirspeedPanel } from "./panels/AirspeedPanel";
import { initAltimeterPanel } from "./panels/AltimeterPanel";
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
  extensionContext.registerPanel({ name: "Airspeed", initPanel: initAirspeedPanel });
  extensionContext.registerPanel({ name: "Altimeter", initPanel: initAltimeterPanel });
  extensionContext.registerPanel({ name: "Attitude Indicator", initPanel: initAttitudeIndicatorPanel });
  extensionContext.registerPanel({ name: "Dual Fuel Gauge", initPanel: initDualFuelGaugePanel });
  extensionContext.registerPanel({ name: "Dual Oil Gauge", initPanel: initDualOilGaugePanel });
  extensionContext.registerPanel({ name: "Dual Gauge", initPanel: initDualGaugePanel });
  extensionContext.registerPanel({ name: "Dual Tachometer", initPanel: initDualTachometerPanel });
  extensionContext.registerPanel({ name: "Heading Indicator", initPanel: initHeadingIndicatorPanel });
  extensionContext.registerPanel({ name: "Tachometer", initPanel: initTachometerPanel });
  extensionContext.registerPanel({ name: "Turn Coordinator", initPanel: initTurnCoordinatorPanel });
  extensionContext.registerPanel({ name: "Variometer", initPanel: initVariometerPanel });
}
