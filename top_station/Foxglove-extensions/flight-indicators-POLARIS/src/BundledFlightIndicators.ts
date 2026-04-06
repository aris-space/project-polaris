import FlightIndicators from "flight-indicators-js";

// Inlined from flight-indicators-js/css/flight-indicators.css (MIT).
// Injected once into the document head to avoid a dependency on style-loader.
const INSTRUMENT_CSS = `
.instrument { width: 250px; height: 250px; position: relative; display: inline-block; overflow: hidden; }
.instrument .box { position: absolute; top: 0; left: 0; width: 100%; height: 100%; }
.instrument.attitude .roll { transform: rotate(0deg); }
.instrument.attitude .roll .pitch { top: 0; }
.instrument.heading .yaw { transform: rotate(0deg); }
.instrument.vertical-speed .vertical-speed { transform: rotate(0deg); }
.instrument.speed .airspeed { transform: rotate(90deg); }
.instrument.altimeter .pressure { transform: rotate(40deg); }
.instrument.altimeter .needle { transform: rotate(90deg); }
.instrument.altimeter .small-needle { transform: rotate(90deg); }
.indicators .hidden { display: none; }
`;

let cssInjected = false;
export function ensureCss(): void {
  if (cssInjected) return;
  const style = document.createElement("style");
  style.textContent = INSTRUMENT_CSS;
  document.head.appendChild(style);
  cssInjected = true;
}

// SVG assets are vendored into src/assets/instruments/ (MIT-licensed, from flight-indicators-js).
// Imported as raw strings via webpack's asset/source loader (?raw query).
import altitudePressure from "./assets/instruments/altitude_pressure.svg?raw";
import altitudeTicks from "./assets/instruments/altitude_ticks.svg?raw";
import fiCircle from "./assets/instruments/fi_circle.svg?raw";
import fiNeedleSmall from "./assets/instruments/fi_needle_small.svg?raw";
import fiNeedle from "./assets/instruments/fi_needle.svg?raw";
import fiTcAirplane from "./assets/instruments/fi_tc_airplane.svg?raw";
import headingMechanics from "./assets/instruments/heading_mechanics.svg?raw";
import headingYaw from "./assets/instruments/heading_yaw.svg?raw";
import horizonBack from "./assets/instruments/horizon_back.svg?raw";
import horizonBall from "./assets/instruments/horizon_ball.svg?raw";
import horizonCircle from "./assets/instruments/horizon_circle.svg?raw";
import horizonMechanics from "./assets/instruments/horizon_mechanics.svg?raw";
import speedMechanics from "./assets/instruments/speed_mechanics.svg?raw";
import turnCoordinator from "./assets/instruments/turn_coordinator.svg?raw";
import verticalMechanics from "./assets/instruments/vertical_mechanics.svg?raw";

function toDataUrl(svgRaw: string): string {
  return `data:image/svg+xml;charset=utf-8,${encodeURIComponent(svgRaw)}`;
}

const SVG_DATA_URLS: Record<string, string> = {
  "altitude_pressure.svg": toDataUrl(altitudePressure),
  "altitude_ticks.svg": toDataUrl(altitudeTicks),
  "fi_circle.svg": toDataUrl(fiCircle),
  "fi_needle_small.svg": toDataUrl(fiNeedleSmall),
  "fi_needle.svg": toDataUrl(fiNeedle),
  "fi_tc_airplane.svg": toDataUrl(fiTcAirplane),
  "heading_mechanics.svg": toDataUrl(headingMechanics),
  "heading_yaw.svg": toDataUrl(headingYaw),
  "horizon_back.svg": toDataUrl(horizonBack),
  "horizon_ball.svg": toDataUrl(horizonBall),
  "horizon_circle.svg": toDataUrl(horizonCircle),
  "horizon_mechanics.svg": toDataUrl(horizonMechanics),
  "speed_mechanics.svg": toDataUrl(speedMechanics),
  "turn_coordinator.svg": toDataUrl(turnCoordinator),
  "vertical_mechanics.svg": toDataUrl(verticalMechanics),
};

/**
 * FlightIndicators subclass that loads all SVG assets as inline data URLs,
 * making it safe to use inside a bundled Foxglove extension where no
 * static file server is available.
 */
export class BundledFlightIndicators extends FlightIndicators {
  constructor(
    placeholder: HTMLElement,
    type: string,
    options?: ConstructorParameters<typeof FlightIndicators>[2],
  ) {
    ensureCss();
    super(placeholder, type, options);
  }

  override createImgBox(_imgDirectory: string, imgSrc: string): HTMLImageElement {
    const img = document.createElement("img");
    img.setAttribute("class", "box");
    if (imgSrc === "fi_box.svg") {
      img.style.display = "none";
    } else {
      img.src = SVG_DATA_URLS[imgSrc] ?? imgSrc;
    }
    return img;
  }
}
