// Allow importing any file as a raw string with the ?raw query suffix.
// This is handled by webpack's asset/source loader (resourceQuery: /\?raw$/).
declare module "*.svg?raw" {
  const content: string;
  export default content;
}

declare module "flight-indicators-js" {
  interface FlightIndicatorsOptions {
    size?: number;
    roll?: number;
    pitch?: number;
    turn?: number;
    heading?: number;
    verticalSpeed?: number;
    airspeed?: number;
    altitude?: number;
    pressure?: number;
    hideBox?: boolean;
    imagesDirectory?: string;
  }

  class FlightIndicators {
    static TYPE_HEADING: string;
    static TYPE_AIRSPEED: string;
    static TYPE_ALTIMETER: string;
    static TYPE_VERTICAL_SPEED: string;
    static TYPE_ATTITUDE: string;
    static TYPE_TURN_COORDINATOR: string;

    constructor(placeholder: HTMLElement, type: string, options?: FlightIndicatorsOptions);

    createImgBox(imgDirectory: string, imgSrc: string): HTMLImageElement;

    updateHeading(heading: number): void;
    updateRoll(roll: number): void;
    updatePitch(pitch: number): void;
    updateCoordinator(turn: number): void;
    updateVerticalSpeed(vSpeed: number): void;
    updateAirSpeed(speed: number): void;
    updateAltitude(altitude: number): void;
    updatePressure(pressure: number): void;
    resize(size: number): void;
  }

  export default FlightIndicators;
}
