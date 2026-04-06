import { ReactElement, useEffect, useRef } from "react";

import { ensureCss } from "../BundledFlightIndicators";
import fiCircleRaw from "../assets/instruments/fi_circle.svg?raw";
import fiNeedleRaw from "../assets/instruments/fi_needle.svg?raw";
import tachometerMechanicsRaw from "../assets/instruments/tachometer_mechanics.svg?raw";

function toDataUrl(svgRaw: string): string {
  return `data:image/svg+xml;charset=utf-8,${encodeURIComponent(svgRaw)}`;
}

const FACE_URL = toDataUrl(tachometerMechanicsRaw);
const NEEDLE_URL = toDataUrl(fiNeedleRaw);
const CIRCLE_URL = toDataUrl(fiCircleRaw);

// Scale: 0–8000 RPM mapped to a 320° sweep (same as the airspeed indicator).
// Needle rotation formula: rotate(90 + (rpm / 8000) * 320)deg
//   rpm=0    → rotate(90deg)  = 0 RPM mark (12 o'clock on the face)
//   rpm=8000 → rotate(410deg) = 8000 RPM mark (~10:30 o'clock on the face)
const MAX_RPM = 8000;
const BASE_DEG = 90;
const SWEEP_DEG = 320;

type Props = { rpm: number | undefined; size: string };

export function Tachometer({ rpm, size }: Props): ReactElement {
  const containerRef = useRef<HTMLDivElement>(null);
  const needleRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;

    ensureCss();

    const instrument = document.createElement("div");
    instrument.className = "instrument tachometer";

    const face = document.createElement("img");
    face.className = "box";
    face.src = FACE_URL;
    instrument.appendChild(face);

    const needleDiv = document.createElement("div");
    needleDiv.className = "box";
    needleDiv.style.transform = `rotate(${BASE_DEG}deg)`;
    const needleImg = document.createElement("img");
    needleImg.className = "box";
    needleImg.src = NEEDLE_URL;
    needleDiv.appendChild(needleImg);
    instrument.appendChild(needleDiv);

    const capDiv = document.createElement("div");
    capDiv.className = "mechanics box";
    const capImg = document.createElement("img");
    capImg.className = "box";
    capImg.src = CIRCLE_URL;
    capDiv.appendChild(capImg);
    instrument.appendChild(capDiv);

    el.appendChild(instrument);
    needleRef.current = needleDiv;

    return () => {
      el.innerHTML = "";
      needleRef.current = null;
    };
  }, []);

  useEffect(() => {
    const n = parseInt(size, 10);
    if (!isNaN(n)) {
      const instrument = containerRef.current?.querySelector<HTMLElement>(".instrument");
      if (instrument) {
        instrument.style.width = `${n}px`;
        instrument.style.height = `${n}px`;
      }
    }
  }, [size]);

  useEffect(() => {
    if (rpm == null || !needleRef.current) return;
    const clamped = Math.max(0, Math.min(MAX_RPM, rpm));
    const rotation = BASE_DEG + (clamped / MAX_RPM) * SWEEP_DEG;
    needleRef.current.style.transform = `rotate(${rotation}deg)`;
  }, [rpm]);

  return <div ref={containerRef} />;
}
