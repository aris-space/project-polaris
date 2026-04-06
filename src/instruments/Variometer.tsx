import FlightIndicators from "flight-indicators-js";
import { ReactElement, useEffect, useRef } from "react";

import { BundledFlightIndicators } from "../BundledFlightIndicators";

// The library's updateVerticalSpeed expects a value in the range ±1.95.
// Panels publish vertical speed in ft/min, so we divide by 1000 here.
const FT_MIN_TO_UNIT = 1000;

type Props = { vario: number | undefined; size: string };

export function Variometer({ vario, size }: Props): ReactElement {
  const containerRef = useRef<HTMLDivElement>(null);
  const fi = useRef<BundledFlightIndicators | null>(null);

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    fi.current = new BundledFlightIndicators(el, FlightIndicators.TYPE_VERTICAL_SPEED);
    return () => {
      el.innerHTML = "";
      fi.current = null;
    };
  }, []);

  useEffect(() => {
    const n = parseInt(size, 10);
    if (!isNaN(n)) fi.current?.resize(n);
  }, [size]);

  useEffect(() => {
    if (vario != null) fi.current?.updateVerticalSpeed(vario / FT_MIN_TO_UNIT);
  }, [vario]);

  return <div ref={containerRef} />;
}
