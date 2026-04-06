import FlightIndicators from "flight-indicators-js";
import { ReactElement, useEffect, useRef } from "react";

import { BundledFlightIndicators } from "../BundledFlightIndicators";

type Props = { speed: number | undefined; size: string };

export function Airspeed({ speed, size }: Props): ReactElement {
  const containerRef = useRef<HTMLDivElement>(null);
  const fi = useRef<BundledFlightIndicators | null>(null);

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    fi.current = new BundledFlightIndicators(el, FlightIndicators.TYPE_AIRSPEED);
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
    if (speed != null) fi.current?.updateAirSpeed(speed);
  }, [speed]);

  return <div ref={containerRef} />;
}
