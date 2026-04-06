import FlightIndicators from "flight-indicators-js";
import { ReactElement, useEffect, useRef } from "react";

import { BundledFlightIndicators } from "../BundledFlightIndicators";

type Props = { altitude: number | undefined; pressure: number | undefined; size: string };

export function Altimeter({ altitude, pressure, size }: Props): ReactElement {
  const containerRef = useRef<HTMLDivElement>(null);
  const fi = useRef<BundledFlightIndicators | null>(null);

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    fi.current = new BundledFlightIndicators(el, FlightIndicators.TYPE_ALTIMETER);
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
    if (altitude != null) fi.current?.updateAltitude(altitude);
  }, [altitude]);

  useEffect(() => {
    if (pressure != null) fi.current?.updatePressure(pressure);
  }, [pressure]);

  return <div ref={containerRef} />;
}
