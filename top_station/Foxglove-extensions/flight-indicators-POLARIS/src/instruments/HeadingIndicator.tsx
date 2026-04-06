import FlightIndicators from "flight-indicators-js";
import { ReactElement, useEffect, useRef } from "react";

import { BundledFlightIndicators } from "../BundledFlightIndicators";

type Props = { heading: number | undefined; size: string };

function setNorthUpHeading(el: HTMLDivElement, heading: number): void {
  const rose = el.querySelector<HTMLElement>("div.instrument.heading div.heading");
  if (rose) {
    rose.style.transform = "rotate(0deg)";
  }

  const airplane = el.querySelector<HTMLElement>("div.instrument.heading img.heading-airplane");
  if (airplane) {
    airplane.style.transform = `rotate(${heading}deg)`;
    airplane.style.transformOrigin = "50% 50%";
  }
}

export function HeadingIndicator({ heading, size }: Props): ReactElement {
  const containerRef = useRef<HTMLDivElement>(null);
  const fi = useRef<BundledFlightIndicators | null>(null);

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return undefined;
    fi.current = new BundledFlightIndicators(el, FlightIndicators.TYPE_HEADING);
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
    if (heading == null) return;
    const el = containerRef.current;
    if (!el) return;
    setNorthUpHeading(el, heading);
  }, [heading]);

  return <div ref={containerRef} />;
}
