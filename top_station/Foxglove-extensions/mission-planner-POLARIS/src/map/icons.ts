import * as L from "leaflet";

/** Polygon vertex handle. `first` highlights the vertex that closes the polygon. */
export function vertexIcon(opts: {
  first?: boolean;
  closable?: boolean;
  variant?: "safe" | "survey";
}): L.DivIcon {
  const cls = ["mp-vertex"];
  if (opts.variant === "survey") {
    cls.push("mp-vertex--survey");
  }
  if (opts.first === true) {
    cls.push("mp-vertex--first");
  }
  if (opts.closable === true) {
    cls.push("mp-vertex--closable");
  }
  return L.divIcon({
    className: "mp-icon",
    html: `<div class="${cls.join(" ")}"></div>`,
    iconSize: [16, 16],
    iconAnchor: [8, 8],
  });
}

/** Ghost "+" handle on an edge midpoint, drag it to insert a vertex. */
export function midpointIcon(opts: { variant?: "safe" | "survey" } = {}): L.DivIcon {
  return L.divIcon({
    className: "mp-icon",
    html: `<div class="mp-midpoint${opts.variant === "survey" ? " mp-midpoint--survey" : ""}">+</div>`,
    iconSize: [14, 14],
    iconAnchor: [7, 7],
  });
}

/** Start point: a cross with a halo. */
export function startIcon({ outside }: { outside: boolean }): L.DivIcon {
  return L.divIcon({
    className: "mp-icon",
    html: `<div class="mp-start${outside ? " mp-start--outside" : ""}">
      <svg viewBox="0 0 32 32" width="32" height="32" aria-hidden="true">
        <circle cx="16" cy="16" r="13" class="mp-start-ring"/>
        <path d="M16 4 V28 M4 16 H28" class="mp-start-cross-halo"/>
        <path d="M16 4 V28 M4 16 H28" class="mp-start-cross"/>
      </svg>
      <span class="mp-start-label">START</span>
    </div>`,
    iconSize: [32, 32],
    iconAnchor: [16, 16],
  });
}

const SURFACE_SVG =
  '<svg viewBox="0 0 12 12" width="9" height="9" aria-hidden="true"><path d="M1.5 1.5h9M6 10.5V4.2M3.4 6.6 6 4l2.6 2.6" fill="none" stroke="currentColor" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/></svg>';
const WAIT_SVG =
  '<svg viewBox="0 0 12 12" width="9" height="9" aria-hidden="true"><path d="M4 2.5v7M8 2.5v7" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round"/></svg>';

export function waypointIcon(
  index: number,
  { outside, action, holdS }: { outside: boolean; action?: "pass" | "wait" | "surface"; holdS?: number },
): L.DivIcon {
  const cls = ["mp-wp"];
  if (outside) {
    cls.push("mp-wp--outside");
  }
  let badge = "";
  if (action === "wait") {
    cls.push("mp-wp--wait");
    badge = `<span class="mp-wp-badge mp-wp-badge--wait">${WAIT_SVG}${holdS ?? 0}s</span>`;
  } else if (action === "surface") {
    cls.push("mp-wp--surface");
    const h = holdS != undefined && holdS > 0 ? `${holdS}s` : "";
    badge = `<span class="mp-wp-badge mp-wp-badge--surface">${SURFACE_SVG}${h}</span>`;
  }
  return L.divIcon({
    className: "mp-icon",
    html: `<div class="${cls.join(" ")}">${index}${badge}</div>`,
    iconSize: [22, 22],
    iconAnchor: [11, 11],
  });
}

export function segmentLabelIcon(
  text: string,
  { outside, variant }: { outside: boolean; variant?: "route" | "survey" | "live" },
): L.DivIcon {
  const cls = ["mp-seglabel"];
  if (variant === "survey") {
    cls.push("mp-seglabel--survey");
  } else if (variant === "live") {
    cls.push("mp-seglabel--live");
  }
  if (outside) {
    cls.push("mp-seglabel--outside");
  }
  return L.divIcon({
    className: `mp-icon mp-label-icon${variant === "live" ? " mp-live-label-icon" : ""}`,
    html: `<div class="${cls.join(" ")}">${text}</div>`,
    iconSize: [0, 0],
    iconAnchor: [0, 0],
  });
}

/** Vehicle marker: arrow when heading is known, dot otherwise. Colour via CSS variable. */
export function vehicleIcon(color: string, headingDeg: number | undefined): L.DivIcon {
  const body =
    headingDeg == undefined
      ? `<svg viewBox="0 0 24 24" width="24" height="24"><circle cx="12" cy="12" r="7" class="mp-veh-shape"/></svg>`
      : `<svg viewBox="0 0 28 28" width="28" height="28" style="transform: rotate(${headingDeg.toFixed(
          1,
        )}deg)"><path d="M14 2 L24 25 L14 19.5 L4 25 Z" class="mp-veh-shape"/></svg>`;
  return L.divIcon({
    className: "mp-icon",
    html: `<div class="mp-vehicle" style="--veh:${color}">${body}</div>`,
    iconSize: [28, 28],
    iconAnchor: [14, 14],
  });
}

export function gnssIcon(): L.DivIcon {
  return L.divIcon({
    className: "mp-icon",
    html: `<div class="mp-gnss"><span class="mp-gnss-pulse"></span><span class="mp-gnss-dot"></span></div>`,
    iconSize: [18, 18],
    iconAnchor: [9, 9],
  });
}
