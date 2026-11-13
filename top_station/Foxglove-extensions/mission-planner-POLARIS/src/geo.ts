import { LatLon, Mission } from "./types";

const R_EARTH = 6371008.8;
const DEG = Math.PI / 180;

export function distanceM(a: LatLon, b: LatLon): number {
  const dLat = (b.lat - a.lat) * DEG;
  const dLon = (b.lon - a.lon) * DEG;
  const s =
    Math.sin(dLat / 2) ** 2 + Math.cos(a.lat * DEG) * Math.cos(b.lat * DEG) * Math.sin(dLon / 2) ** 2;
  return 2 * R_EARTH * Math.asin(Math.min(1, Math.sqrt(s)));
}

/** Initial bearing a -> b in degrees, 0 = north, clockwise. */
export function bearingDeg(a: LatLon, b: LatLon): number {
  const φ1 = a.lat * DEG;
  const φ2 = b.lat * DEG;
  const dλ = (b.lon - a.lon) * DEG;
  const y = Math.sin(dλ) * Math.cos(φ2);
  const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(dλ);
  return ((Math.atan2(y, x) / DEG) + 360) % 360;
}

type XY = { x: number; y: number };

/** Local equirectangular projection (metres) around a reference — exact enough for lake-sized areas. */
function projector(ref: LatLon): (p: LatLon) => XY {
  const kx = Math.cos(ref.lat * DEG) * R_EARTH * DEG;
  const ky = R_EARTH * DEG;
  return (p) => ({ x: (p.lon - ref.lon) * kx, y: (p.lat - ref.lat) * ky });
}

export function polygonAreaM2(poly: LatLon[]): number {
  if (poly.length < 3) {
    return 0;
  }
  const proj = projector(poly[0]!);
  const pts = poly.map(proj);
  let sum = 0;
  for (let i = 0; i < pts.length; i++) {
    const a = pts[i]!;
    const b = pts[(i + 1) % pts.length]!;
    sum += a.x * b.y - b.x * a.y;
  }
  return Math.abs(sum) / 2;
}

export function pointInPolygon(p: LatLon, poly: LatLon[]): boolean {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const a = poly[i]!;
    const b = poly[j]!;
    if (a.lat > p.lat !== b.lat > p.lat) {
      const x = ((b.lon - a.lon) * (p.lat - a.lat)) / (b.lat - a.lat) + a.lon;
      if (p.lon < x) {
        inside = !inside;
      }
    }
  }
  return inside;
}

function cross(o: XY, a: XY, b: XY): number {
  return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
}

function segmentsProperlyIntersect(p1: XY, p2: XY, q1: XY, q2: XY): boolean {
  const d1 = cross(q1, q2, p1);
  const d2 = cross(q1, q2, p2);
  const d3 = cross(p1, p2, q1);
  const d4 = cross(p1, p2, q2);
  return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
}

/** True if the segment a-b leaves the polygon (an endpoint outside, or it crosses an edge). */
export function segmentLeavesPolygon(a: LatLon, b: LatLon, poly: LatLon[]): boolean {
  if (!pointInPolygon(a, poly) || !pointInPolygon(b, poly)) {
    return true;
  }
  const proj = projector(a);
  const pa = proj(a);
  const pb = proj(b);
  for (let i = 0; i < poly.length; i++) {
    const q1 = proj(poly[i]!);
    const q2 = proj(poly[(i + 1) % poly.length]!);
    if (segmentsProperlyIntersect(pa, pb, q1, q2)) {
      return true;
    }
  }
  return false;
}

/** Ordered route points: start, waypoints…, and start again when the loop is closed. */
export function routePoints(m: Mission): LatLon[] {
  if (!m.start) {
    return [];
  }
  const pts = [m.start, ...m.path];
  if (m.closed && m.path.length >= 2) {
    pts.push(m.start);
  }
  return pts;
}

export type MissionAnalysis = {
  /** Per route segment: does it leave the safe area? (empty when no closed safe area) */
  segmentOutside: boolean[];
  /** Per waypoint (index into mission.path): outside the safe area? */
  waypointOutside: boolean[];
  startOutside: boolean;
  outsideSegments: number;
  routeLengthM: number;
  safeAreaM2: number;
};

export function analyzeMission(m: Mission): MissionAnalysis {
  const hasSafe = m.safeClosed && m.safeArea.length >= 3;
  const route = routePoints(m);
  const segmentOutside: boolean[] = [];
  let routeLengthM = 0;
  for (let i = 0; i + 1 < route.length; i++) {
    const a = route[i]!;
    const b = route[i + 1]!;
    routeLengthM += distanceM(a, b);
    segmentOutside.push(hasSafe ? segmentLeavesPolygon(a, b, m.safeArea) : false);
  }
  return {
    segmentOutside,
    waypointOutside: m.path.map((p) => (hasSafe ? !pointInPolygon(p, m.safeArea) : false)),
    startOutside: hasSafe && m.start != undefined ? !pointInPolygon(m.start, m.safeArea) : false,
    outsideSegments: segmentOutside.filter(Boolean).length,
    routeLengthM,
    safeAreaM2: hasSafe ? polygonAreaM2(m.safeArea) : 0,
  };
}

export function formatDistance(m: number): string {
  if (m < 1000) {
    return `${m.toFixed(m < 10 ? 1 : 0)} m`;
  }
  return `${(m / 1000).toFixed(2)} km`;
}

export function formatArea(m2: number): string {
  if (m2 < 10000) {
    return `${m2.toFixed(0)} m²`;
  }
  return `${(m2 / 10000).toFixed(2)} ha`;
}

export function formatLatLon(p: LatLon): string {
  const lat = `${Math.abs(p.lat).toFixed(6)}° ${p.lat >= 0 ? "N" : "S"}`;
  const lon = `${Math.abs(p.lon).toFixed(6)}° ${p.lon >= 0 ? "E" : "W"}`;
  return `${lat}  ${lon}`;
}
