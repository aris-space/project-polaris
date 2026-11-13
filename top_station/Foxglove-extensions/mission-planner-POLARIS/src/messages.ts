import { Fix } from "./types";

function num(v: unknown): number | undefined {
  if (typeof v === "number" && Number.isFinite(v)) {
    return v;
  }
  if (typeof v === "bigint") {
    return Number(v);
  }
  return undefined;
}

/**
 * Decode a position from sensor_msgs/NavSatFix or foxglove.LocationFix.
 * Both carry `latitude`, `longitude` and a row-major 3x3 `position_covariance`.
 * NavSatFix additionally carries `status.status`.
 */
export function extractFix(message: unknown, receivedAt: number): Fix | undefined {
  if (message == undefined || typeof message !== "object") {
    return undefined;
  }
  const m = message as Record<string, unknown>;
  const lat = num(m.latitude);
  const lon = num(m.longitude);
  if (lat == undefined || lon == undefined) {
    return undefined;
  }
  if (Math.abs(lat) > 90 || Math.abs(lon) > 180 || (lat === 0 && lon === 0)) {
    return undefined;
  }

  let status: number | undefined;
  const st = m.status;
  if (st != undefined && typeof st === "object") {
    status = num((st as Record<string, unknown>).status);
  }

  let hAcc: number | undefined;
  const cov = m.position_covariance as ArrayLike<unknown> | undefined;
  const covType = num(m.position_covariance_type);
  if (cov != undefined && cov.length >= 5 && covType !== 0) {
    const ex = num(cov[0]);
    const ny = num(cov[4]);
    if (ex != undefined && ny != undefined && ex >= 0 && ny >= 0) {
      hAcc = Math.sqrt(Math.max(ex, ny));
    }
  }

  return { lat, lon, status, hAcc, receivedAt };
}
