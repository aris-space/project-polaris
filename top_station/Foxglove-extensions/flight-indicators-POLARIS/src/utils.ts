/**
 * Parse a Foxglove message path string into a topic name and a field path.
 *
 * Examples:
 *   "/altimeter/altitude.data"  → { topic: "/altimeter/altitude", fieldPath: "data" }
 *   "/altimeter.altitude"       → { topic: "/altimeter",          fieldPath: "altitude" }
 *   "/sensors[0].value"         → { topic: "/sensors",            fieldPath: "[0].value" }
 *   "/some/topic"               → { topic: "/some/topic",         fieldPath: "" }
 */
export function parseMessagePath(
  messagePath: string,
): { topic: string; fieldPath: string } | undefined {
  if (!messagePath || !messagePath.startsWith("/")) return undefined;

  const dotIdx = messagePath.indexOf(".");
  const bracketIdx = messagePath.indexOf("[");

  let sepIdx = -1;
  if (dotIdx !== -1 && bracketIdx !== -1) sepIdx = Math.min(dotIdx, bracketIdx);
  else if (dotIdx !== -1) sepIdx = dotIdx;
  else if (bracketIdx !== -1) sepIdx = bracketIdx;

  if (sepIdx === -1) return { topic: messagePath, fieldPath: "" };

  const topic = messagePath.slice(0, sepIdx);
  const raw = messagePath.slice(sepIdx);
  const fieldPath = raw.startsWith(".") ? raw.slice(1) : raw;
  return { topic, fieldPath };
}

/**
 * Walk a field path through a message object and return the numeric value, or undefined.
 * Supports dot notation and array indexing:
 *   "data", "sensors.value", "[0].data", "arr[2].x"
 */
export function getValueAtPath(obj: unknown, fieldPath: string): number | undefined {
  if (!fieldPath) {
    if (typeof obj === "number" && isFinite(obj)) return obj;
    return undefined;
  }

  const tokens = tokenizePath(fieldPath);
  let current: unknown = obj;
  for (const token of tokens) {
    if (current == null || typeof current !== "object") return undefined;
    current = (current as Record<string | number, unknown>)[token];
  }

  // typeof NaN === "number", so explicitly reject NaN and Infinity
  if (typeof current !== "number" || !isFinite(current)) return undefined;
  return current;
}

function tokenizePath(path: string): (string | number)[] {
  const tokens: (string | number)[] = [];
  let remaining = path;

  while (remaining.length > 0) {
    if (remaining.startsWith("[")) {
      const end = remaining.indexOf("]");
      if (end === -1) break; // malformed
      tokens.push(parseInt(remaining.slice(1, end), 10));
      remaining = remaining.slice(end + 1);
      if (remaining.startsWith(".")) remaining = remaining.slice(1);
    } else {
      const dotIdx = remaining.indexOf(".");
      const bracketIdx = remaining.indexOf("[");
      if (dotIdx === -1 && bracketIdx === -1) {
        tokens.push(remaining);
        remaining = "";
      } else if (dotIdx !== -1 && (bracketIdx === -1 || dotIdx < bracketIdx)) {
        tokens.push(remaining.slice(0, dotIdx));
        remaining = remaining.slice(dotIdx + 1);
      } else {
        tokens.push(remaining.slice(0, bracketIdx));
        remaining = remaining.slice(bracketIdx);
      }
    }
  }

  return tokens;
}

/**
 * Type for a quaternion with x, y, z, w components
 */
export type Quaternion = {
  x: number;
  y: number;
  z: number;
  w: number;
};

/**
 * Extract a quaternion object from a message using a base path.
 * Assumes x, y, z, w are sub-fields of the base path.
 * Example: if basePath is "orientation", expects fields like:
 *   "orientation.x", "orientation.y", "orientation.z", "orientation.w"
 */
export function getQuaternionAtPath(obj: unknown, basePath: string): Quaternion | undefined {
  const x = getValueAtPath(obj, basePath ? `${basePath}.x` : "x");
  const y = getValueAtPath(obj, basePath ? `${basePath}.y` : "y");
  const z = getValueAtPath(obj, basePath ? `${basePath}.z` : "z");
  const w = getValueAtPath(obj, basePath ? `${basePath}.w` : "w");

  if (x === undefined || y === undefined || z === undefined || w === undefined) {
    return undefined;
  }

  return { x, y, z, w };
}

/**
 * Convert a quaternion to Euler angles (in radians).
 * Uses the conversion formulas for quaternion to Euler angles.
 * Returns { roll, pitch, yaw } in radians.
 */
export function quaternionToEuler(q: Quaternion): {
  roll: number;
  pitch: number;
  yaw: number;
} {
  const { x, y, z, w } = q;

  // Roll (x-axis rotation)
  const sinr_cosp = 2 * (w * x + y * z);
  const cosr_cosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinr_cosp, cosr_cosp);

  // Pitch (y-axis rotation)
  const sinp = 2 * (w * y - z * x);
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);

  // Yaw (z-axis rotation) - this is heading
  const siny_cosp = 2 * (w * z + x * y);
  const cosy_cosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(siny_cosp, cosy_cosp);

  return { roll, pitch, yaw };
}

/**
 * Convert quaternion yaw (heading) to degrees.
 * Returns a heading in [0, 360) range.
 */
export function quaternionToHeadingDegrees(q: Quaternion): number {
  const euler = quaternionToEuler(q);
  let headingDeg = (euler.yaw * 180) / Math.PI;
  // Normalize to [0, 360)
  headingDeg = ((headingDeg % 360) + 360) % 360;
  return headingDeg;
}

/**
 * Convert quaternion roll to degrees.
 * Returns a roll in [-180, 180] range (but normalized values typically in [-90, 90] for validity).
 */
export function quaternionToRollDegrees(q: Quaternion): number {
  const euler = quaternionToEuler(q);
  const rollDeg = (euler.roll * 180) / Math.PI;
  return rollDeg;
}
