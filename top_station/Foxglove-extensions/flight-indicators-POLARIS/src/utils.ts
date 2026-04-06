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
