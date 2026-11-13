/**
 * Mission files: JSON export/import so missions can live outside the Foxglove
 * layout (e.g. versioned in git next to the autonomy mission CSVs).
 *
 * Single mission:
 *   { "format": "polaris-mission", "version": 1, "name": "...", "exportedAt": "...", "mission": {...} }
 * Library (several missions):
 *   { "format": "polaris-mission-library", "version": 1, "exportedAt": "...",
 *     "missions": [ { "name": "...", "savedAt": "...", "mission": {...} }, ... ] }
 *
 * `mission` = { type: "waypoints" | "survey", safeArea: [{lat, lon}], safeClosed,
 *               start: {lat, lon} | null, path: [{lat, lon}], closed,
 *               survey: [{lat, lon}], surveyClosed }
 * Coordinates are WGS84 degrees.
 */
import { LatLon, Mission, SavedMission } from "./types";

export const MISSION_FORMAT = "polaris-mission";
export const LIBRARY_FORMAT = "polaris-mission-library";
export const FILE_VERSION = 1;

export function isLatLon(v: unknown): v is LatLon {
  const p = v as LatLon | undefined;
  return (
    p != undefined &&
    typeof p === "object" &&
    Number.isFinite(p.lat) &&
    Number.isFinite(p.lon) &&
    Math.abs(p.lat) <= 90 &&
    Math.abs(p.lon) <= 180
  );
}

const cleanPoint = (p: LatLon): LatLon => ({ lat: p.lat, lon: p.lon });

/** Coerce anything mission-shaped into a valid Mission (drops invalid points). */
export function normalizeMission(raw: unknown): Mission {
  const sm = (raw ?? {}) as Partial<Mission>;
  const mission: Mission = {
    type: sm.type === "survey" ? "survey" : "waypoints",
    safeArea: Array.isArray(sm.safeArea) ? sm.safeArea.filter(isLatLon).map(cleanPoint) : [],
    safeClosed: sm.safeClosed === true,
    start: isLatLon(sm.start) ? cleanPoint(sm.start) : null,
    path: Array.isArray(sm.path) ? sm.path.filter(isLatLon).map(cleanPoint) : [],
    closed: sm.closed === true,
    survey: Array.isArray(sm.survey) ? sm.survey.filter(isLatLon).map(cleanPoint) : [],
    surveyClosed: sm.surveyClosed === true,
  };
  if (mission.safeArea.length < 3) {
    mission.safeClosed = false;
  }
  if (mission.survey.length < 3) {
    mission.surveyClosed = false;
  }
  if (mission.path.length < 2) {
    mission.closed = false;
  }
  return mission;
}

export function missionToFile(name: string, mission: Mission): object {
  return {
    format: MISSION_FORMAT,
    version: FILE_VERSION,
    name,
    exportedAt: new Date().toISOString(),
    mission,
  };
}

export function libraryToFile(missions: SavedMission[]): object {
  return {
    format: LIBRARY_FORMAT,
    version: FILE_VERSION,
    exportedAt: new Date().toISOString(),
    missions: missions.map((m) => ({
      name: m.name,
      savedAt: new Date(m.savedAt).toISOString(),
      mission: m.mission,
    })),
  };
}

export type ImportedMission = { name: string; mission: Mission };

function looksLikeMission(o: unknown): boolean {
  if (o == undefined || typeof o !== "object") {
    return false;
  }
  const m = o as Record<string, unknown>;
  return Array.isArray(m.safeArea) || Array.isArray(m.path) || Array.isArray(m.survey) || "start" in m;
}

/** Parse a mission or library file. Throws an Error with a user-facing message. */
export function parseMissionFile(text: string, fallbackName: string): ImportedMission[] {
  let data: unknown;
  try {
    data = JSON.parse(text);
  } catch {
    throw new Error("not valid JSON");
  }
  if (data == undefined || typeof data !== "object") {
    throw new Error("not a mission file");
  }
  const d = data as Record<string, unknown>;
  const version = typeof d.version === "number" ? d.version : FILE_VERSION;
  if (version > FILE_VERSION) {
    throw new Error(`file version ${version} is newer than this panel supports (${FILE_VERSION})`);
  }
  const nameOf = (v: unknown, fb: string) => (typeof v === "string" && v.trim() ? v.trim() : fb);

  if (d.format === LIBRARY_FORMAT || Array.isArray(d.missions)) {
    const list = Array.isArray(d.missions) ? d.missions : [];
    const out = list
      .filter((e): e is Record<string, unknown> => e != undefined && typeof e === "object")
      .filter((e) => looksLikeMission(e.mission))
      .map((e, i) => ({ name: nameOf(e.name, `${fallbackName} ${i + 1}`), mission: normalizeMission(e.mission) }));
    if (out.length === 0) {
      throw new Error("library contains no missions");
    }
    return out;
  }
  if (d.format === MISSION_FORMAT || looksLikeMission(d.mission)) {
    if (!looksLikeMission(d.mission)) {
      throw new Error("mission data missing");
    }
    return [{ name: nameOf(d.name, fallbackName), mission: normalizeMission(d.mission) }];
  }
  if (looksLikeMission(d)) {
    // bare mission object
    return [{ name: fallbackName, mission: normalizeMission(d) }];
  }
  throw new Error("not a POLARIS mission file");
}

/** "Lake test A" -> "lake-test-a" */
export function slugify(name: string): string {
  const s = name
    .toLowerCase()
    .normalize("NFKD")
    .replace(/[̀-ͯ]/g, "")
    .replace(/[^a-z0-9]+/g, "-")
    .replace(/^-+|-+$/g, "");
  return s || "mission";
}

/** Make `name` unique among `taken` by appending " (2)", " (3)", … */
export function uniqueName(name: string, taken: Iterable<string>): string {
  const set = new Set(taken);
  if (!set.has(name)) {
    return name;
  }
  for (let i = 2; ; i++) {
    const candidate = `${name} (${i})`;
    if (!set.has(candidate)) {
      return candidate;
    }
  }
}

// ---------------------------------------------------------------------------- file I/O

type SaveFilePicker = (opts: {
  suggestedName: string;
  types: Array<{ description: string; accept: Record<string, string[]> }>;
}) => Promise<{ createWritable: () => Promise<{ write: (d: string) => Promise<void>; close: () => Promise<void> }> }>;

/**
 * Save text to a file. Uses the native "Save as" dialog when available
 * (Foxglove desktop / Chromium), otherwise falls back to a download.
 * Must be called directly from a user gesture (click).
 */
export async function saveTextFile(filename: string, text: string): Promise<"saved" | "cancelled"> {
  const picker = (window as unknown as { showSaveFilePicker?: SaveFilePicker }).showSaveFilePicker;
  if (picker) {
    try {
      const handle = await picker({
        suggestedName: filename,
        types: [{ description: "POLARIS mission (JSON)", accept: { "application/json": [".json"] } }],
      });
      const w = await handle.createWritable();
      await w.write(text);
      await w.close();
      return "saved";
    } catch (err) {
      if (err instanceof DOMException && err.name === "AbortError") {
        return "cancelled";
      }
      // Picker not permitted in this context: fall back to a download.
    }
  }
  const url = URL.createObjectURL(new Blob([text], { type: "application/json" }));
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  setTimeout(() => {
    URL.revokeObjectURL(url);
  }, 2000);
  return "saved";
}
