export type LatLon = { lat: number; lon: number };

export type Mission = {
  /** Safe-area polygon vertices (open ring, no repeated first point). */
  safeArea: LatLon[];
  /** True once the operator finished drawing the polygon (>= 3 vertices). */
  safeClosed: boolean;
  /** Mission start; the path is anchored here. */
  start: LatLon | null;
  /** Waypoints after the start point, in order. */
  path: LatLon[];
  /** Close the loop: last waypoint connects back to the start. */
  closed: boolean;
};

export const EMPTY_MISSION: Mission = {
  safeArea: [],
  safeClosed: false,
  start: null,
  path: [],
  closed: false,
};

/** A named mission stored in the layout. */
export type SavedMission = {
  id: string;
  name: string;
  mission: Mission;
  /** ms since epoch */
  savedAt: number;
};

export type Tool = "pan" | "safe" | "start" | "path";

export type Basemap = "osm" | "satellite" | "custom";

export type MapView = { lat: number; lon: number; zoom: number };

export type Config = {
  mission: Mission;
  /** Name of the mission currently being edited ("" = untitled). */
  missionName: string;
  /** Saved mission the current one was loaded from / saved to, if any. */
  activeMissionId: string | null;
  missions: SavedMission[];
  basemap: Basemap;
  customTileUrl: string;
  vehicleTopic: string;
  /** "all" keeps every trail point (thinned by vehicleMinSpacing), "last" keeps vehicleHistory points. */
  vehicleHistoryMode: "all" | "last";
  vehicleHistory: number;
  /** New trail points closer than this (m) to the previous one are skipped. */
  vehicleMinSpacing: number;
  vehicleColor: string;
  followVehicle: boolean;
  gnssTopic: string;
  showGnss: boolean;
  view: MapView | null;
};

export const DEFAULT_CONFIG: Config = {
  mission: EMPTY_MISSION,
  missionName: "",
  activeMissionId: null,
  missions: [],
  basemap: "osm",
  customTileUrl: "",
  vehicleTopic: "/gps/filtered/global",
  vehicleHistoryMode: "all",
  vehicleHistory: 500,
  vehicleMinSpacing: 0.2,
  vehicleColor: "#f5a524",
  followVehicle: false,
  gnssTopic: "/fix",
  showGnss: true,
  view: null,
};

/** A decoded position fix from NavSatFix or foxglove.LocationFix. */
export type Fix = {
  lat: number;
  lon: number;
  /** NavSatFix status.status (-1 no fix, 0 fix, 1 SBAS, 2 GBAS). Undefined for LocationFix. */
  status?: number;
  /** Horizontal 1-sigma accuracy in metres from the covariance diagonal, if known. */
  hAcc?: number;
  /** Receive time (ms since epoch, wall clock) for staleness display. */
  receivedAt: number;
};

/** Message schemas the panel can decode positions from. */
export const POSITION_SCHEMAS = [
  "sensor_msgs/msg/NavSatFix",
  "sensor_msgs/NavSatFix",
  "foxglove.LocationFix",
  "foxglove_msgs/msg/LocationFix",
  "foxglove_msgs/LocationFix",
];
