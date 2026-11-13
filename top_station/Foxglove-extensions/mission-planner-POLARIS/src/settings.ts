import { SettingsTreeNodes, Topic } from "@foxglove/extension";

import { analyzeMission, formatArea, formatDistance, formatLatLon } from "./geo";
import { Config, POSITION_SCHEMAS } from "./types";

/** Topics whose schema we can decode a lat/lon from (NavSatFix / LocationFix). */
export function positionTopics(topics: readonly Topic[]): string[] {
  return topics
    .filter((t) => POSITION_SCHEMAS.includes(t.schemaName))
    .map((t) => t.name)
    .sort();
}

export function buildSettingsNodes(config: Config, topics: readonly Topic[]): SettingsTreeNodes {
  const items = positionTopics(topics);
  const topicWarning = (topic: string): string | undefined => {
    if (topic.length === 0 || topics.length === 0) {
      return undefined;
    }
    const t = topics.find((x) => x.name === topic);
    if (!t) {
      return "Topic not available (yet)";
    }
    if (!POSITION_SCHEMAS.includes(t.schemaName)) {
      return `Unsupported type ${t.schemaName} — use NavSatFix or LocationFix`;
    }
    return undefined;
  };

  const m = config.mission;
  const a = analyzeMission(m);

  return {
    map: {
      label: "Map",
      icon: "Map",
      fields: {
        basemap: {
          label: "Base map",
          input: "select",
          value: config.basemap,
          options: [
            { label: "Street (OpenStreetMap)", value: "osm" },
            { label: "Satellite (Esri)", value: "satellite" },
            { label: "Custom tile URL", value: "custom" },
          ],
        },
        customTileUrl:
          config.basemap === "custom"
            ? {
                label: "Tile URL",
                input: "string",
                value: config.customTileUrl,
                placeholder: "http://host/tiles/{z}/{x}/{y}.png",
                help: "XYZ tile template, e.g. a local tile server for offline use in the field",
              }
            : undefined,
      },
    },
    vehicle: {
      label: "Vehicle position",
      icon: "Points",
      fields: {
        vehicleTopic: {
          label: "Topic",
          input: "autocomplete",
          value: config.vehicleTopic,
          items,
          placeholder: "/gps/filtered/global",
          help: "sensor_msgs/NavSatFix or foxglove.LocationFix",
          warning: topicWarning(config.vehicleTopic),
        },
        vehicleHistory: {
          label: "History (points)",
          input: "number",
          value: config.vehicleHistory,
          min: 0,
          max: 100000,
          step: 100,
          help: "Number of past positions kept for the trail (0 = marker only)",
        },
        vehicleColor: { label: "Color", input: "rgb", value: config.vehicleColor },
        followVehicle: { label: "Follow vehicle", input: "boolean", value: config.followVehicle },
      },
    },
    gnss: {
      label: "GNSS start point",
      icon: "World",
      fields: {
        gnssTopic: {
          label: "GNSS fix topic",
          input: "autocomplete",
          value: config.gnssTopic,
          items,
          placeholder: "/fix",
          help: "Used by “Start from GNSS”",
          warning: topicWarning(config.gnssTopic),
        },
        showGnss: { label: "Show live GNSS fix", input: "boolean", value: config.showGnss },
      },
    },
    mission: {
      label: "Mission",
      icon: "Timeline",
      actions: [{ type: "action", id: "clear-mission", label: "Clear mission", icon: "Delete" }],
      fields: {
        safe: {
          label: "Safe area",
          input: "string",
          readonly: true,
          value: m.safeClosed
            ? `${m.safeArea.length} vertices · ${formatArea(a.safeAreaM2)}`
            : m.safeArea.length > 0
              ? `drawing (${m.safeArea.length} vertices)`
              : "not set",
        },
        start: {
          label: "Start",
          input: "string",
          readonly: true,
          value: m.start ? formatLatLon(m.start) : "not set",
        },
        path: {
          label: "Path",
          input: "string",
          readonly: true,
          value: `${m.path.length} waypoints · ${formatDistance(a.routeLengthM)}${m.closed ? " · loop" : ""}`,
        },
      },
    },
  };
}
