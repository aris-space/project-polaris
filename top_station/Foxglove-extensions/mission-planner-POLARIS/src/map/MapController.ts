import * as L from "leaflet";
import "leaflet/dist/leaflet.css";

import {
  gnssIcon,
  midpointIcon,
  segmentLabelIcon,
  startIcon,
  vehicleIcon,
  vertexIcon,
  waypointIcon,
} from "./icons";
import { analyzeMission, bearingDeg, distanceM, formatDistance, routePoints } from "../geo";
import { Basemap, Fix, LatLon, MapView, Mission, Tool } from "../types";

export type HandleKind = "safe" | "path" | "start";

export type MapCallbacks = {
  onMapClick: (p: LatLon, clickCount: number) => void;
  onMapDblClick: () => void;
  onFirstSafeVertexClick: () => void;
  onHandleMoved: (kind: HandleKind, index: number, p: LatLon) => void;
  onHandleDelete: (kind: "safe" | "path", index: number) => void;
  onSafeInsert: (index: number, p: LatLon) => void;
  onCursor: (p: LatLon | undefined) => void;
  onViewChange: (v: MapView) => void;
  onUserPan: () => void;
};

const COLORS = {
  safe: "#ff4d4f",
  route: "#22d3ee",
  routeOutside: "#ff4d4f",
  casing: "#0b1220",
};

const WORLD_RING: L.LatLngExpression[] = [
  [-89.9, -720],
  [89.9, -720],
  [89.9, 720],
  [-89.9, 720],
];

const TILES: Record<Exclude<Basemap, "custom">, { url: string; attribution: string; maxNative: number }> =
  {
    osm: {
      url: "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
      attribution: "© OpenStreetMap contributors",
      maxNative: 19,
    },
    satellite: {
      url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
      attribution: "Imagery © Esri, Maxar, Earthstar Geographics",
      maxNative: 19,
    },
  };

const toLL = (p: LatLon): L.LatLng => L.latLng(p.lat, p.lon);
const fromLL = (p: L.LatLng): LatLon => ({ lat: p.lat, lon: p.lng });

/** Minimum on-screen segment length (px) before a distance label is shown. */
const LABEL_MIN_PX = 70;

/**
 * Imperative Leaflet wrapper. React owns the mission state; this class only
 * renders it and reports user gestures back through MapCallbacks.
 */
export class MapController {
  #map: L.Map;
  #container: HTMLElement;
  #cb: MapCallbacks;
  #resizeObserver: ResizeObserver;

  #tiles?: L.TileLayer;
  #tileKey = "";

  #maskLayer = L.layerGroup();
  #shapesLayer = L.layerGroup();
  #handlesLayer = L.layerGroup();
  #trailLayer = L.layerGroup();
  #rubber: L.Polyline;
  #rubberClose: L.Polyline;
  #vehicleMarker?: L.Marker;
  #vehicleKey = "";
  #gnssMarker?: L.Marker;

  #mission?: Mission;
  #tool: Tool = "pan";
  #rubberAnchors: { from?: LatLon; closeTo?: LatLon } = {};
  #dragging = false;
  #suppressViewEvents = false;

  public constructor(container: HTMLElement, callbacks: MapCallbacks, view: MapView | null) {
    this.#container = container;
    this.#cb = callbacks;
    this.#map = L.map(container, {
      zoomControl: false,
      attributionControl: true,
      maxZoom: 22,
      worldCopyJump: true,
      boxZoom: false,
    });
    this.#map.attributionControl.setPrefix(false);
    L.control.zoom({ position: "topright" }).addTo(this.#map);
    L.control.scale({ position: "bottomright", imperial: false, maxWidth: 140 }).addTo(this.#map);

    const panes: Array<[string, number]> = [
      ["mp-mask", 405],
      ["mp-trail", 415],
      ["mp-shapes", 420],
      ["mp-labels", 590],
      ["mp-vehicle", 640],
    ];
    for (const [name, z] of panes) {
      const pane = this.#map.createPane(name);
      pane.style.zIndex = String(z);
    }
    this.#map.getPane("mp-labels")!.style.pointerEvents = "none";

    this.#maskLayer.addTo(this.#map);
    this.#trailLayer.addTo(this.#map);
    this.#shapesLayer.addTo(this.#map);
    this.#handlesLayer.addTo(this.#map);

    const rubberStyle: L.PolylineOptions = {
      pane: "mp-shapes",
      interactive: false,
      weight: 2,
      dashArray: "6 6",
      opacity: 0.9,
    };
    this.#rubber = L.polyline([], rubberStyle).addTo(this.#map);
    this.#rubberClose = L.polyline([], { ...rubberStyle, opacity: 0.35 }).addTo(this.#map);

    if (view) {
      this.#map.setView([view.lat, view.lon], view.zoom);
    } else {
      // Switzerland overview until we have mission or vehicle data.
      this.#map.setView([46.8, 8.2], 8);
    }

    this.#map.on("click", (e: L.LeafletMouseEvent) => {
      this.#cb.onMapClick(fromLL(e.latlng), e.originalEvent.detail);
    });
    this.#map.on("dblclick", () => {
      if (this.#tool !== "pan") {
        this.#cb.onMapDblClick();
      }
    });
    this.#map.on("mousemove", (e: L.LeafletMouseEvent) => {
      this.#cb.onCursor(fromLL(e.latlng));
      this.#updateRubber(e.latlng);
    });
    this.#map.on("mouseout", () => {
      this.#cb.onCursor(undefined);
      this.#rubber.setLatLngs([]);
      this.#rubberClose.setLatLngs([]);
    });
    this.#map.on("dragstart", () => {
      this.#cb.onUserPan();
    });
    this.#map.on("moveend", () => {
      if (this.#suppressViewEvents) {
        return;
      }
      const c = this.#map.getCenter();
      this.#cb.onViewChange({ lat: c.lat, lon: c.lng, zoom: this.#map.getZoom() });
    });
    this.#map.on("zoomend", () => {
      if (this.#mission) {
        this.#renderHandles(this.#mission);
      }
    });

    this.#resizeObserver = new ResizeObserver(() => {
      this.#map.invalidateSize({ animate: false });
    });
    this.#resizeObserver.observe(container);
  }

  public destroy(): void {
    this.#resizeObserver.disconnect();
    this.#map.remove();
  }

  public setCallbacks(cb: MapCallbacks): void {
    this.#cb = cb;
  }

  // ---------------------------------------------------------------- basemap

  public setBasemap(kind: Basemap, customUrl: string, opts: { dark: boolean }): void {
    const { dark } = opts;
    const url = kind === "custom" ? customUrl.trim() : TILES[kind].url;
    const key = `${kind}|${url}`;
    this.#container.classList.toggle("mp-tiles-dark", dark && kind === "osm");
    if (key === this.#tileKey) {
      return;
    }
    this.#tileKey = key;
    this.#tiles?.remove();
    this.#tiles = undefined;
    if (url.length === 0) {
      return;
    }
    const meta = kind === "custom" ? { attribution: "Custom tiles", maxNative: 19 } : TILES[kind];
    this.#tiles = L.tileLayer(url, {
      attribution: meta.attribution,
      maxNativeZoom: meta.maxNative,
      maxZoom: 22,
      crossOrigin: true,
    }).addTo(this.#map);
  }

  // ------------------------------------------------------------------- tool

  public setTool(tool: Tool): void {
    this.#tool = tool;
    this.#container.classList.toggle("mp-drawing", tool !== "pan");
    if (tool === "pan") {
      this.#map.doubleClickZoom.enable();
    } else {
      this.#map.doubleClickZoom.disable();
    }
    if (this.#mission) {
      this.#renderHandles(this.#mission);
    }
  }

  /** Where the dashed preview line starts (and optionally where it would close to). */
  public setRubberAnchors(from: LatLon | undefined, closeTo: LatLon | undefined): void {
    this.#rubberAnchors = { from, closeTo };
    if (!from) {
      this.#rubber.setLatLngs([]);
      this.#rubberClose.setLatLngs([]);
    }
  }

  #updateRubber(cursor: L.LatLng): void {
    const { from, closeTo } = this.#rubberAnchors;
    if (!from || this.#dragging) {
      return;
    }
    const color = this.#tool === "safe" ? COLORS.safe : COLORS.route;
    this.#rubber.setStyle({ color });
    this.#rubberClose.setStyle({ color });
    this.#rubber.setLatLngs([toLL(from), cursor]);
    this.#rubberClose.setLatLngs(closeTo ? [cursor, toLL(closeTo)] : []);
  }

  // ---------------------------------------------------------------- mission

  public setMission(m: Mission): void {
    this.#mission = m;
    this.#renderShapes(m);
    this.#renderHandles(m);
  }

  #renderShapes(m: Mission): void {
    const a = analyzeMission(m);
    this.#maskLayer.clearLayers();
    this.#shapesLayer.clearLayers();

    // Safe area: red translucent mask outside, solid outline.
    if (m.safeArea.length > 0) {
      const ring = m.safeArea.map(toLL);
      if (m.safeClosed && m.safeArea.length >= 3) {
        L.polygon([WORLD_RING, ring], {
          pane: "mp-mask",
          interactive: false,
          stroke: false,
          fillColor: COLORS.safe,
          fillOpacity: 0.3,
          fillRule: "evenodd",
        }).addTo(this.#maskLayer);
        L.polygon(ring, {
          pane: "mp-shapes",
          interactive: false,
          color: COLORS.safe,
          weight: 2.5,
          fill: false,
        }).addTo(this.#shapesLayer);
      } else {
        L.polyline(ring, {
          pane: "mp-shapes",
          interactive: false,
          color: COLORS.safe,
          weight: 2.5,
        }).addTo(this.#shapesLayer);
      }
    }

    // Route: start -> wp1 -> ... (-> start when closed). Dark casing for contrast on imagery.
    const route = routePoints(m);
    for (let i = 0; i + 1 < route.length; i++) {
      const seg = [toLL(route[i]!), toLL(route[i + 1]!)];
      const outside = a.segmentOutside[i] === true;
      const closing = m.closed && i === route.length - 2;
      L.polyline(seg, {
        pane: "mp-shapes",
        interactive: false,
        color: COLORS.casing,
        opacity: 0.45,
        weight: 7,
        lineCap: "round",
      }).addTo(this.#shapesLayer);
      L.polyline(seg, {
        pane: "mp-shapes",
        interactive: false,
        color: outside ? COLORS.routeOutside : COLORS.route,
        weight: 3.5,
        lineCap: "round",
        dashArray: closing ? "10 8" : undefined,
      }).addTo(this.#shapesLayer);
    }
  }

  #renderHandles(m: Mission): void {
    this.#handlesLayer.clearLayers();
    const a = analyzeMission(m);
    const drawingSafe = this.#tool === "safe" && !m.safeClosed;

    // --- safe-area vertices
    m.safeArea.forEach((p, i) => {
      const closable = drawingSafe && i === 0 && m.safeArea.length >= 3;
      const marker = this.#makeHandle(p, vertexIcon({ first: i === 0 && drawingSafe, closable }), "safe", i);
      if (closable) {
        marker.on("click", () => {
          this.#cb.onFirstSafeVertexClick();
        });
      }
    });

    // --- midpoint "+" insert handles (only while editing a finished safe area)
    if (m.safeClosed && this.#tool === "safe") {
      const n = m.safeArea.length;
      for (let i = 0; i < n; i++) {
        const p1 = m.safeArea[i]!;
        const p2 = m.safeArea[(i + 1) % n]!;
        if (!this.#longEnoughOnScreen(p1, p2, 40)) {
          continue;
        }
        const mid = { lat: (p1.lat + p2.lat) / 2, lon: (p1.lon + p2.lon) / 2 };
        const marker = L.marker(toLL(mid), {
          icon: midpointIcon(),
          draggable: true,
          keyboard: false,
          bubblingMouseEvents: false,
        }).addTo(this.#handlesLayer);
        marker.on("dragstart", () => {
          this.#setDragging("on");
        });
        marker.on("drag", () => {
          const live = { ...m, safeArea: [...m.safeArea] };
          live.safeArea.splice(i + 1, 0, fromLL(marker.getLatLng()));
          this.#renderShapes(live);
        });
        marker.on("dragend", () => {
          this.#setDragging("off");
          this.#cb.onSafeInsert(i + 1, fromLL(marker.getLatLng()));
        });
        marker.on("click", () => {
          this.#cb.onSafeInsert(i + 1, mid);
        });
      }
    }

    // --- route segment distance labels
    const route = routePoints(m);
    for (let i = 0; i + 1 < route.length; i++) {
      const p1 = route[i]!;
      const p2 = route[i + 1]!;
      if (!this.#longEnoughOnScreen(p1, p2, LABEL_MIN_PX)) {
        continue;
      }
      const mid = { lat: (p1.lat + p2.lat) / 2, lon: (p1.lon + p2.lon) / 2 };
      L.marker(toLL(mid), {
        icon: segmentLabelIcon(formatDistance(distanceM(p1, p2)), { outside: a.segmentOutside[i] === true }),
        interactive: false,
        keyboard: false,
        pane: "mp-labels",
      }).addTo(this.#handlesLayer);
    }

    // --- waypoints
    m.path.forEach((p, i) => {
      this.#makeHandle(p, waypointIcon(i + 1, { outside: a.waypointOutside[i] === true }), "path", i);
    });

    // --- start (on top)
    if (m.start) {
      const marker = this.#makeHandle(m.start, startIcon({ outside: a.startOutside }), "start", 0);
      marker.setZIndexOffset(1000);
    }
  }

  #makeHandle(p: LatLon, icon: L.DivIcon, kind: HandleKind, index: number): L.Marker {
    const marker = L.marker(toLL(p), {
      icon,
      draggable: true,
      keyboard: false,
      bubblingMouseEvents: false,
      autoPan: true,
    }).addTo(this.#handlesLayer);

    marker.on("dragstart", () => {
          this.#setDragging("on");
        });
    marker.on("drag", () => {
      if (!this.#mission) {
        return;
      }
      this.#renderShapes(this.#withMoved(this.#mission, kind, index, fromLL(marker.getLatLng())));
    });
    marker.on("dragend", () => {
      this.#setDragging("off");
      this.#cb.onHandleMoved(kind, index, fromLL(marker.getLatLng()));
    });
    if (kind !== "start") {
      marker.on("contextmenu", (e: L.LeafletMouseEvent) => {
        L.DomEvent.preventDefault(e.originalEvent);
        L.DomEvent.stopPropagation(e.originalEvent);
        this.#cb.onHandleDelete(kind, index);
      });
    }
    return marker;
  }

  #withMoved(m: Mission, kind: HandleKind, index: number, p: LatLon): Mission {
    switch (kind) {
      case "start":
        return { ...m, start: p };
      case "safe": {
        const safeArea = [...m.safeArea];
        safeArea[index] = p;
        return { ...m, safeArea };
      }
      case "path": {
        const path = [...m.path];
        path[index] = p;
        return { ...m, path };
      }
    }
  }

  #setDragging(state: "on" | "off"): void {
    const on = state === "on";
    this.#dragging = on;
    this.#container.classList.toggle("mp-handle-dragging", on);
    if (on) {
      this.#rubber.setLatLngs([]);
      this.#rubberClose.setLatLngs([]);
    }
  }

  #longEnoughOnScreen(p1: LatLon, p2: LatLon, px: number): boolean {
    const a = this.#map.latLngToLayerPoint(toLL(p1));
    const b = this.#map.latLngToLayerPoint(toLL(p2));
    return a.distanceTo(b) >= px;
  }

  // ---------------------------------------------------------------- vehicle

  public setVehicle(track: LatLon[], opts: { color: string; follow: boolean }): void {
    const { color, follow } = opts;
    this.#trailLayer.clearLayers();
    const last = track[track.length - 1];
    if (!last) {
      this.#vehicleMarker?.remove();
      this.#vehicleMarker = undefined;
      this.#vehicleKey = "";
      return;
    }
    if (track.length >= 2) {
      L.polyline(track.map(toLL), {
        pane: "mp-trail",
        interactive: false,
        color,
        weight: 2.5,
        opacity: 0.85,
      }).addTo(this.#trailLayer);
    }

    // Course over ground from the most recent point at least 0.3 m away.
    let heading: number | undefined;
    for (let i = track.length - 2; i >= 0; i--) {
      const prev = track[i]!;
      if (distanceM(prev, last) >= 0.3) {
        heading = bearingDeg(prev, last);
        break;
      }
    }
    const key = `${color}|${heading == undefined ? "dot" : heading.toFixed(0)}`;
    if (!this.#vehicleMarker) {
      this.#vehicleMarker = L.marker(toLL(last), {
        icon: vehicleIcon(color, heading),
        pane: "mp-vehicle",
        interactive: false,
        keyboard: false,
      }).addTo(this.#map);
    } else {
      this.#vehicleMarker.setLatLng(toLL(last));
      if (key !== this.#vehicleKey) {
        this.#vehicleMarker.setIcon(vehicleIcon(color, heading));
      }
    }
    this.#vehicleKey = key;

    if (follow && !this.#dragging) {
      this.#panQuietly(last);
    }
  }

  public setGnss(fix: Fix | undefined): void {
    if (!fix) {
      this.#gnssMarker?.remove();
      this.#gnssMarker = undefined;
      return;
    }
    if (!this.#gnssMarker) {
      this.#gnssMarker = L.marker(toLL(fix), {
        icon: gnssIcon(),
        pane: "mp-vehicle",
        interactive: false,
        keyboard: false,
      }).addTo(this.#map);
    } else {
      this.#gnssMarker.setLatLng(toLL(fix));
    }
  }

  // ------------------------------------------------------------------- view

  #panQuietly(p: LatLon): void {
    const target = toLL(p);
    if (this.#map.getCenter().distanceTo(target) < 0.05) {
      return;
    }
    this.#suppressViewEvents = true;
    this.#map.panTo(target, { animate: false });
    this.#suppressViewEvents = false;
  }

  public centerOn(p: LatLon, minZoom = 17): void {
    this.#map.setView(toLL(p), Math.max(this.#map.getZoom(), minZoom), { animate: true });
  }

  /** Fit the map to everything in the mission. Returns false if there was nothing to fit. */
  public fitMission(m: Mission, extra: LatLon[] = []): boolean {
    const pts = [...m.safeArea, ...(m.start ? [m.start] : []), ...m.path, ...extra];
    if (pts.length === 0) {
      return false;
    }
    if (pts.length === 1) {
      this.centerOn(pts[0]!);
      return true;
    }
    this.#map.fitBounds(L.latLngBounds(pts.map(toLL)), { padding: [60, 60], maxZoom: 19 });
    return true;
  }
}
