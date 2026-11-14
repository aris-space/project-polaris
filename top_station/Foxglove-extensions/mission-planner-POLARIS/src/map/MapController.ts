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
import { Basemap, Fix, LatLon, MapView, Mission, PolygonKind, Tool } from "../types";

export type HandleKind = "safe" | "path" | "start" | "survey";

export type Selection = { kind: HandleKind; index: number } | undefined;

export type MapCallbacks = {
  onMapClick: (p: LatLon, clickCount: number) => void;
  onMapDblClick: () => void;
  onFirstVertexClick: (kind: PolygonKind) => void;
  onHandleMoved: (kind: HandleKind, index: number, p: LatLon) => void;
  onHandleDelete: (kind: Exclude<HandleKind, "start">, index: number) => void;
  onHandleClick: (kind: HandleKind, index: number) => void;
  onSnapToStart: () => void;
  onPolygonInsert: (kind: PolygonKind, index: number, p: LatLon) => void;
  onCursor: (p: LatLon | undefined) => void;
  onViewChange: (v: MapView) => void;
  onUserPan: () => void;
};

const COLORS = {
  safe: "#ff4d4f",
  survey: "#c084fc",
  surveyFill: "#a855f7",
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

/** Path tool: a click within this many pixels of the start closes the loop. */
const SNAP_PX = 18;

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
  #rubber: L.Polyline;
  #rubberClose: L.Polyline;
  #rubberLabel: L.Marker;
  #vehicleMarker?: L.Marker;
  #vehicleKey = "";
  #gnssMarker?: L.Marker;
  #startMarker?: L.Marker;
  #trail?: L.Polyline;
  #trailCount = 0;
  #trailFirst?: LatLon;
  #trailColor = "";
  #heading?: number;
  #headingAnchor?: LatLon;
  #followWasOn = false;
  #selection: Selection;
  #editable = true;

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
    this.#rubberLabel = L.marker([0, 0], {
      icon: segmentLabelIcon("", { outside: false, variant: "live" }),
      interactive: false,
      keyboard: false,
      pane: "mp-labels",
      opacity: 0,
    }).addTo(this.#map);

    if (view) {
      this.#map.setView([view.lat, view.lon], view.zoom);
    } else {
      // Switzerland overview until we have mission or vehicle data.
      this.#map.setView([46.8, 8.2], 8);
    }

    this.#map.on("click", (e: L.LeafletMouseEvent) => {
      if (!this.#editable) {
        return;
      }
      if (this.#nearStartForSnap(e.latlng)) {
        if (e.originalEvent.detail <= 1) {
          this.#cb.onSnapToStart();
        }
        return;
      }
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
      this.#rubberLabel.setOpacity(0);
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
  /** Live length of the segment being drawn, at its midpoint. */
  #showRubberLabel(a: L.LatLng, b: L.LatLng | undefined): void {
    if (!b) {
      this.#rubberLabel.setOpacity(0);
      return;
    }
    const el = this.#rubberLabel.getElement()?.querySelector(".mp-seglabel");
    if (el) {
      el.textContent = formatDistance(a.distanceTo(b));
      el.classList.toggle("mp-seglabel--survey", this.#tool === "survey");
      el.classList.toggle("mp-seglabel--safe", this.#tool === "safe");
    }
    this.#rubberLabel.setLatLng(L.latLng((a.lat + b.lat) / 2, (a.lng + b.lng) / 2));
    this.#rubberLabel.setOpacity(1);
  }

  public setRubberAnchors(from: LatLon | undefined, closeTo: LatLon | undefined): void {
    this.#rubberAnchors = { from, closeTo };
    if (!from) {
      this.#rubberLabel.setOpacity(0);
      this.#rubber.setLatLngs([]);
      this.#rubberClose.setLatLngs([]);
    }
  }

  #updateRubber(cursor: L.LatLng): void {
    const { from, closeTo } = this.#rubberAnchors;
    if (!from || this.#dragging) {
      return;
    }
    const color =
      this.#tool === "safe" ? COLORS.safe : this.#tool === "survey" ? COLORS.survey : COLORS.route;
    this.#rubber.setStyle({ color });
    this.#rubberClose.setStyle({ color });
    const snap = this.#editable && this.#nearStartForSnap(cursor);
    this.#startMarker?.getElement()?.classList.toggle("mp-snap-target", snap);
    this.#container.classList.toggle("mp-snapping", snap);
    if (snap && this.#mission?.start) {
      this.#rubber.setLatLngs([toLL(from), toLL(this.#mission.start)]);
      this.#showRubberLabel(toLL(from), toLL(this.#mission.start));
      this.#rubberClose.setLatLngs([]);
      return;
    }
    this.#rubber.setLatLngs([toLL(from), cursor]);
    this.#showRubberLabel(toLL(from), cursor);
    this.#rubberClose.setLatLngs(closeTo ? [cursor, toLL(closeTo)] : []);
  }

  /** Path tool with >= 2 waypoints and the point within SNAP_PX of the start. */
  #nearStartForSnap(p: L.LatLng): boolean {
    const m = this.#mission;
    if (this.#tool !== "path" || !m?.start || m.path.length < 2) {
      return false;
    }
    const a = this.#map.latLngToContainerPoint(p);
    const b = this.#map.latLngToContainerPoint(toLL(m.start));
    return a.distanceTo(b) <= SNAP_PX;
  }

  /** Watch mode = not editable: handles can't be dragged/clicked, map clicks are ignored. */
  public setEditable(opts: { editable: boolean }): void {
    if (this.#editable === opts.editable) {
      return;
    }
    this.#editable = opts.editable;
    this.#container.classList.toggle("mp-watch", !opts.editable);
    if (this.#mission) {
      this.#renderHandles(this.#mission);
    }
  }

  public setSelection(sel: Selection): void {
    this.#selection = sel;
    if (this.#mission) {
      this.#renderHandles(this.#mission);
    }
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

    // Survey area (survey missions): violet translucent fill; red outline if it leaves the safe area.
    if (m.survey.length > 0) {
      const ring = m.survey.map(toLL);
      const color = a.surveyOutside ? COLORS.routeOutside : COLORS.survey;
      if (m.surveyClosed && m.survey.length >= 3) {
        L.polygon(ring, {
          pane: "mp-shapes",
          interactive: false,
          color,
          weight: 2.5,
          fillColor: COLORS.surveyFill,
          fillOpacity: 0.22,
          dashArray: a.surveyOutside ? "8 6" : undefined,
        }).addTo(this.#shapesLayer);
      } else {
        L.polyline(ring, {
          pane: "mp-shapes",
          interactive: false,
          color: COLORS.survey,
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
    this.#polygonHandles(m, "safe");
    this.#polygonHandles(m, "survey");

    // --- survey edge lengths (shown while drawing and afterwards)
    for (let i = 0; i < a.surveyEdgeOutside.length; i++) {
      const p1 = m.survey[i]!;
      const p2 = m.survey[(i + 1) % m.survey.length]!;
      if (!this.#longEnoughOnScreen(p1, p2, LABEL_MIN_PX)) {
        continue;
      }
      const mid = { lat: (p1.lat + p2.lat) / 2, lon: (p1.lon + p2.lon) / 2 };
      L.marker(toLL(mid), {
        icon: segmentLabelIcon(formatDistance(distanceM(p1, p2)), {
          outside: a.surveyEdgeOutside[i] === true,
          variant: "survey",
        }),
        interactive: false,
        keyboard: false,
        pane: "mp-labels",
      }).addTo(this.#handlesLayer);
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
    this.#startMarker = undefined;
    if (m.start) {
      const marker = this.#makeHandle(m.start, startIcon({ outside: a.startOutside }), "start", 0);
      marker.setZIndexOffset(1000);
      this.#startMarker = marker;
    }
  }

  /** Vertex handles (+ "+" midpoint inserts while editing) for a polygon. */
  #polygonHandles(m: Mission, kind: PolygonKind): void {
    const pts = kind === "safe" ? m.safeArea : m.survey;
    const closed = kind === "safe" ? m.safeClosed : m.surveyClosed;
    const drawing = this.#tool === kind && !closed;
    pts.forEach((p, i) => {
      const closable = drawing && i === 0 && pts.length >= 3;
      this.#makeHandle(
        p,
        vertexIcon({ first: i === 0 && drawing, closable, variant: kind }),
        kind,
        i,
      );
    });

    if (!closed || this.#tool !== kind || !this.#editable) {
      return;
    }
    const n = pts.length;
    for (let i = 0; i < n; i++) {
      const p1 = pts[i]!;
      const p2 = pts[(i + 1) % n]!;
      if (!this.#longEnoughOnScreen(p1, p2, 40)) {
        continue;
      }
      const mid = { lat: (p1.lat + p2.lat) / 2, lon: (p1.lon + p2.lon) / 2 };
      const marker = L.marker(toLL(mid), {
        icon: midpointIcon({ variant: kind }),
        draggable: true,
        keyboard: false,
        bubblingMouseEvents: false,
      }).addTo(this.#handlesLayer);
      marker.on("dragstart", () => {
        this.#setDragging("on");
      });
      marker.on("drag", () => {
        const list = [...pts];
        list.splice(i + 1, 0, fromLL(marker.getLatLng()));
        this.#renderShapes(kind === "safe" ? { ...m, safeArea: list } : { ...m, survey: list });
      });
      marker.on("dragend", () => {
        this.#setDragging("off");
        this.#cb.onPolygonInsert(kind, i + 1, fromLL(marker.getLatLng()));
      });
      marker.on("click", () => {
        this.#cb.onPolygonInsert(kind, i + 1, mid);
      });
    }
  }

  #makeHandle(p: LatLon, icon: L.DivIcon, kind: HandleKind, index: number): L.Marker {
    const marker = L.marker(toLL(p), {
      icon,
      draggable: this.#editable,
      interactive: this.#editable,
      keyboard: false,
      bubblingMouseEvents: false,
      autoPan: true,
    }).addTo(this.#handlesLayer);

    const sel = this.#selection;
    if (sel?.kind === kind && sel.index === index) {
      marker.getElement()?.classList.add("mp-selected");
      marker.setZIndexOffset(2000);
    }

    marker.on("click", () => {
      const m = this.#mission;
      const polygonDrawing =
        m != undefined &&
        (kind === "safe" || kind === "survey") &&
        this.#tool === kind &&
        index === 0 &&
        (kind === "safe" ? !m.safeClosed && m.safeArea.length >= 3 : !m.surveyClosed && m.survey.length >= 3);
      if (polygonDrawing) {
        this.#cb.onFirstVertexClick(kind);
      } else if (kind === "start" && this.#tool === "path" && m != undefined && m.path.length >= 2) {
        this.#cb.onSnapToStart();
      } else {
        this.#cb.onHandleClick(kind, index);
      }
    });
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
      case "survey": {
        const survey = [...m.survey];
        survey[index] = p;
        return { ...m, survey };
      }
    }
  }

  #setDragging(state: "on" | "off"): void {
    const on = state === "on";
    this.#dragging = on;
    if (on) {
      this.#rubberLabel.setOpacity(0);
    }
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

  /**
   * Draw the vehicle trail (stored points) and marker (latest raw position).
   * The trail polyline is appended incrementally while the history only grows,
   * and rebuilt when the window slides, is cleared or the colour changes.
   */
  public setVehicle(
    track: LatLon[],
    head: LatLon | undefined,
    opts: { color: string; follow: boolean },
  ): void {
    const { color, follow } = opts;
    const last = head ?? track[track.length - 1];
    if (!last) {
      this.#trail?.remove();
      this.#trail = undefined;
      this.#trailCount = 0;
      this.#trailFirst = undefined;
      this.#vehicleMarker?.remove();
      this.#vehicleMarker = undefined;
      this.#vehicleKey = "";
      this.#heading = undefined;
      this.#headingAnchor = undefined;
      this.#followWasOn = follow;
      return;
    }

    // --- trail
    if (track.length < 2) {
      this.#trail?.remove();
      this.#trail = undefined;
    } else if (!this.#trail || this.#trailColor !== color) {
      this.#trail?.remove();
      this.#trail = L.polyline(track.map(toLL), {
        pane: "mp-trail",
        interactive: false,
        color,
        weight: 2.5,
        opacity: 0.85,
      }).addTo(this.#map);
      this.#trailColor = color;
    } else if (track[0] === this.#trailFirst && track.length >= this.#trailCount) {
      for (let i = this.#trailCount; i < track.length; i++) {
        this.#trail.addLatLng(toLL(track[i]!));
      }
    } else {
      this.#trail.setLatLngs(track.map(toLL));
    }
    this.#trailFirst = track[0];
    this.#trailCount = track.length;

    // --- heading: course over ground. Only updated once the vehicle has really
    // moved >= 1.5 m from where the heading was last computed, so GNSS jitter
    // while (nearly) stationary can never twist the arrow.
    if (!this.#headingAnchor) {
      this.#headingAnchor = track[0] ?? last;
    }
    if (distanceM(this.#headingAnchor, last) >= 1.5) {
      this.#heading = bearingDeg(this.#headingAnchor, last);
      this.#headingAnchor = last;
    }
    const heading = this.#heading;
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

    // --- follow: centre once when enabled, afterwards only re-centre when the
    // vehicle leaves the middle half of the view (no per-fix jitter).
    if (follow && !this.#dragging) {
      if (!this.#followWasOn) {
        this.#panQuietly(last, { animate: false });
      } else {
        const size = this.#map.getSize();
        const p = this.#map.latLngToContainerPoint(toLL(last));
        if (
          Math.abs(p.x - size.x / 2) > size.x * 0.25 ||
          Math.abs(p.y - size.y / 2) > size.y * 0.25
        ) {
          this.#panQuietly(last, { animate: true });
        }
      }
    }
    this.#followWasOn = follow;
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

  #panQuietly(p: LatLon, opts: { animate: boolean }): void {
    this.#suppressViewEvents = true;
    this.#map.panTo(toLL(p), opts.animate ? { animate: true, duration: 0.5 } : { animate: false });
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
