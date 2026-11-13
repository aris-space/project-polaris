import {
  PanelExtensionContext,
  SettingsTreeAction,
  Time,
  Topic,
} from "@foxglove/extension";
import {
  KeyboardEvent,
  ReactElement,
  useCallback,
  useEffect,
  useLayoutEffect,
  useMemo,
  useRef,
  useState,
} from "react";
import { createRoot } from "react-dom/client";

import "./styles.css";
import { analyzeMission, distanceM } from "./geo";
import { HandleKind, MapCallbacks, MapController, Selection } from "./map/MapController";
import { extractFix } from "./messages";
import { buildSettingsNodes } from "./settings";
import {
  Config,
  DEFAULT_CONFIG,
  EMPTY_MISSION,
  Fix,
  LatLon,
  Mission,
  SavedMission,
  Tool,
} from "./types";
import { MissionMenu, UNTITLED } from "./ui/MissionMenu";
import { StatusBar } from "./ui/StatusBar";
import { ClearTarget, Toolbar, gnssUsable } from "./ui/Toolbar";
import { IconFit, IconFollow, IconVehicle } from "./ui/icons";

const UNDO_LIMIT = 50;
/** Safety cap for "All points" trail history. */
const TRAIL_CAP = 200_000;

function toMillis(t: Time | undefined): number | undefined {
  return t == undefined ? undefined : t.sec * 1000 + t.nsec / 1e6;
}

function isLatLon(v: unknown): v is LatLon {
  const p = v as LatLon | undefined;
  return p != undefined && Number.isFinite(p.lat) && Number.isFinite(p.lon);
}

/** Merge persisted layout state with defaults, defensively. */
function normalizeMission(raw: unknown): Mission {
  const sm = (raw ?? {}) as Partial<Mission>;
  const mission: Mission = {
    safeArea: Array.isArray(sm.safeArea) ? sm.safeArea.filter(isLatLon) : [],
    safeClosed: sm.safeClosed === true,
    start: isLatLon(sm.start) ? sm.start : null,
    path: Array.isArray(sm.path) ? sm.path.filter(isLatLon) : [],
    closed: sm.closed === true,
  };
  if (mission.safeArea.length < 3) {
    mission.safeClosed = false;
  }
  return mission;
}

function loadConfig(initial: unknown): Config {
  const saved = (initial ?? {}) as Partial<Config>;
  const missions: SavedMission[] = Array.isArray(saved.missions)
    ? saved.missions
        .filter((m) => typeof m.id === "string" && typeof m.name === "string")
        .map((m) => ({
          id: m.id,
          name: m.name,
          mission: normalizeMission(m.mission),
          savedAt: Number.isFinite(m.savedAt) ? m.savedAt : 0,
        }))
    : [];
  const activeMissionId =
    typeof saved.activeMissionId === "string" && missions.some((m) => m.id === saved.activeMissionId)
      ? saved.activeMissionId
      : null;
  return {
    ...DEFAULT_CONFIG,
    ...saved,
    mission: normalizeMission(saved.mission),
    missionName: typeof saved.missionName === "string" ? saved.missionName : "",
    activeMissionId,
    missions,
  };
}

function sameMission(a: Mission, b: Mission): boolean {
  return JSON.stringify(a) === JSON.stringify(b);
}

function isEmptyMission(m: Mission): boolean {
  return m.safeArea.length === 0 && m.start == undefined && m.path.length === 0;
}

function newMissionId(): string {
  return `m-${Date.now().toString(36)}-${Math.random().toString(36).slice(2, 7)}`;
}

/** Drop consecutive points closer than 5 cm (double-click artefacts). */
function dedupe(points: LatLon[]): LatLon[] {
  return points.filter((p, i) => i === 0 || distanceM(points[i - 1]!, p) > 0.05);
}

function MissionPlannerPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  // ------------------------------------------------------------------ config / mission state
  const [config, setConfig] = useState<Config>(() => loadConfig(context.initialState));
  const updateConfig = useCallback(
    (patch: Partial<Config>) => {
      setConfig((prev) => {
        const next = { ...prev, ...patch };
        context.saveState(next);
        return next;
      });
    },
    [context],
  );

  const configRef = useRef(config);
  configRef.current = config;
  const mission = config.mission;
  const missionRef = useRef(mission);
  missionRef.current = mission;
  const [selection, setSelection] = useState<Selection>();
  const past = useRef<Mission[]>([]);
  const future = useRef<Mission[]>([]);

  const commit = useCallback(
    (next: Mission) => {
      past.current.push(missionRef.current);
      if (past.current.length > UNDO_LIMIT) {
        past.current.shift();
      }
      future.current = [];
      missionRef.current = next;
      updateConfig({ mission: next });
    },
    [updateConfig],
  );

  const undo = useCallback(() => {
    const prev = past.current.pop();
    if (!prev) {
      return;
    }
    future.current.push(missionRef.current);
    setSelection(undefined);
    missionRef.current = prev;
    updateConfig({ mission: prev });
  }, [updateConfig]);

  const redo = useCallback(() => {
    const next = future.current.pop();
    if (!next) {
      return;
    }
    past.current.push(missionRef.current);
    setSelection(undefined);
    missionRef.current = next;
    updateConfig({ mission: next });
  }, [updateConfig]);

  const [tool, setTool] = useState<Tool>("pan");
  const [cursor, setCursor] = useState<LatLon | undefined>();

  // ------------------------------------------------------------------ Foxglove render loop
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [colorScheme, setColorScheme] = useState<"dark" | "light">("dark");
  const [topics, setTopics] = useState<readonly Topic[]>([]);
  const [now, setNow] = useState(() => Date.now());
  const [gnssFix, setGnssFix] = useState<Fix | undefined>();
  /** Stored trail points (thinned by min spacing, capped by the history setting). */
  const trackRef = useRef<LatLon[]>([]);
  /** Latest raw vehicle position (marker), even when not stored in the trail. */
  const headRef = useRef<LatLon | undefined>();
  const [trackVersion, setTrackVersion] = useState(0);
  const resetTrack = useCallback(() => {
    trackRef.current = [];
    headRef.current = undefined;
    setTrackVersion((v) => v + 1);
  }, []);

  const liveValue = {
    vehicle: config.vehicleTopic,
    gnss: config.gnssTopic,
    keep: config.vehicleHistoryMode === "all" ? TRAIL_CAP : Math.max(0, config.vehicleHistory),
    minSpacing: Math.max(0, config.vehicleMinSpacing),
  };
  const live = useRef(liveValue);
  live.current = liveValue;

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      if (renderState.colorScheme) {
        setColorScheme(renderState.colorScheme);
      }
      if (renderState.topics) {
        setTopics(renderState.topics);
      }
      const nowMs = toMillis(renderState.currentTime) ?? Date.now();
      setNow(nowMs);

      if (renderState.didSeek === true) {
        trackRef.current = [];
        headRef.current = undefined;
        setTrackVersion((v) => v + 1);
        setGnssFix(undefined);
      }

      const frame = renderState.currentFrame;
      if (!frame || frame.length === 0) {
        return;
      }
      const { vehicle, gnss, keep, minSpacing } = live.current;
      let trackChanged = false;
      let latestGnss: Fix | undefined;
      for (const ev of frame) {
        const at = toMillis(ev.receiveTime) ?? nowMs;
        if (ev.topic === vehicle) {
          const fix = extractFix(ev.message, at);
          if (fix && (fix.status == undefined || fix.status >= 0)) {
            const p = { lat: fix.lat, lon: fix.lon };
            const head = headRef.current;
            if (head?.lat !== p.lat || head.lon !== p.lon) {
              headRef.current = p;
              trackChanged = true;
              const track = trackRef.current;
              const last = track[track.length - 1];
              if (keep > 0 && (!last || distanceM(last, p) >= Math.max(minSpacing, 1e-6))) {
                track.push(p);
                if (track.length > keep) {
                  track.splice(0, track.length - keep);
                }
              }
            }
          }
        }
        if (ev.topic === gnss) {
          latestGnss = extractFix(ev.message, at) ?? latestGnss;
        }
      }
      if (trackChanged) {
        setTrackVersion((v) => v + 1);
      }
      if (latestGnss) {
        setGnssFix(latestGnss);
      }
    };
    context.watch("currentFrame");
    context.watch("currentTime");
    context.watch("topics");
    context.watch("colorScheme");
    context.watch("didSeek");
    context.setDefaultPanelTitle("Mission Planner");
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    const subs = [...new Set([config.vehicleTopic, config.gnssTopic].filter((t) => t.length > 0))];
    context.subscribe(subs.map((topic) => ({ topic })));
  }, [context, config.vehicleTopic, config.gnssTopic]);

  useEffect(() => {
    resetTrack();
  }, [config.vehicleTopic, resetTrack]);

  // Shrinking the history applies immediately.
  useEffect(() => {
    const keep = live.current.keep;
    if (trackRef.current.length > keep) {
      trackRef.current.splice(0, trackRef.current.length - keep);
      setTrackVersion((v) => v + 1);
    }
  }, [config.vehicleHistoryMode, config.vehicleHistory]);

  useEffect(() => {
    setGnssFix(undefined);
  }, [config.gnssTopic]);

  // ------------------------------------------------------------------ mission editing
  const finishDrawing = useCallback(() => {
    const m = missionRef.current;
    if (tool === "safe" && !m.safeClosed) {
      const safeArea = dedupe(m.safeArea);
      if (safeArea.length >= 3) {
        commit({ ...m, safeArea, safeClosed: true });
        setTool(m.start ? "pan" : "start");
      }
      return;
    }
    if (tool === "path") {
      const path = dedupe(m.path);
      if (path.length !== m.path.length) {
        commit({ ...m, path });
      }
    }
    setTool("pan");
  }, [tool, commit]);

  const removeLastPoint = useCallback(() => {
    const m = missionRef.current;
    if (tool === "safe" && !m.safeClosed && m.safeArea.length > 0) {
      commit({ ...m, safeArea: m.safeArea.slice(0, -1) });
    } else if (tool === "path" && m.path.length > 0) {
      const path = m.path.slice(0, -1);
      commit({ ...m, path, closed: m.closed && path.length >= 2 });
    }
  }, [tool, commit]);

  const setStart = useCallback(
    (p: LatLon) => {
      const m = missionRef.current;
      commit({ ...m, start: p });
      setTool(m.path.length === 0 ? "path" : "pan");
    },
    [commit],
  );

  const toggleLoop = useCallback(() => {
    const m = missionRef.current;
    if (m.start && m.path.length >= 2) {
      commit({ ...m, closed: !m.closed });
    }
  }, [commit]);

  const startFromGnss = useCallback(() => {
    if (gnssFix && gnssUsable(gnssFix, now)) {
      const p = { lat: gnssFix.lat, lon: gnssFix.lon };
      setStart(p);
      mapRef.current?.centerOn(p);
    }
  }, [gnssFix, now, setStart]);

  /** Delete one point. Returns false if not allowed (closed safe area needs >= 3 vertices). */
  const deletePoint = useCallback(
    (kind: HandleKind, index: number): boolean => {
      const m = missionRef.current;
      if (kind === "start") {
        commit({ ...m, start: null });
        if (tool === "path") {
          setTool("pan");
        }
      } else if (kind === "safe") {
        if (m.safeClosed && m.safeArea.length <= 3) {
          return false;
        }
        commit({ ...m, safeArea: m.safeArea.filter((_, i) => i !== index) });
      } else {
        const path = m.path.filter((_, i) => i !== index);
        commit({ ...m, path, closed: m.closed && path.length >= 2 });
      }
      setSelection(undefined);
      return true;
    },
    [commit, tool],
  );

  // ------------------------------------------------------------------ saved missions
  const activeSaved = config.missions.find((m) => m.id === config.activeMissionId);
  const dirty = activeSaved
    ? !sameMission(activeSaved.mission, mission) || activeSaved.name !== (config.missionName.trim() || UNTITLED)
    : !isEmptyMission(mission);

  const saveMission = useCallback(
    (opts: { asCopy: boolean }) => {
      const c = configRef.current;
      const savedAt = Date.now();
      let name = c.missionName.trim() || UNTITLED;
      const existing = c.missions.find((m) => m.id === c.activeMissionId);
      if (existing && !opts.asCopy) {
        updateConfig({
          missionName: name,
          missions: c.missions.map((m) =>
            m.id === existing.id ? { ...m, name, mission: missionRef.current, savedAt } : m,
          ),
        });
        return;
      }
      if (opts.asCopy) {
        name = `${name} (copy)`;
      }
      const id = newMissionId();
      updateConfig({
        missionName: name,
        activeMissionId: id,
        missions: [...c.missions, { id, name, mission: missionRef.current, savedAt }],
      });
    },
    [updateConfig],
  );

  const loadMission = useCallback(
    (id: string) => {
      const sm = configRef.current.missions.find((m) => m.id === id);
      if (!sm) {
        return;
      }
      commit(sm.mission);
      updateConfig({ activeMissionId: sm.id, missionName: sm.name });
      setTool("pan");
      mapRef.current?.fitMission(sm.mission);
    },
    [commit, updateConfig],
  );

  const deleteMission = useCallback(
    (id: string) => {
      const c = configRef.current;
      updateConfig({
        missions: c.missions.filter((m) => m.id !== id),
        activeMissionId: c.activeMissionId === id ? null : c.activeMissionId,
      });
    },
    [updateConfig],
  );

  const newMission = useCallback(() => {
    commit(EMPTY_MISSION);
    updateConfig({ activeMissionId: null, missionName: "" });
    setTool("pan");
  }, [commit, updateConfig]);

  const clear = useCallback(
    (target: ClearTarget) => {
      setSelection(undefined);
      const m = missionRef.current;
      if (target === "safe") {
        commit({ ...m, safeArea: [], safeClosed: false });
      } else if (target === "path") {
        commit({ ...m, start: null, path: [], closed: false });
        if (tool === "path") {
          setTool("pan");
        }
      } else {
        commit(EMPTY_MISSION);
        setTool("pan");
      }
    },
    [commit, tool],
  );

  const selectTool = useCallback((t: Tool) => {
    if (t === "path" && !missionRef.current.start) {
      return;
    }
    setTool(t);
  }, []);

  // ------------------------------------------------------------------ map
  const mapDivRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<MapController>();

  const handlers: MapCallbacks = {
    onMapClick: (p, clickCount) => {
      setSelection(undefined);
      if (clickCount > 1) {
        return; // second click of a double-click: handled by onMapDblClick
      }
      const m = missionRef.current;
      if (tool === "safe" && !m.safeClosed) {
        commit({ ...m, safeArea: [...m.safeArea, p] });
      } else if (tool === "start") {
        setStart(p);
      } else if (tool === "path" && m.start) {
        commit({ ...m, path: [...m.path, p] });
      }
    },
    onMapDblClick: finishDrawing,
    onFirstSafeVertexClick: finishDrawing,
    onHandleMoved: (kind: HandleKind, index, p) => {
      const m = missionRef.current;
      if (kind === "start") {
        commit({ ...m, start: p });
      } else if (kind === "safe") {
        const safeArea = [...m.safeArea];
        safeArea[index] = p;
        commit({ ...m, safeArea });
      } else {
        const path = [...m.path];
        path[index] = p;
        commit({ ...m, path });
      }
    },
    onHandleDelete: (kind, index) => {
      deletePoint(kind, index);
    },
    onHandleClick: (kind, index) => {
      setSelection((cur) => (cur?.kind === kind && cur.index === index ? undefined : { kind, index }));
    },
    onSnapToStart: () => {
      const m = missionRef.current;
      if (!m.closed) {
        commit({ ...m, closed: true });
      }
      setSelection(undefined);
      setTool("pan");
    },
    onSafeInsert: (index, p) => {
      const m = missionRef.current;
      const safeArea = [...m.safeArea];
      safeArea.splice(index, 0, p);
      commit({ ...m, safeArea });
    },
    onCursor: setCursor,
    onViewChange: (view) => {
      updateConfig({ view });
    },
    onUserPan: () => {
      if (configRef.current.followVehicle) {
        updateConfig({ followVehicle: false });
      }
    },
  };
  const handlersRef = useRef(handlers);
  handlersRef.current = handlers;

  useEffect(() => {
    const el = mapDivRef.current;
    if (!el) {
      return undefined;
    }
    // Stable proxy so the controller always calls the latest closures.
    const proxy: MapCallbacks = {
      onMapClick: (...a) => {
        handlersRef.current.onMapClick(...a);
      },
      onMapDblClick: () => {
        handlersRef.current.onMapDblClick();
      },
      onFirstSafeVertexClick: () => {
        handlersRef.current.onFirstSafeVertexClick();
      },
      onHandleMoved: (...a) => {
        handlersRef.current.onHandleMoved(...a);
      },
      onHandleDelete: (...a) => {
        handlersRef.current.onHandleDelete(...a);
      },
      onSafeInsert: (...a) => {
        handlersRef.current.onSafeInsert(...a);
      },
      onHandleClick: (...a) => {
        handlersRef.current.onHandleClick(...a);
      },
      onSnapToStart: () => {
        handlersRef.current.onSnapToStart();
      },
      onCursor: (...a) => {
        handlersRef.current.onCursor(...a);
      },
      onViewChange: (...a) => {
        handlersRef.current.onViewChange(...a);
      },
      onUserPan: () => {
        handlersRef.current.onUserPan();
      },
    };
    const ctrl = new MapController(el, proxy, configRef.current.view);
    mapRef.current = ctrl;
    return () => {
      ctrl.destroy();
      mapRef.current = undefined;
    };
  }, []);

  // Drop a selection that no longer points at an existing point.
  const validSelection: Selection =
    selection == undefined
      ? undefined
      : selection.kind === "start"
        ? mission.start
          ? selection
          : undefined
        : selection.index < (selection.kind === "safe" ? mission.safeArea : mission.path).length
          ? selection
          : undefined;
  const selKey = validSelection ? `${validSelection.kind}:${validSelection.index}` : "";
  useEffect(() => {
    mapRef.current?.setSelection(validSelection);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [selKey, mission, tool]);

  const dark = colorScheme === "dark";
  useEffect(() => {
    mapRef.current?.setBasemap(config.basemap, config.customTileUrl, { dark });
  }, [config.basemap, config.customTileUrl, dark]);

  useEffect(() => {
    const ctrl = mapRef.current;
    if (!ctrl) {
      return;
    }
    ctrl.setTool(tool);
    ctrl.setMission(mission);
    if (tool === "safe" && !mission.safeClosed && mission.safeArea.length > 0) {
      ctrl.setRubberAnchors(
        mission.safeArea[mission.safeArea.length - 1],
        mission.safeArea.length >= 2 ? mission.safeArea[0] : undefined,
      );
    } else if (tool === "path" && mission.start) {
      ctrl.setRubberAnchors(
        mission.path[mission.path.length - 1] ?? mission.start,
        mission.closed ? mission.start : undefined,
      );
    } else {
      ctrl.setRubberAnchors(undefined, undefined);
    }
  }, [mission, tool]);

  useEffect(() => {
    mapRef.current?.setVehicle(trackRef.current, headRef.current, {
      color: config.vehicleColor,
      follow: config.followVehicle,
    });
  }, [trackVersion, config.vehicleColor, config.followVehicle]);

  useEffect(() => {
    mapRef.current?.setGnss(config.showGnss ? gnssFix : undefined);
  }, [gnssFix, config.showGnss]);

  // First open without a saved view: fit to the mission, else jump to the first vehicle fix.
  const fittedRef = useRef(config.view != undefined);
  useEffect(() => {
    if (fittedRef.current || !mapRef.current) {
      return;
    }
    if (mapRef.current.fitMission(mission)) {
      fittedRef.current = true;
      return;
    }
    const first = headRef.current ?? gnssFix;
    if (first) {
      mapRef.current.centerOn(first, 17);
      fittedRef.current = true;
    }
  }, [mission, trackVersion, gnssFix]);

  const vehiclePos = headRef.current;
  const fitAll = useCallback(() => {
    const extra = vehiclePos ? [vehiclePos] : [];
    mapRef.current?.fitMission(missionRef.current, extra);
  }, [vehiclePos]);

  // ------------------------------------------------------------------ settings editor
  const settingsActionHandler = useCallback(
    (action: SettingsTreeAction) => {
      if (action.action === "perform-node-action") {
        if (action.payload.id === "clear-mission") {
          commit(EMPTY_MISSION);
          setTool("pan");
        } else if (action.payload.id === "clear-trail") {
          const head = headRef.current;
          trackRef.current = head ? [head] : [];
          setTrackVersion((v) => v + 1);
        }
        return;
      }
      const { path, value } = action.payload as { path: readonly string[]; value: unknown };
      const key = path[1] ?? "";
      switch (key) {
        case "basemap":
          updateConfig({ basemap: value as Config["basemap"] });
          break;
        case "customTileUrl":
          updateConfig({ customTileUrl: typeof value === "string" ? value : "" });
          break;
        case "vehicleTopic":
          updateConfig({ vehicleTopic: typeof value === "string" ? value.trim() : "" });
          break;
        case "gnssTopic":
          updateConfig({ gnssTopic: typeof value === "string" ? value.trim() : "" });
          break;
        case "vehicleHistory": {
          const n = Math.round(Number(value));
          updateConfig({ vehicleHistory: Number.isFinite(n) ? Math.min(100000, Math.max(0, n)) : 0 });
          break;
        }
        case "vehicleHistoryMode":
          updateConfig({ vehicleHistoryMode: value === "last" ? "last" : "all" });
          break;
        case "vehicleMinSpacing": {
          const m = Number(value);
          updateConfig({ vehicleMinSpacing: Number.isFinite(m) ? Math.min(50, Math.max(0, m)) : 0 });
          break;
        }
        case "vehicleColor":
          updateConfig({ vehicleColor: typeof value === "string" && value ? value : DEFAULT_CONFIG.vehicleColor });
          break;
        case "followVehicle":
          updateConfig({ followVehicle: value === true });
          break;
        case "showGnss":
          updateConfig({ showGnss: value === true });
          break;
        default:
          break;
      }
    },
    [commit, updateConfig],
  );

  useEffect(() => {
    context.updatePanelSettingsEditor({
      actionHandler: settingsActionHandler,
      nodes: buildSettingsNodes(config, topics),
    });
  }, [context, config, topics, settingsActionHandler]);

  // ------------------------------------------------------------------ layout / keyboard
  const rootRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState({ w: 1000, h: 800 });
  useEffect(() => {
    const el = rootRef.current;
    if (!el) {
      return undefined;
    }
    const ro = new ResizeObserver((entries) => {
      const r = entries[0]?.contentRect;
      if (r) {
        setSize({ w: r.width, h: r.height });
      }
    });
    ro.observe(el);
    return () => {
      ro.disconnect();
    };
  }, []);
  const compact = size.w < 600 || size.h < 460;

  const onKeyDown = (e: KeyboardEvent<HTMLDivElement>) => {
    const target = e.target as HTMLElement;
    if (target.tagName === "INPUT" || target.tagName === "TEXTAREA") {
      return;
    }
    const mod = e.metaKey || e.ctrlKey;
    let handled = true;
    if (mod && e.key.toLowerCase() === "z") {
      if (e.shiftKey) {
        redo();
      } else {
        undo();
      }
    } else if (mod && e.key.toLowerCase() === "y") {
      redo();
    } else if (mod && e.key.toLowerCase() === "s") {
      saveMission({ asCopy: false });
    } else if (mod) {
      handled = false;
    } else {
      switch (e.key) {
        case "Escape":
          if (validSelection) {
            setSelection(undefined);
            break;
          }
          // Leave the tool; an unfinished safe area with <3 points stays as a draft.
          if (tool === "safe" && !mission.safeClosed && mission.safeArea.length < 3) {
            setTool("pan");
          } else {
            finishDrawing();
          }
          break;
        case "Enter":
          finishDrawing();
          break;
        case "Backspace":
        case "Delete":
          if (validSelection) {
            deletePoint(validSelection.kind, validSelection.index);
          } else {
            removeLastPoint();
          }
          break;
        case "1":
          selectTool(tool === "safe" ? "pan" : "safe");
          break;
        case "2":
          selectTool(tool === "start" ? "pan" : "start");
          break;
        case "3":
          selectTool(tool === "path" ? "pan" : "path");
          break;
        case "g":
        case "G":
          startFromGnss();
          break;
        case "l":
        case "L":
          toggleLoop();
          break;
        case "f":
        case "F":
          fitAll();
          break;
        default:
          handled = false;
      }
    }
    if (handled) {
      e.preventDefault();
      e.stopPropagation();
    }
  };

  const analysis = useMemo(() => analyzeMission(mission), [mission]);

  return (
    <div
      ref={rootRef}
      className="mp-root"
      data-theme={colorScheme}
      tabIndex={0}
      onKeyDown={onKeyDown}
      onMouseDown={() => rootRef.current?.focus({ preventScroll: true })}
    >
      <div ref={mapDivRef} className="mp-map" />

      <Toolbar
        tool={tool}
        mission={mission}
        gnssFix={gnssFix}
        gnssTopic={config.gnssTopic}
        now={now}
        compact={compact}
        canUndo={past.current.length > 0}
        canRedo={future.current.length > 0}
        onTool={selectTool}
        onGnssStart={startFromGnss}
        onToggleLoop={toggleLoop}
        onUndo={undo}
        onRedo={redo}
        onClear={clear}
      />

      <MissionMenu
        name={config.missionName}
        missions={config.missions}
        activeId={config.activeMissionId}
        dirty={dirty}
        onNameChange={(missionName) => {
          updateConfig({ missionName });
        }}
        onSave={() => {
          saveMission({ asCopy: false });
        }}
        onSaveCopy={() => {
          saveMission({ asCopy: true });
        }}
        onLoad={loadMission}
        onDelete={deleteMission}
        onNew={newMission}
      />

      <div className="mp-group mp-viewbtns">
        <button className="mp-btn" onClick={fitAll} data-tip="Fit mission (F)">
          <IconFit />
        </button>
        <button
          className="mp-btn"
          disabled={!vehiclePos}
          onClick={() => {
            if (vehiclePos) {
              mapRef.current?.centerOn(vehiclePos);
            }
          }}
          data-tip={vehiclePos ? "Center on vehicle" : `No vehicle position on ${config.vehicleTopic || "(no topic)"}`}
        >
          <IconVehicle />
        </button>
        <button
          className={`mp-btn${config.followVehicle ? " mp-btn--active" : ""}`}
          onClick={() => {
            updateConfig({ followVehicle: !config.followVehicle });
          }}
          aria-pressed={config.followVehicle}
          data-tip={config.followVehicle ? "Following vehicle — click or drag map to stop" : "Follow vehicle"}
        >
          <IconFollow />
        </button>
      </div>

      <StatusBar
        tool={tool}
        mission={mission}
        analysis={analysis}
        cursor={cursor}
        selection={validSelection}
      />
    </div>
  );
}

export function initMissionPlannerPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<MissionPlannerPanel context={context} />);
  return () => {
    root.unmount();
  };
}
