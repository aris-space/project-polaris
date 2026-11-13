import { ReactElement, useEffect, useRef, useState } from "react";

import {
  IconEdit,
  IconEye,
  IconGnss,
  IconLoop,
  IconPan,
  IconPath,
  IconRedo,
  IconSafeArea,
  IconStart,
  IconTrash,
  IconUndo,
} from "./icons";
import { Fix, Mission, PanelMode, Tool } from "../types";

export type ClearTarget = "safe" | "path" | "all";

type Props = {
  mode: PanelMode;
  onMode: (mode: PanelMode) => void;
  tool: Tool;
  mission: Mission;
  gnssFix: Fix | undefined;
  gnssTopic: string;
  now: number;
  compact: boolean;
  canUndo: boolean;
  canRedo: boolean;
  onTool: (tool: Tool) => void;
  onGnssStart: () => void;
  onToggleLoop: () => void;
  onUndo: () => void;
  onRedo: () => void;
  onClear: (target: ClearTarget) => void;
};

/** A fix older than this is treated as unusable for "Start from GNSS". */
export const GNSS_STALE_MS = 5000;

export function gnssUsable(fix: Fix | undefined, now: number): boolean {
  return (
    fix != undefined && (fix.status == undefined || fix.status >= 0) && now - fix.receivedAt < GNSS_STALE_MS
  );
}

function gnssTip(fix: Fix | undefined, topic: string, now: number): string {
  if (!fix) {
    return `Start from GNSS — waiting for a fix on ${topic || "(no topic set)"}`;
  }
  const age = (now - fix.receivedAt) / 1000;
  if (fix.status != undefined && fix.status < 0) {
    return `Start from GNSS — receiver reports NO FIX on ${topic}`;
  }
  if (age * 1000 >= GNSS_STALE_MS) {
    return `Start from GNSS — last fix on ${topic} is ${age.toFixed(0)} s old`;
  }
  const acc = fix.hAcc != undefined ? ` · ±${fix.hAcc < 1 ? fix.hAcc.toFixed(2) : fix.hAcc.toFixed(1)} m` : "";
  return `Set start to current GNSS fix (${topic}, ${age.toFixed(1)} s old${acc})`;
}

function gnssDotColor(fix: Fix | undefined, now: number): string {
  if (!gnssUsable(fix, now)) {
    return "#6b7280";
  }
  if (fix?.hAcc != undefined && fix.hAcc > 2) {
    return "#f59e0b";
  }
  return "#34d399";
}

export function Toolbar(props: Props): ReactElement {
  const { tool, mission, gnssFix, now, compact } = props;
  const [menuOpen, setMenuOpen] = useState(false);
  const [confirm, setConfirm] = useState<ClearTarget | undefined>();
  const menuRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!menuOpen) {
      return undefined;
    }
    const onDown = (e: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(e.target as Node)) {
        setMenuOpen(false);
        setConfirm(undefined);
      }
    };
    document.addEventListener("mousedown", onDown);
    return () => {
      document.removeEventListener("mousedown", onDown);
    };
  }, [menuOpen]);

  const hasStart = mission.start != undefined;
  const canLoop = hasStart && mission.path.length >= 2;
  const gnssOk = gnssUsable(gnssFix, now);

  const toolBtn = (
    t: Tool,
    label: string,
    icon: ReactElement,
    kbd: string,
    tip: string,
    opts: { disabled?: boolean } = {},
  ) => (
    <button
      className={`mp-btn${tool === t ? " mp-btn--active" : ""}`}
      onClick={() => {
        props.onTool(tool === t && t !== "pan" ? "pan" : t);
      }}
      disabled={opts.disabled}
      data-tip={compact ? `${label} — ${tip}` : tip}
      aria-pressed={tool === t}
    >
      {icon}
      <span className="mp-btn-label">{label}</span>
      <span className="mp-kbd">{kbd}</span>
    </button>
  );

  const clearItem = (target: ClearTarget, label: string, opts: { disabled: boolean }) => (
    <button
      className={`mp-btn ${confirm === target ? "mp-btn--confirm" : "mp-btn--danger"}`}
      disabled={opts.disabled}
      onClick={() => {
        if (confirm === target) {
          props.onClear(target);
          setConfirm(undefined);
          setMenuOpen(false);
        } else {
          setConfirm(target);
        }
      }}
    >
      <span className="mp-btn-label" style={{ display: "inline" }}>
        {confirm === target ? "Click again to confirm" : label}
      </span>
    </button>
  );

  const safeEmpty = mission.safeArea.length === 0;
  const pathEmpty = mission.start == undefined && mission.path.length === 0;

  return (
    <div className={`mp-toolbar${compact ? " mp-compact" : ""}`}>
      <ModeSwitch mode={props.mode} onMode={props.onMode} compact={compact} />
      {props.mode === "plan" && (
        <>
      <div className="mp-group">
        {toolBtn("pan", "Pan", <IconPan />, "Esc", "Move the map and drag existing points")}
        {toolBtn(
          "safe",
          "Safe area",
          <IconSafeArea />,
          "1",
          mission.safeClosed
            ? "Edit the safe area: drag vertices, drag “+” to insert, right-click to delete"
            : "Click to place ≥3 vertices; click the first vertex, Enter or double-click to finish",
        )}
        {toolBtn("start", "Start point", <IconStart />, "2", "Click on the map to place the start cross")}
        <button
          className="mp-btn"
          onClick={props.onGnssStart}
          disabled={!gnssOk}
          data-tip={gnssTip(gnssFix, props.gnssTopic, now)}
        >
          <IconGnss />
          <span className="mp-btn-label">Start from GNSS</span>
          <span className="mp-dot" style={{ background: gnssDotColor(gnssFix, now) }} />
        </button>
        {toolBtn(
          "path",
          "Path",
          <IconPath />,
          "3",
          hasStart
            ? "Click to add waypoints after the start; Enter or double-click to finish"
            : "Set a start point first",
          { disabled: !hasStart },
        )}
        <button
          className={`mp-btn${mission.closed ? " mp-btn--on" : ""}`}
          onClick={props.onToggleLoop}
          disabled={!canLoop}
          aria-pressed={mission.closed}
          data-tip={
            canLoop
              ? mission.closed
                ? "Open the loop (remove the return leg to start)"
                : "Close the loop: connect the last waypoint back to the start"
              : "Needs a start point and at least 2 waypoints"
          }
        >
          <IconLoop />
          <span className="mp-btn-label">{mission.closed ? "Loop closed" : "Close loop"}</span>
          <span className="mp-kbd">L</span>
        </button>
      </div>

      <div className="mp-group" ref={menuRef} style={{ position: "relative" }}>
        <div className="mp-row">
          <button className="mp-btn" onClick={props.onUndo} disabled={!props.canUndo} data-tip="Undo (Ctrl/⌘ Z)">
            <IconUndo />
          </button>
          <button
            className="mp-btn"
            onClick={props.onRedo}
            disabled={!props.canRedo}
            data-tip="Redo (Ctrl/⌘ Shift Z)"
          >
            <IconRedo />
          </button>
          <button
            className={`mp-btn${menuOpen ? " mp-btn--active" : ""}`}
            onClick={() => {
              setMenuOpen((o) => !o);
              setConfirm(undefined);
            }}
            disabled={safeEmpty && pathEmpty}
            data-tip="Clear…"
          >
            <IconTrash />
          </button>
        </div>
        {menuOpen && (
          <div className="mp-menu">
            <div className="mp-menu-title">Clear</div>
            {clearItem("safe", "Safe area", { disabled: safeEmpty })}
            {clearItem("path", "Start & path", { disabled: pathEmpty })}
            <div className="mp-sep" />
            {clearItem("all", "Everything", { disabled: safeEmpty && pathEmpty })}
          </div>
        )}
      </div>
        </>
      )}
    </div>
  );
}

function ModeSwitch(props: {
  mode: PanelMode;
  onMode: (mode: PanelMode) => void;
  compact: boolean;
}): ReactElement {
  const item = (m: PanelMode, label: string, icon: ReactElement, tip: string) => (
    <button
      className={`mp-seg${props.mode === m ? " mp-seg--active" : ""}`}
      onClick={() => {
        props.onMode(m);
      }}
      aria-pressed={props.mode === m}
      data-tip={tip}
    >
      {icon}
      {!props.compact && <span>{label}</span>}
    </button>
  );
  return (
    <div className="mp-group mp-modeswitch" role="group" aria-label="Panel mode">
      {item("plan", "Plan", <IconEdit />, "Plan mode: draw and edit the mission")}
      {item("watch", "Watch", <IconEye />, "Watch mode: view only — the mission is locked")}
    </div>
  );
}
