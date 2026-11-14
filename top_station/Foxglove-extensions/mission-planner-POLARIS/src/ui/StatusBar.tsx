import { ReactElement } from "react";

import { MissionAnalysis, formatArea, formatDistance, formatLatLon } from "../geo";
import { Selection } from "../map/MapController";
import { LatLon, Mission, Tool } from "../types";

type Props = {
  tool: Tool;
  mission: Mission;
  analysis: MissionAnalysis;
  cursor: LatLon | undefined;
  selection: Selection;
  watch: boolean;
};

const TOOL_NAMES: Record<Tool, string> = {
  pan: "Pan",
  safe: "Safe area",
  start: "Start",
  path: "Path",
  survey: "Survey area",
};

function selectionHint(sel: NonNullable<Selection>, m: Mission): ReactElement {
  const name =
    sel.kind === "start"
      ? "Start point"
      : sel.kind === "path"
        ? `Waypoint ${sel.index + 1}`
        : sel.kind === "survey"
          ? `Survey vertex ${sel.index + 1}`
          : `Safe-area vertex ${sel.index + 1}`;
  const atMinimum =
    (sel.kind === "safe" && m.safeClosed && m.safeArea.length <= 3) ||
    (sel.kind === "survey" && m.surveyClosed && m.survey.length <= 3);
  if (atMinimum) {
    return (
      <>
        <b>{name}</b> selected · an area needs at least 3 vertices · <kbd>Esc</kbd> deselect
      </>
    );
  }
  return (
    <>
      <b>{name}</b> selected · <kbd>Delete</kbd> remove · drag to move · <kbd>Esc</kbd> deselect
    </>
  );
}

function hint(tool: Tool, m: Mission): ReactElement {
  switch (tool) {
    case "safe":
      if (m.safeClosed) {
        return (
          <>
            Drag vertices · drag <b>+</b> to insert · right-click to delete
          </>
        );
      }
      if (m.safeArea.length < 3) {
        return (
          <>
            Click to add vertices — {3 - m.safeArea.length} more needed · <kbd>⌫</kbd> remove last
          </>
        );
      }
      return (
        <>
          Click first vertex, <kbd>Enter</kbd> or double-click to close · <kbd>⌫</kbd> remove last
        </>
      );
    case "start":
      return <>Click on the map to place the start point</>;
    case "survey":
      if (m.surveyClosed) {
        return (
          <>
            Drag vertices · drag <b>+</b> to insert · right-click to delete
          </>
        );
      }
      if (m.survey.length < 3) {
        return (
          <>
            Click around the area to survey — {3 - m.survey.length} more needed · <kbd>⌫</kbd> remove last
          </>
        );
      }
      return (
        <>
          Click first vertex, <kbd>Enter</kbd> or double-click to close · <kbd>⌫</kbd> remove last
        </>
      );
    case "path":
      return (
        <>
          Click to add waypoint {m.path.length + 1} ·{" "}
          {m.path.length >= 2 ? (
            <>
              click <b>START</b> to close loop ·{" "}
            </>
          ) : undefined}
          <kbd>Enter</kbd> finish · <kbd>⌫</kbd> remove last
        </>
      );
    case "pan":
      if (!m.safeClosed) {
        return (
          <>
            Start by drawing a safe area — press <kbd>1</kbd>
          </>
        );
      }
      if (!m.start) {
        return (
          <>
            Set a start point — press <kbd>2</kbd> or use <b>Start from GNSS</b>
          </>
        );
      }
      if (m.type === "survey" && !m.surveyClosed) {
        return (
          <>
            Mark the survey area — press <kbd>3</kbd>
          </>
        );
      }
      if (m.type === "waypoints" && m.path.length === 0) {
        return (
          <>
            Plan the path — press <kbd>3</kbd>
          </>
        );
      }
      return <>Click a point to select it · drag to move · right-click to delete</>;
  }
}

export function StatusBar({ tool, mission, analysis, cursor, selection, watch }: Props): ReactElement {
  const warnStart = analysis.startOutside;
  const warnSegs = analysis.outsideSegments;
  return (
    <div className="mp-status">
      <div className="mp-hint">
        {watch ? (
          <>
            <span className="mp-hint-tool mp-hint-tool--watch">Watch</span>
            <span>View only — switch to Plan to edit the mission</span>
          </>
        ) : (
          <>
            {tool !== "pan" && <span className="mp-hint-tool">{TOOL_NAMES[tool]}</span>}
            <span>{selection ? selectionHint(selection, mission) : hint(tool, mission)}</span>
          </>
        )}
      </div>
      <div className="mp-stats">
        {mission.safeClosed && (
          <span className="mp-stat">
            Safe <b>{formatArea(analysis.safeAreaM2)}</b>
          </span>
        )}
        {mission.start && (mission.type === "waypoints" || mission.path.length > 0) && (
          <span className="mp-stat">
            <b>{mission.path.length}</b> wp · <b>{formatDistance(analysis.routeLengthM)}</b>
            {mission.closed ? " loop" : ""}
          </span>
        )}
        {mission.surveyClosed && (
          <span className="mp-stat">
            Survey <b>{formatArea(analysis.surveyAreaM2)}</b>
          </span>
        )}
        {analysis.surveyOutside && (
          <span className="mp-stat mp-stat--warn">⚠ Survey area outside safe area</span>
        )}
        {mission.safeClosed &&
          mission.surveyClosed &&
          !analysis.surveyOutside &&
          !warnStart && <span className="mp-stat mp-stat--ok">✓ Survey inside safe area</span>}
        {warnStart && <span className="mp-stat mp-stat--warn">⚠ Start outside safe area</span>}
        {warnSegs > 0 && (
          <span className="mp-stat mp-stat--warn">
            ⚠ <b>{warnSegs}</b> segment{warnSegs === 1 ? "" : "s"} outside safe area
          </span>
        )}
        {mission.safeClosed && mission.path.length > 0 && !warnStart && warnSegs === 0 && (
          <span className="mp-stat mp-stat--ok">✓ Path inside safe area</span>
        )}
      </div>
      <div className="mp-coords">{cursor ? formatLatLon(cursor) : ""}</div>
    </div>
  );
}
