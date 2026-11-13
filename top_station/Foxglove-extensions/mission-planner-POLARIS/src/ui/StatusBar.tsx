import { ReactElement } from "react";

import { MissionAnalysis, formatArea, formatDistance, formatLatLon } from "../geo";
import { LatLon, Mission, Tool } from "../types";

type Props = {
  tool: Tool;
  mission: Mission;
  analysis: MissionAnalysis;
  cursor: LatLon | undefined;
};

const TOOL_NAMES: Record<Tool, string> = {
  pan: "Pan",
  safe: "Safe area",
  start: "Start",
  path: "Path",
};

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
    case "path":
      return (
        <>
          Click to add waypoint {m.path.length + 1} · <kbd>Enter</kbd> finish · <kbd>L</kbd> close loop ·{" "}
          <kbd>⌫</kbd> remove last
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
      if (m.path.length === 0) {
        return (
          <>
            Plan the path — press <kbd>3</kbd>
          </>
        );
      }
      return <>Drag points to adjust · right-click a point to delete it</>;
  }
}

export function StatusBar({ tool, mission, analysis, cursor }: Props): ReactElement {
  const warnStart = analysis.startOutside;
  const warnSegs = analysis.outsideSegments;
  return (
    <div className="mp-status">
      <div className="mp-hint">
        {tool !== "pan" && <span className="mp-hint-tool">{TOOL_NAMES[tool]}</span>}
        <span>{hint(tool, mission)}</span>
      </div>
      <div className="mp-stats">
        {mission.safeClosed && (
          <span className="mp-stat">
            Safe <b>{formatArea(analysis.safeAreaM2)}</b>
          </span>
        )}
        {mission.start && (
          <span className="mp-stat">
            <b>{mission.path.length}</b> wp · <b>{formatDistance(analysis.routeLengthM)}</b>
            {mission.closed ? " loop" : ""}
          </span>
        )}
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
