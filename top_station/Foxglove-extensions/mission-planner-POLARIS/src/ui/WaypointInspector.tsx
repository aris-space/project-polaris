import { ReactElement, useEffect, useState } from "react";

import { IconPass, IconSurface, IconTrash, IconWait } from "./icons";
import { DEFAULT_HOLD_S, MAX_HOLD_S, Waypoint, WaypointAction } from "../types";

type Props = {
  index: number;
  waypoint: Waypoint;
  count: number;
  onChange: (action: WaypointAction, holdS: number) => void;
  onApplyAll: () => void;
  onDelete: () => void;
  onClose: () => void;
};

const ACTIONS: Array<{ id: WaypointAction; label: string; sub: string; icon: ReactElement }> = [
  { id: "pass", label: "Pass", sub: "no stopping", icon: <IconPass /> },
  { id: "wait", label: "Stop & wait", sub: "hold position", icon: <IconWait /> },
  { id: "surface", label: "Surface", sub: "rise to surface / ice", icon: <IconSurface /> },
];

const clampHold = (n: number) => Math.min(MAX_HOLD_S, Math.max(0, Math.round(n)));

/** Card for the selected waypoint: choose its action and hold time. */
export function WaypointInspector(props: Props): ReactElement {
  const { waypoint, index } = props;
  const action = waypoint.action ?? "pass";
  const hold = waypoint.holdS ?? DEFAULT_HOLD_S;
  const [text, setText] = useState(String(hold));

  // Keep the input in sync when another waypoint is selected or undo changes it.
  useEffect(() => {
    setText(String(hold));
  }, [hold, index]);

  const commitText = () => {
    const n = Number(text);
    if (Number.isFinite(n)) {
      const v = clampHold(n);
      setText(String(v));
      if (v !== hold) {
        props.onChange(action, v);
      }
    } else {
      setText(String(hold));
    }
  };

  const step = (d: number) => {
    const v = clampHold(hold + d);
    setText(String(v));
    props.onChange(action, v);
  };

  return (
    <div className="mp-inspector" onMouseDown={(e) => { e.stopPropagation(); }}>
      <div className="mp-inspector-head">
        <span className="mp-inspector-num">{index + 1}</span>
        <span className="mp-inspector-title">Waypoint {index + 1}</span>
        <span className="mp-inspector-of">of {props.count}</span>
        <button className="mp-inspector-x" onClick={props.onClose} title="Deselect (Esc)">
          ×
        </button>
      </div>

      <div className="mp-field-label">Action</div>
      <div className="mp-actions" role="radiogroup" aria-label="Waypoint action">
        {ACTIONS.map((a) => (
          <button
            key={a.id}
            role="radio"
            aria-checked={action === a.id}
            className={`mp-action mp-action--${a.id}${action === a.id ? " mp-action--active" : ""}`}
            onClick={() => {
              props.onChange(a.id, hold);
            }}
          >
            {a.icon}
            <span className="mp-action-label">{a.label}</span>
            <span className="mp-action-sub">{a.sub}</span>
          </button>
        ))}
      </div>

      {action !== "pass" && (
        <div className="mp-hold">
          <label className="mp-field-label" htmlFor="mp-hold-input">
            {action === "wait" ? "Wait time" : "Hold at surface / ice"}
          </label>
          <div className="mp-hold-row">
            <button className="mp-step" onClick={() => { step(-5); }} title="−5 s">
              −
            </button>
            <input
              id="mp-hold-input"
              className="mp-input mp-hold-input"
              inputMode="numeric"
              value={text}
              onChange={(e) => {
                setText(e.target.value.replace(/[^0-9]/g, ""));
              }}
              onBlur={commitText}
              onKeyDown={(e) => {
                if (e.key === "Enter") {
                  commitText();
                  (e.target as HTMLInputElement).blur();
                } else if (e.key === "Escape") {
                  setText(String(hold));
                  (e.target as HTMLInputElement).blur();
                }
                e.stopPropagation();
              }}
            />
            <span className="mp-hold-unit">s</span>
            <button className="mp-step" onClick={() => { step(5); }} title="+5 s">
              +
            </button>
          </div>
        </div>
      )}

      <div className="mp-inspector-foot">
        <button className="mp-link" onClick={props.onApplyAll} disabled={props.count < 2}>
          Apply to all waypoints
        </button>
        <button className="mp-link mp-link--confirm" onClick={props.onDelete}>
          <IconTrash />
          Delete
        </button>
      </div>
    </div>
  );
}
