import { ReactElement, useEffect, useRef, useState } from "react";

import { IconChevron, IconFolder, IconPlus, IconSave, IconTrash } from "./icons";
import { analyzeMission, formatDistance } from "../geo";
import { Mission, SavedMission } from "../types";

type Props = {
  name: string;
  missions: SavedMission[];
  activeId: string | null;
  dirty: boolean;
  onNameChange: (name: string) => void;
  onSave: () => void;
  onSaveCopy: () => void;
  onLoad: (id: string) => void;
  onDelete: (id: string) => void;
  onNew: () => void;
};

export const UNTITLED = "Untitled mission";

function summary(m: Mission): string {
  const parts: string[] = [];
  if (m.safeClosed) {
    parts.push("safe area");
  }
  if (m.start) {
    const a = analyzeMission(m);
    parts.push(`${m.path.length} wp`, formatDistance(a.routeLengthM));
    if (m.closed) {
      parts.push("loop");
    }
  }
  return parts.length > 0 ? parts.join(" · ") : "empty";
}

function formatWhen(ms: number): string {
  const d = new Date(ms);
  const pad = (n: number) => String(n).padStart(2, "0");
  const time = `${pad(d.getHours())}:${pad(d.getMinutes())}`;
  const today = new Date();
  if (d.toDateString() === today.toDateString()) {
    return `Today ${time}`;
  }
  return `${pad(d.getDate())}.${pad(d.getMonth() + 1)}.${d.getFullYear()} ${time}`;
}

/** Top-centre mission chip + dropdown: name, save, save copy, new, load, delete. */
export function MissionMenu(props: Props): ReactElement {
  const { name, missions, activeId, dirty } = props;
  const [open, setOpen] = useState(false);
  const [confirm, setConfirm] = useState<string | undefined>();
  const rootRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  useEffect(() => {
    if (!open) {
      return undefined;
    }
    const onDown = (e: MouseEvent) => {
      if (rootRef.current && !rootRef.current.contains(e.target as Node)) {
        setOpen(false);
        setConfirm(undefined);
      }
    };
    document.addEventListener("mousedown", onDown);
    return () => {
      document.removeEventListener("mousedown", onDown);
    };
  }, [open]);

  useEffect(() => {
    if (open && activeId == undefined) {
      inputRef.current?.focus();
      inputRef.current?.select();
    }
  }, [open, activeId]);

  const sorted = [...missions].sort((a, b) => b.savedAt - a.savedAt);
  const displayName = name.trim() || UNTITLED;

  /** Two-step confirm: first click arms, second click runs. */
  const armed = (key: string, opts: { confirm: boolean }, run: () => void) => () => {
    if (!opts.confirm || confirm === key) {
      setConfirm(undefined);
      run();
    } else {
      setConfirm(key);
    }
  };

  return (
    <div className="mp-mission" ref={rootRef}>
      <button
        className={`mp-mission-chip${open ? " mp-mission-chip--open" : ""}`}
        onClick={() => {
          setOpen((o) => !o);
          setConfirm(undefined);
        }}
        title="Missions — save, load and switch (⌘/Ctrl S saves)"
      >
        <IconFolder />
        <span className="mp-mission-name">{displayName}</span>
        {dirty && <span className="mp-unsaved" title="Unsaved changes" />}
        <IconChevron />
      </button>

      {open && (
        <div className="mp-mission-panel">
          <div className="mp-mission-section">
            <label className="mp-field-label" htmlFor="mp-mission-name">
              Mission name
            </label>
            <div className="mp-name-row">
              <input
                id="mp-mission-name"
                ref={inputRef}
                className="mp-input"
                value={name}
                placeholder={UNTITLED}
                maxLength={80}
                onChange={(e) => {
                  props.onNameChange(e.target.value);
                }}
                onKeyDown={(e) => {
                  if (e.key === "Enter") {
                    props.onSave();
                  } else if (e.key === "Escape") {
                    setOpen(false);
                  }
                  e.stopPropagation();
                }}
              />
              <button className="mp-primary" onClick={props.onSave} disabled={!dirty && activeId != undefined}>
                <IconSave />
                {activeId != undefined ? "Save" : "Save new"}
              </button>
            </div>
            <div className="mp-mission-actions">
              <span className="mp-mission-state">
                {activeId == undefined ? "Not saved yet" : dirty ? "Unsaved changes" : "All changes saved"}
              </span>
              {activeId != undefined && (
                <button className="mp-link" onClick={props.onSaveCopy}>
                  Save as copy
                </button>
              )}
              <button
                className={`mp-link${confirm === "new" ? " mp-link--confirm" : ""}`}
                onClick={armed("new", { confirm: dirty }, () => {
                  props.onNew();
                  setOpen(false);
                })}
              >
                <IconPlus />
                {confirm === "new" ? "Discard changes?" : "New mission"}
              </button>
            </div>
          </div>

          <div className="mp-menu-title">Saved missions ({missions.length})</div>
          <div className="mp-mission-list">
            {sorted.length === 0 && (
              <div className="mp-empty">No saved missions yet. Name this mission and press Save.</div>
            )}
            {sorted.map((m) => {
              const active = m.id === activeId;
              const loadKey = `load:${m.id}`;
              const delKey = `del:${m.id}`;
              return (
                <div key={m.id} className={`mp-mission-row${active ? " mp-mission-row--active" : ""}`}>
                  <button
                    className="mp-mission-load"
                    onClick={armed(loadKey, { confirm: dirty }, () => {
                      props.onLoad(m.id);
                      setOpen(false);
                    })}
                    title={active ? "Reload this mission (discards unsaved changes)" : "Load this mission"}
                  >
                    <span className="mp-mission-row-name">
                      {active && <span className="mp-active-dot" />}
                      {m.name}
                    </span>
                    <span className="mp-mission-row-meta">
                      {confirm === loadKey
                        ? "Unsaved changes will be lost — click again to load"
                        : `${summary(m.mission)} · ${formatWhen(m.savedAt)}`}
                    </span>
                  </button>
                  <button
                    className={`mp-icon-btn${confirm === delKey ? " mp-icon-btn--confirm" : ""}`}
                    onClick={armed(delKey, { confirm: true }, () => {
                      props.onDelete(m.id);
                    })}
                    title={confirm === delKey ? "Click again to delete" : "Delete saved mission"}
                  >
                    {confirm === delKey ? "Delete?" : <IconTrash />}
                  </button>
                </div>
              );
            })}
          </div>
        </div>
      )}
    </div>
  );
}
