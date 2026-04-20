import {
  Immutable,
  MessageEvent,
  PanelExtensionContext,
  SettingsTreeAction,
  SettingsTreeNodes,
  Topic,
} from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import {
  readSharedRecorderSettings,
  subscribeSharedRecorderSettings,
  updateSharedRecorderSettings,
} from "./sharedRecorderSettings";

type RecorderMetadata = {
  name: string;
  testname: string;
  location: string;
};

type RecorderStatus = {
  is_recording: boolean;
  active_event: string | null;
  recording_id: string | null;
  output_path: string | null;
  base_output_dir?: string | null;
  started_at: string | null;
  metadata: RecorderMetadata;
  last_error: string | null;
  recording_options?: RecordingOptions;
};

type RecorderCommand = {
  command:
    | "start_recording"
    | "stop_recording"
    | "add_instant_event"
    | "start_event"
    | "stop_event"
    | "set_metadata";
  metadata?: RecorderMetadata;
  base_output_dir?: string;
  Event?: string;
  recording_options?: RecordingOptions;
  source: "foxglove-panel";
  timestamp: string;
};

type RecordingOptions = {
  mode: "all" | "selection" | "exclude_selection";
  include_topics: string[];
  exclude_topics: string[];
};

const NODE_DEFAULT_OUTPUT_DIR_PLACEHOLDER = "__USE_NODE_DEFAULT_OUTPUT_DIR__";
const DEFAULT_COMMAND_TOPIC = "/polaris/recorder/command";
const DEFAULT_STATUS_TOPIC = "/polaris/recorder/status";

type PanelConfig = {
  recordMode: "all" | "selection" | "exclude_selection";
  includeTopics: string[];
  excludeTopics: string[];
  touchscreenMode: boolean;
};

const DEFAULT_CONFIG: PanelConfig = {
  recordMode: "all",
  includeTopics: [],
  excludeTopics: [],
  touchscreenMode: false,
};

const DEFAULT_METADATA: RecorderMetadata = {
  name: "",
  testname: "",
  location: "",
};

const EMPTY_STATUS: RecorderStatus = {
  is_recording: false,
  active_event: null,
  recording_id: null,
  output_path: null,
  started_at: null,
  metadata: DEFAULT_METADATA,
  last_error: null,
};

function safeJsonParse(raw: unknown): unknown {
  if (typeof raw !== "string") {
    return undefined;
  }
  try {
    return JSON.parse(raw);
  } catch {
    return undefined;
  }
}

function parseStatusMessage(msg: unknown): RecorderStatus | undefined {
  if (typeof msg !== "object" || msg == undefined) {
    return undefined;
  }

  const asRecord = msg as Record<string, unknown>;
  const parsed = safeJsonParse(asRecord.data);
  if (typeof parsed !== "object" || parsed == undefined) {
    return undefined;
  }

  const status = parsed as Record<string, unknown>;
  const options = (status.recording_options as Partial<RecordingOptions>) ?? {};

  return {
    is_recording: Boolean(status.is_recording),
    active_event: typeof status.active_event === "string" ? status.active_event : null,
    recording_id: typeof status.recording_id === "string" ? status.recording_id : null,
    output_path: typeof status.output_path === "string" ? status.output_path : null,
    base_output_dir: typeof status.base_output_dir === "string" ? status.base_output_dir : null,
    started_at: typeof status.started_at === "string" ? status.started_at : null,
    metadata: {
      name: ((status.metadata as Partial<RecorderMetadata> | undefined)?.name ?? "") as string,
      testname: ((status.metadata as Partial<RecorderMetadata> | undefined)?.testname ?? "") as string,
      location: ((status.metadata as Partial<RecorderMetadata> | undefined)?.location ?? "") as string,
    },
    last_error: typeof status.last_error === "string" ? status.last_error : null,
    recording_options: {
      mode: parseRecordMode(options.mode),
      include_topics: Array.isArray(options.include_topics)
        ? options.include_topics.filter((v): v is string => typeof v === "string")
        : [],
      exclude_topics: Array.isArray(options.exclude_topics)
        ? options.exclude_topics.filter((v): v is string => typeof v === "string")
        : [],
    },
  };
}

function sanitizeFileToken(value: string): string {
  return value.replace(/[^A-Za-z0-9_-]/g, "");
}

function sanitizeEventToken(value: string): string {
  return value.replace(/[^A-Za-z0-9_\- ]/g, "");
}

function normalizeTopicName(value: string): string {
  const cleaned = value.trim();
  if (!cleaned) {
    return "";
  }
  return cleaned.startsWith("/") ? cleaned : `/${cleaned}`;
}

function parseRecordMode(mode: unknown): RecordingOptions["mode"] {
  if (mode === "selection" || mode === "exclude_selection") {
    return mode;
  }
  if (mode === "list") {
    return "selection";
  }
  return "all";
}

function RecorderPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [messages, setMessages] = useState<undefined | Immutable<MessageEvent[]>>();
  const [topics, setTopics] = useState<undefined | Immutable<Topic[]>>();
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [status, setStatus] = useState<RecorderStatus>(EMPTY_STATUS);
  const [metadata, setMetadata] = useState<RecorderMetadata>(DEFAULT_METADATA);
  const [instantEventName, setInstantEventName] = useState<string>("");
  const [longEventName, setLongEventName] = useState<string>("");
  const [panelError, setPanelError] = useState<string | null>(null);
  const [stopConfirmPending, setStopConfirmPending] = useState<boolean>(false);
  const [isCompact, setIsCompact] = useState<boolean>(() => context.panelElement.clientWidth < 440);
  const stopConfirmTimerRef = useRef<ReturnType<typeof setTimeout> | undefined>(undefined);
  const initializedFromStatus = useRef(false);
  const [draftIncludeTopic, setDraftIncludeTopic] = useState<string>("");
  const [draftExcludeTopic, setDraftExcludeTopic] = useState<string>("");
  const [includeAddFieldVersion, setIncludeAddFieldVersion] = useState(0);
  const [excludeAddFieldVersion, setExcludeAddFieldVersion] = useState(0);

  const [config, setConfig] = useState<PanelConfig>(() => {
    const shared = readSharedRecorderSettings();
    return {
      recordMode: parseRecordMode(shared.recordMode ?? DEFAULT_CONFIG.recordMode),
      includeTopics: shared.includeTopics ?? DEFAULT_CONFIG.includeTopics,
      excludeTopics: shared.excludeTopics ?? DEFAULT_CONFIG.excludeTopics,
      touchscreenMode: shared.touchscreenMode ?? DEFAULT_CONFIG.touchscreenMode,
    };
  });

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      setMessages(renderState.currentFrame);
      setTopics(renderState.topics);
    };

    context.watch("currentFrame");
    context.watch("topics");
  }, [context]);

  const availableTopicNames = useMemo(() => (topics ?? []).map((topic) => topic.name), [topics]);

  const normalizeTopicList = useCallback((list: string[]) => {
    return list
      .map((topic) => normalizeTopicName(topic))
      .filter((topic, index, arr) => topic !== "" && arr.indexOf(topic) === index);
  }, []);

  useEffect(() => {
    const updateCompactMode = () => {
      const width = context.panelElement.clientWidth;
      setIsCompact(width < 440);
    };

    updateCompactMode();

    const observer = new ResizeObserver(updateCompactMode);
    observer.observe(context.panelElement);

    return () => {
      observer.disconnect();
    };
  }, [context]);

  useEffect(() => {
    updateSharedRecorderSettings({
      recordMode: config.recordMode,
      includeTopics: config.includeTopics,
      excludeTopics: config.excludeTopics,
      touchscreenMode: config.touchscreenMode,
    });
  }, [config.excludeTopics, config.includeTopics, config.recordMode, config.touchscreenMode]);

  useEffect(() => {
    return subscribeSharedRecorderSettings((shared) => {
      setConfig((prev) => {
        const next: PanelConfig = {
          recordMode: parseRecordMode(shared.recordMode ?? prev.recordMode),
          includeTopics: shared.includeTopics ?? prev.includeTopics,
          excludeTopics: shared.excludeTopics ?? prev.excludeTopics,
          touchscreenMode: shared.touchscreenMode ?? prev.touchscreenMode,
        };

        const includeUnchanged =
          next.includeTopics.length === prev.includeTopics.length &&
          next.includeTopics.every((topic, index) => topic === prev.includeTopics[index]);
        const excludeUnchanged =
          next.excludeTopics.length === prev.excludeTopics.length &&
          next.excludeTopics.every((topic, index) => topic === prev.excludeTopics[index]);

        if (
          next.recordMode === prev.recordMode &&
          next.touchscreenMode === prev.touchscreenMode &&
          includeUnchanged &&
          excludeUnchanged
        ) {
          return prev;
        }
        return next;
      });
    });
  }, []);

  const handleSettingsAction = useCallback(
    (action: SettingsTreeAction) => {
      if (action.action === "update") {
        if (!Array.isArray(action.payload.path)) {
          return;
        }
        const path = action.payload.path;
        const value = String(action.payload.value ?? "").trim();

        if (path[0] === "metadata") {
          const key =
            typeof path[1] === "string"
              ? path[1]
              : path[1] === "fields" && typeof path[2] === "string"
                ? path[2]
                : undefined;

          if (key === "missionName") {
            setMetadata((prev) => ({ ...prev, name: sanitizeFileToken(value) }));
          }
          if (key === "location") {
            setMetadata((prev) => ({ ...prev, location: sanitizeFileToken(value) }));
          }
          return;
        }

        if (path[0] === "topicLists" && path[1] === "recordMode") {
          const mode = parseRecordMode(value);
          setConfig((prev) => ({ ...prev, recordMode: mode }));
          return;
        }

        if (path[0] === "display" && path[1] === "touchscreenMode") {
          const nextValue = value === "true";
          setConfig((prev) => ({ ...prev, touchscreenMode: nextValue }));
          return;
        }

        const includeOrExclude = path[1] === "include" || path[1] === "exclude";
        const isAddTopicField =
          includeOrExclude &&
          ((typeof path[2] === "string" && path[2].startsWith("topicPath-add-")) ||
            (path[2] === "fields" && typeof path[3] === "string" && path[3].startsWith("topicPath-add-")));

        if (path[0] === "topicLists" && isAddTopicField) {
          if (path[1] === "include") {
            setDraftIncludeTopic(value);
          } else {
            setDraftExcludeTopic(value);
          }

          const normalized = normalizeTopicName(value);
          if (!normalized) {
            return;
          }

          const isKnownTopic = availableTopicNames.includes(normalized);
          if (!isKnownTopic) {
            return;
          }

          const listKey = path[1] === "include" ? "includeTopics" : "excludeTopics";
          setConfig((prev) => ({
            ...prev,
            [listKey]: normalizeTopicList([...prev[listKey], normalized]),
          }));

          if (path[1] === "include") {
            setDraftIncludeTopic("");
            setIncludeAddFieldVersion((prev) => prev + 1);
          } else {
            setDraftExcludeTopic("");
            setExcludeAddFieldVersion((prev) => prev + 1);
          }
          return;
        }

        const topicFieldKey =
          typeof path[2] === "string" && path[2].startsWith("topic-")
            ? path[2]
            : path[2] === "fields" && typeof path[3] === "string" && path[3].startsWith("topic-")
              ? path[3]
              : null;

        if (path[0] === "topicLists" && includeOrExclude && topicFieldKey != null) {
          const index = Number(topicFieldKey.replace("topic-", ""));
          if (!Number.isFinite(index) || index < 0) {
            return;
          }

          const listKey = path[1] === "include" ? "includeTopics" : "excludeTopics";
          setConfig((prev) => {
            const next = [...prev[listKey]];
            if (index >= next.length) {
              return prev;
            }

            const normalized = normalizeTopicName(value);
            if (!normalized) {
              return {
                ...prev,
                [listKey]: next.filter((_, itemIndex) => itemIndex !== index),
              };
            }

            next[index] = normalized;
            return {
              ...prev,
              [listKey]: normalizeTopicList(next),
            };
          });
          return;
        }
        return;
      }

      const { id, path } = action.payload;
      if (!Array.isArray(path)) {
        return;
      }

      if (path[0] === "topicLists" && (path[1] === "include" || path[1] === "exclude")) {
        const listKey = path[1] === "include" ? "includeTopics" : "excludeTopics";
        if (id === "clear") {
          setConfig((prev) => ({
            ...prev,
            [listKey]: [],
          }));
        }
      }
    },
    [availableTopicNames, normalizeTopicList],
  );

  const updateSettingsTree = useCallback(() => {
    const nodes: SettingsTreeNodes = {
      metadata: {
        label: "Metadata",
        defaultExpansionState: "expanded",
        fields: {
          missionName: {
            label: "Mission name",
            input: "string",
            value: metadata.name,
          },
          location: {
            label: "Location",
            input: "string",
            value: metadata.location,
          },
        },
      },
      topicLists: {
        label: "Topic Lists",
        defaultExpansionState: "collapsed",
        fields: {
          recordMode: {
            label: "Mode",
            input: "select",
            options: [
              { label: "All topics", value: "all" },
              { label: "Record selection", value: "selection" },
              { label: "Exclude selection", value: "exclude_selection" },
            ],
            value: config.recordMode,
          },
        },
        children: {
          include: {
            label: `Record Selection List (${config.includeTopics.length})`,
            defaultExpansionState: "collapsed",
            actions: [{ id: "clear", type: "action", label: "Clear list" }],
            fields: {
              [`topicPath-add-${includeAddFieldVersion}`]: {
                label: "[ADD] New topic (select once)",
                input: "messagepath",
                validTopics: availableTopicNames,
                value: draftIncludeTopic,
              },
              ...Object.fromEntries(
                config.includeTopics.map((topic, index) => [
                  `topic-${index}`,
                  {
                    label: " ",
                    input: "messagepath",
                    validTopics: availableTopicNames,
                    value: topic,
                  },
                ]),
              ),
            },
          },
          exclude: {
            label: `Exclude Selection List (${config.excludeTopics.length})`,
            defaultExpansionState: "collapsed",
            actions: [{ id: "clear", type: "action", label: "Clear list" }],
            fields: {
              [`topicPath-add-${excludeAddFieldVersion}`]: {
                label: "[ADD] New topic (select once)",
                input: "messagepath",
                validTopics: availableTopicNames,
                value: draftExcludeTopic,
              },
              ...Object.fromEntries(
                config.excludeTopics.map((topic, index) => [
                  `topic-${index}`,
                  {
                    label: " ",
                    input: "messagepath",
                    validTopics: availableTopicNames,
                    value: topic,
                  },
                ]),
              ),
            },
          },
        },
      },
      display: {
        label: "Display",
        defaultExpansionState: "collapsed",
        fields: {
          touchscreenMode: {
            label: "Touchscreen mode",
            input: "boolean",
            value: config.touchscreenMode,
          },
        },
      },
    };

    context.updatePanelSettingsEditor({
      actionHandler: handleSettingsAction,
      nodes,
    });
  }, [
    availableTopicNames,
    config.excludeTopics,
    config.includeTopics,
    config.recordMode,
    config.touchscreenMode,
    context,
    draftExcludeTopic,
    draftIncludeTopic,
    excludeAddFieldVersion,
    handleSettingsAction,
    includeAddFieldVersion,
    metadata.location,
    metadata.name,
  ]);

  useLayoutEffect(() => {
    updateSettingsTree();
  }, [updateSettingsTree]);

  useEffect(() => {
    context.subscribe([{ topic: DEFAULT_STATUS_TOPIC }]);
    return () => {
      context.unsubscribeAll();
    };
  }, [context]);

  useEffect(() => {
    if (!status.is_recording) {
      setStopConfirmPending(false);
    }
  }, [status.is_recording]);

  useEffect(() => {
    return () => {
      if (stopConfirmTimerRef.current != undefined) {
        clearTimeout(stopConfirmTimerRef.current);
      }
    };
  }, []);

  useEffect(() => {
    const latestMessage = messages?.[messages.length - 1]?.message;
    const parsed = parseStatusMessage(latestMessage);
    if (parsed != undefined) {
      setStatus(parsed);
      setMetadata((prev) => ({
        name: prev.name || parsed.metadata.name,
        testname: prev.testname || parsed.metadata.testname,
        location: prev.location || parsed.metadata.location,
      }));

      if (!initializedFromStatus.current && parsed.recording_options) {
        setConfig((prev) => ({
          ...prev,
          recordMode: parseRecordMode(parsed.recording_options?.mode),
          includeTopics: parsed.recording_options?.include_topics ?? prev.includeTopics,
          excludeTopics: parsed.recording_options?.exclude_topics ?? prev.excludeTopics,
        }));
        initializedFromStatus.current = true;
      }
    }
  }, [messages]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const publishCommand = useCallback(
    (command: RecorderCommand["command"], eventName?: string) => {
      const payload: RecorderCommand = {
        command,
        source: "foxglove-panel",
        timestamp: new Date().toISOString(),
        metadata,
        base_output_dir: NODE_DEFAULT_OUTPUT_DIR_PLACEHOLDER,
        recording_options: {
          mode: config.recordMode,
          include_topics: config.includeTopics,
          exclude_topics: config.excludeTopics,
        },
      };

      if (eventName != undefined) {
        const cleaned = eventName.trim();
        if (cleaned) {
          payload.Event = cleaned;
        }
      }

      try {
        context.advertise?.(DEFAULT_COMMAND_TOPIC, "std_msgs/String");
        context.publish?.(DEFAULT_COMMAND_TOPIC, { data: JSON.stringify(payload) });
        setPanelError(null);
      } catch (error) {
        setPanelError(error instanceof Error ? error.message : "Failed to publish command.");
      }
    },
    [config.excludeTopics, config.includeTopics, config.recordMode, context, metadata],
  );

  const recordingIndicator = useMemo(() => {
    if (status.is_recording) {
      return {
        label: "RECORDING",
        color: "#f34a4a",
        glow: "rgba(243, 74, 74, 0.35)",
      };
    }

    return {
      label: "IDLE",
      color: "#8f9aa4",
      glow: "rgba(143, 154, 164, 0.22)",
    };
  }, [status.is_recording]);

  const touchScale = config.touchscreenMode ? 1.35 : 1;
  const touchPaddingScale = config.touchscreenMode ? 1.5 : 1;
  const touchRowAlignItems = config.touchscreenMode ? "stretch" : "center";

  const palette = {
    bgTop: "#0c1824",
    bgBottom: "#141f31",
    panelCard: "rgba(255, 255, 255, 0.06)",
    panelCardBorder: "rgba(255, 255, 255, 0.12)",
    textPrimary: "#eaf2ff",
    textSecondary: "#afbccf",
    buttonSuccess: "#22a06b",
    buttonSuccessHover: "#1b8a5c",
    buttonDanger: "#d84a4a",
    buttonDangerHover: "#b63d3d",
    buttonAccent: "#f28c28",
    buttonAccentHover: "#d97818",
    buttonInfo: "#2f80ff",
    buttonInfoHover: "#276fe0",
    buttonGhost: "rgba(242, 140, 40, 0.18)",
    buttonGhostHover: "rgba(242, 140, 40, 0.28)",
    inputBg: "rgba(255, 255, 255, 0.08)",
    inputBorder: "rgba(255, 255, 255, 0.18)",
    warning: "#ffb347",
    error: "#ff8f8f",
  };

  const cardStyle = {
    background: palette.panelCard,
    border: `1px solid ${palette.panelCardBorder}`,
    borderRadius: isCompact ? 8 : 10,
    padding: isCompact ? 4 : config.touchscreenMode ? 8 : 6,
  };

  const inputStyle = {
    background: palette.inputBg,
    border: `1px solid ${palette.inputBorder}`,
    color: palette.textPrimary,
    borderRadius: 7,
    padding: isCompact
      ? `${Math.round(3 * touchPaddingScale)}px ${Math.round(6 * touchPaddingScale)}px`
      : config.touchscreenMode
        ? `${Math.round(5 * touchPaddingScale)}px ${Math.round(8 * touchPaddingScale)}px`
        : "4px 7px",
    fontSize: isCompact ? 11 : config.touchscreenMode ? 13 : 12,
    lineHeight: 1.2,
    outline: "none",
    width: "100%",
    boxSizing: "border-box" as const,
  };

  const fieldRowStyle = {
    display: "grid",
    gridTemplateColumns: isCompact ? "1fr" : "88px 1fr",
    alignItems: isCompact ? "start" : "center",
    gap: isCompact ? 2 : config.touchscreenMode ? 6 : 4,
  };

  const fieldLabelStyle = {
    color: palette.textSecondary,
    fontSize: isCompact ? 10 : 11,
    fontWeight: 600,
    letterSpacing: "0.02em",
  };

  const sectionTitleStyle = {
    fontSize: isCompact ? 10 : 11,
    color: palette.textSecondary,
    textTransform: "uppercase" as const,
    letterSpacing: "0.03em",
    lineHeight: 1.2,
  };

  const compactTextStyle = {
    fontSize: isCompact ? 10 : config.touchscreenMode ? 12 : 11,
    color: palette.textSecondary,
    lineHeight: 1.2,
  };

  const cardGap = isCompact ? 3 : config.touchscreenMode ? 6 : 4;

  function buttonStyle(kind: "primary" | "danger" | "ghost" | "accent") {
    const base = {
      border: "none",
      borderRadius: 7,
      color: "#ffffff",
      cursor: "pointer",
      fontWeight: 700,
      padding: isCompact
        ? `${Math.round(4 * touchPaddingScale)}px ${Math.round(7 * touchPaddingScale)}px`
        : config.touchscreenMode
          ? `${Math.round(8 * touchPaddingScale)}px ${Math.round(12 * touchPaddingScale)}px`
          : "5px 8px",
      fontSize: isCompact ? 11 : config.touchscreenMode ? 13 : 12,
      lineHeight: 1.2,
      letterSpacing: "0.01em",
      transition: "background-color 0.15s ease",
      width: "100%",
      height: config.touchscreenMode ? "100%" : undefined,
      maxWidth: isCompact ? undefined : config.touchscreenMode ? 216 : 160,
      justifySelf: isCompact ? ("stretch" as const) : ("start" as const),
      alignSelf: config.touchscreenMode ? ("stretch" as const) : undefined,
    };

    if (kind === "primary") {
      return { ...base, background: palette.buttonSuccess };
    }
    if (kind === "danger") {
      return { ...base, background: palette.buttonDanger };
    }
    if (kind === "accent") {
      return { ...base, background: palette.buttonInfo };
    }
    return {
      ...base,
      background: palette.buttonGhost,
      boxShadow: "0 0 0 1px rgba(242, 140, 40, 0.35) inset",
    };
  }

  const isRecording = status.is_recording;
  const hasActiveLongEvent = status.active_event !== null;

  const recordingButtonKind = isRecording ? "danger" : "primary";
  const recordingButtonLabel = isRecording
    ? stopConfirmPending
      ? "■ Confirm Stop"
      : "■ Stop Rec"
    : "● Start Rec";
  const missingSettingsMetadataFields = !isRecording
    ? [
        metadata.name.trim() === "" ? "mission name" : null,
        metadata.location.trim() === "" ? "location" : null,
      ].filter((field): field is string => field != null)
    : [];
  const missingPanelMetadataFields = !isRecording
    ? [metadata.testname.trim() === "" ? "test name" : null].filter((field): field is string => field != null)
    : [];
  const startRecordingDisabled =
    !isRecording && (missingSettingsMetadataFields.length > 0 || missingPanelMetadataFields.length > 0);

  const longEventButtonKind = hasActiveLongEvent ? "danger" : "accent";
  const longEventButtonLabel = hasActiveLongEvent ? "Stop Long Event" : "Start Long Event";

  const instantEventButtonDisabled = false;
  const longEventButtonDisabled = false;

  return (
    <div
      style={{
        padding: isCompact ? 4 : config.touchscreenMode ? 6 : 6,
        fontFamily: "Avenir Next, Segoe UI, Helvetica Neue, Arial, sans-serif",
        color: palette.textPrimary,
        display: "grid",
        gap: isCompact ? 4 : config.touchscreenMode ? 8 : 6,
        background: `linear-gradient(150deg, ${palette.bgTop} 0%, ${palette.bgBottom} 100%)`,
        borderRadius: isCompact ? 8 : 12,
        height: "100%",
        boxSizing: "border-box",
        overflowY: "auto",
      }}
    >
      <div
        style={{
          ...cardStyle,
          display: "grid",
          gap: cardGap,
          background: isRecording ? "rgba(34, 160, 107, 0.08)" : "rgba(34, 160, 107, 0.05)",
          borderColor: isRecording ? "rgba(34, 160, 107, 0.38)" : palette.panelCardBorder,
        }}
      >
        <div style={{ display: "flex", alignItems: touchRowAlignItems, gap: isCompact ? 6 : config.touchscreenMode ? 10 : 8 }}>
          <button
            type="button"
            disabled={startRecordingDisabled}
            style={{
              ...buttonStyle(recordingButtonKind),
              width: "auto",
              minWidth: isCompact ? 88 : config.touchscreenMode ? 132 : 104,
              maxWidth: "none",
              boxShadow: isRecording
                ? stopConfirmPending
                  ? "0 0 0 1px rgba(255, 179, 71, 0.55), 0 0 0 4px rgba(255, 179, 71, 0.16)"
                  : "0 0 0 1px rgba(243, 74, 74, 0.45), 0 0 0 4px rgba(243, 74, 74, 0.14)"
                : "0 0 0 1px rgba(34, 160, 107, 0.38), 0 0 0 4px rgba(34, 160, 107, 0.10)",
              opacity: startRecordingDisabled ? 0.55 : 1,
              cursor: startRecordingDisabled ? "not-allowed" : "pointer",
              textTransform: "uppercase",
            }}
            onMouseOver={(e) => {
              if (!startRecordingDisabled) {
                e.currentTarget.style.backgroundColor =
                  recordingButtonKind === "danger" ? palette.buttonDangerHover : palette.buttonSuccessHover;
              }
            }}
            onMouseOut={(e) => {
              e.currentTarget.style.backgroundColor =
                recordingButtonKind === "danger" ? palette.buttonDanger : palette.buttonSuccess;
            }}
            onClick={() => {
              if (startRecordingDisabled) {
                if (missingSettingsMetadataFields.length > 0) {
                  setPanelError(
                    `Cannot start recording. Missing in settings: ${missingSettingsMetadataFields.join(", ")}.`,
                  );
                } else {
                  setPanelError("Cannot start recording. Test name is required.");
                }
                return;
              }

              if (isRecording && !stopConfirmPending) {
                setStopConfirmPending(true);
                setPanelError("Click Stop Rec again to confirm.");
                if (stopConfirmTimerRef.current != undefined) {
                  clearTimeout(stopConfirmTimerRef.current);
                }
                stopConfirmTimerRef.current = setTimeout(() => {
                  setStopConfirmPending(false);
                }, 3000);
                return;
              }

              if (stopConfirmTimerRef.current != undefined) {
                clearTimeout(stopConfirmTimerRef.current);
                stopConfirmTimerRef.current = undefined;
              }

              setStopConfirmPending(false);
              publishCommand(isRecording ? "stop_recording" : "start_recording");
            }}
          >
            {recordingButtonLabel}
          </button>
          <input
            placeholder="e.g. tank-validation_03"
            style={{
              ...inputStyle,
              width: "100%",
              minWidth: 0,
              flex: "1 1 auto",
            }}
            value={metadata.testname}
            onChange={(e) =>
              setMetadata((prev) => ({
                ...prev,
                testname: sanitizeFileToken(e.target.value),
              }))
            }
          />
        </div>

        {!isRecording && missingSettingsMetadataFields.length > 0 ? (
          <div
            style={{
              color: palette.warning,
              fontSize: isCompact ? 10 : Math.round(10.5 * touchScale),
              lineHeight: 1.1,
              marginTop: 1,
            }}
          >
            Missing in settings: {missingSettingsMetadataFields.join(", ")}.
          </div>
        ) : undefined}
      </div>

      <div
        style={{
          ...cardStyle,
          display: "grid",
          gap: cardGap,
          background: "rgba(242, 140, 40, 0.08)",
          borderColor: "rgba(242, 140, 40, 0.30)",
        }}
      >
        <div style={{ display: "flex", alignItems: touchRowAlignItems, gap: isCompact ? 6 : config.touchscreenMode ? 10 : 8 }}>
          <button
            type="button"
            disabled={instantEventButtonDisabled}
            style={{
              ...buttonStyle("ghost"),
              width: "auto",
              minWidth: isCompact ? 120 : config.touchscreenMode ? 164 : 140,
              maxWidth: "none",
              opacity: instantEventButtonDisabled ? 0.55 : 1,
              cursor: instantEventButtonDisabled ? "not-allowed" : "pointer",
              textTransform: "uppercase",
            }}
            onMouseOver={(e) => {
              if (!instantEventButtonDisabled) {
                e.currentTarget.style.backgroundColor = palette.buttonGhostHover;
              }
            }}
            onMouseOut={(e) => (e.currentTarget.style.backgroundColor = palette.buttonGhost)}
            onClick={() => publishCommand("add_instant_event", instantEventName)}
          >
            Add Instant Event
          </button>
          <input
            placeholder="optional label"
            style={{
              ...inputStyle,
              width: isCompact ? "100%" : config.touchscreenMode ? "clamp(164px, 58%, 280px)" : "clamp(140px, 58%, 260px)",
              flex: "1 1 auto",
              minWidth: 0,
              borderColor: "rgba(242, 140, 40, 0.32)",
            }}
            value={instantEventName}
            onChange={(e) => setInstantEventName(sanitizeEventToken(e.target.value))}
          />
        </div>
      </div>

      <div
        style={{
          ...cardStyle,
          display: "grid",
          gap: cardGap,
          background: hasActiveLongEvent ? "rgba(215, 61, 61, 0.08)" : "rgba(47, 128, 255, 0.07)",
          borderColor: hasActiveLongEvent ? "rgba(215, 61, 61, 0.34)" : "rgba(47, 128, 255, 0.28)",
        }}
      >
        <div style={{ display: "flex", alignItems: touchRowAlignItems, gap: isCompact ? 6 : config.touchscreenMode ? 10 : 8 }}>
          <button
            type="button"
            disabled={longEventButtonDisabled}
            style={{
              ...buttonStyle(longEventButtonKind),
              width: "auto",
              minWidth: isCompact ? 88 : config.touchscreenMode ? 132 : 104,
              maxWidth: "none",
              opacity: longEventButtonDisabled ? 0.55 : 1,
              cursor: longEventButtonDisabled ? "not-allowed" : "pointer",
            }}
            onMouseOver={(e) => {
              if (!longEventButtonDisabled) {
                e.currentTarget.style.backgroundColor =
                  longEventButtonKind === "danger" ? palette.buttonDangerHover : palette.buttonInfoHover;
              }
            }}
            onMouseOut={(e) => {
              e.currentTarget.style.backgroundColor =
                longEventButtonKind === "danger" ? palette.buttonDanger : palette.buttonInfo;
            }}
            onClick={() =>
              publishCommand(
                hasActiveLongEvent ? "stop_event" : "start_event",
                hasActiveLongEvent ? status.active_event ?? undefined : longEventName,
              )
            }
          >
            {longEventButtonLabel}
          </button>
          <input
            placeholder="optional label"
            style={{
              ...inputStyle,
              width: isCompact ? "100%" : config.touchscreenMode ? "clamp(164px, 58%, 280px)" : "clamp(140px, 58%, 260px)",
              flex: "1 1 auto",
              minWidth: 0,
              borderColor: hasActiveLongEvent ? "rgba(215, 61, 61, 0.32)" : "rgba(47, 128, 255, 0.28)",
            }}
            value={longEventName}
            onChange={(e) => setLongEventName(sanitizeEventToken(e.target.value))}
          />
        </div>
      </div>

      {status.last_error ? (
        <div
          style={{
            background: "rgba(255, 84, 84, 0.14)",
            border: "1px solid rgba(255, 84, 84, 0.4)",
            borderRadius: 7,
            color: palette.error,
            padding: isCompact ? "5px 7px" : "6px 8px",
            fontSize: isCompact ? 10 : 11,
            lineHeight: 1.2,
          }}
        >
          Node error: {status.last_error}
        </div>
      ) : undefined}

      {panelError ? (
        <div
          style={{
            background: "rgba(255, 179, 71, 0.12)",
            border: "1px solid rgba(255, 179, 71, 0.4)",
            borderRadius: 7,
            color: palette.warning,
            padding: isCompact ? "5px 7px" : "6px 8px",
            fontSize: isCompact ? 10 : 11,
            lineHeight: 1.2,
          }}
        >
          Panel error: {panelError}
        </div>
      ) : undefined}
    </div>
  );
}

export function initRecorderActivePanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<RecorderPanel context={context} />);

  return () => {
    root.unmount();
  };
}
