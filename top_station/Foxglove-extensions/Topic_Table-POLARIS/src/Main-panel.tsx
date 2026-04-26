import {
  Immutable,
  PanelExtensionContext,
  SettingsTreeAction,
  SettingsTreeNodeAction,
  SettingsTreeNodes,
  Topic,
} from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useMemo, useState } from "react";
import { createRoot } from "react-dom/client";

interface TopicConfig {
  id: string;
  topicPath: string;
  customLabel: string;
}

interface PanelSettings {
  trackedTopics: TopicConfig[];
  updateFrequency: number; // Hz (0 = every frame)
  leftColumnWidth: number; // Percentage width of left column (0-100)
}

function splitTopicPath(
  topicPath: string,
  availableTopicNames: readonly string[],
): { topicName: string; fieldPath: string | undefined } {
  const trimmed = topicPath.trim();
  if (!trimmed) {
    return { topicName: "", fieldPath: undefined };
  }

  const bestTopicMatch = [...availableTopicNames]
    .sort((a, b) => b.length - a.length)
    .find((candidate) => trimmed === candidate || trimmed.startsWith(`${candidate}.`));

  if (bestTopicMatch != undefined) {
    if (trimmed === bestTopicMatch) {
      return { topicName: bestTopicMatch, fieldPath: undefined };
    }
    return {
      topicName: bestTopicMatch,
      fieldPath: trimmed.slice(bestTopicMatch.length + 1),
    };
  }

  const fallbackDotIndex = trimmed.indexOf(".");
  if (fallbackDotIndex > 0) {
    return {
      topicName: trimmed.slice(0, fallbackDotIndex),
      fieldPath: trimmed.slice(fallbackDotIndex + 1),
    };
  }

  return { topicName: trimmed, fieldPath: undefined };
}

function getNestedValue(message: unknown, fieldPath: string | undefined): unknown {
  if (!fieldPath) {
    return message;
  }

  const tokens: Array<string | number> = [];
  const tokenRegex = /([^.[\]]+)|\[(\d+)\]/g;
  let match: RegExpExecArray | null;

  while ((match = tokenRegex.exec(fieldPath)) != undefined) {
    if (match[1] != undefined) {
      tokens.push(match[1]);
    } else if (match[2] != undefined) {
      tokens.push(Number(match[2]));
    }
  }

  if (tokens.length === 0) {
    return undefined;
  }

  let current: unknown = message;
  for (const token of tokens) {
    if (typeof token === "number") {
      if (!Array.isArray(current) || token < 0 || token >= current.length) {
        return undefined;
      }
      current = current[token];
      continue;
    }

    if (typeof current !== "object" || current == undefined || !(token in current)) {
      return undefined;
    }

    current = (current as Record<string, unknown>)[token];
  }

  return current;
}

function TopicsTablePanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [topics, setTopics] = useState<undefined | Immutable<Topic[]>>();
  const [messages, setMessages] = useState<Map<string, unknown>>(new Map());
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [config, setConfig] = useState<PanelSettings>({
    trackedTopics: [],
    updateFrequency: 5, // 5 Hz
    leftColumnWidth: 40, // Default to 40%
  });
  const [isDragging, setIsDragging] = useState(false);
  const [isDividerHovered, setIsDividerHovered] = useState(false);
  const [containerRef, setContainerRef] = useState<HTMLDivElement | null>(null);
  const [lastUpdateTime, setLastUpdateTime] = useState<number>(0);
  const availableTopicNames = useMemo(() => (topics ?? []).map((topic) => topic.name), [topics]);
  const resolvedTrackedTopics = useMemo(
    () =>
      config.trackedTopics.map((trackedTopic) => {
        const resolved = splitTopicPath(trackedTopic.topicPath, availableTopicNames);
        return {
          ...trackedTopic,
          resolvedTopicName: resolved.topicName,
          resolvedFieldPath: resolved.fieldPath,
        };
      }),
    [availableTopicNames, config.trackedTopics],
  );

  // Setup render handling and subscriptions
  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      setTopics(renderState.topics);

      // Check if we should update based on frequency
      const now = Date.now();
      const timeSinceLastUpdate = now - lastUpdateTime;
      // Convert Hz to milliseconds: ms = 1000 / Hz
      const updateIntervalMs = config.updateFrequency === 0 ? 0 : 1000 / config.updateFrequency;
      const shouldUpdate = config.updateFrequency === 0 || timeSinceLastUpdate >= updateIntervalMs;

      // Build a map of the latest messages for subscribed topics
      if (renderState.currentFrame && shouldUpdate) {
        const messageMap = new Map<string, unknown>();

        for (const message of renderState.currentFrame) {
          messageMap.set(message.topic, message.message);
        }

        setMessages((prevMessages) => {
          const newMessages = new Map(prevMessages);
          messageMap.forEach((msg, topic) => {
            newMessages.set(topic, msg);
          });
          return newMessages;
        });

        setLastUpdateTime(now);
      }
    };

    context.watch("topics");
    context.watch("currentFrame");
  }, [context, lastUpdateTime, config.updateFrequency]);

  // Load saved settings on mount
  useEffect(() => {
    const savedSettings = context.initialState as Partial<PanelSettings> | undefined;
    if (savedSettings?.trackedTopics) {
      const settingsWithFrequency: PanelSettings = {
        trackedTopics: savedSettings.trackedTopics.map((topic) => ({
          id: topic.id,
          topicPath: topic.topicPath,
          customLabel:
            "customLabel" in topic && typeof topic.customLabel === "string"
              ? topic.customLabel
              : "",
        })),
        updateFrequency: savedSettings.updateFrequency ?? 5,
        leftColumnWidth: savedSettings.leftColumnWidth ?? 40,
      };
      setConfig(settingsWithFrequency);
    }
  }, [context]);

  // Handle settings changes
  useEffect(() => {
    const subscriptionTopics = [
      ...new Set(resolvedTrackedTopics.map((t) => t.resolvedTopicName)),
    ].filter((topicName) => topicName.length > 0);

    context.subscribe(subscriptionTopics.map((topic) => ({ topic })));
    context.saveState(config);
  }, [config, context, resolvedTrackedTopics]);

  // Invoke the done callback
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const handleAddTopic = useCallback((topicPath: string) => {
    const normalizedPath = topicPath.trim();

    setConfig((prevConfig) => {
      if (
        normalizedPath &&
        prevConfig.trackedTopics.some((t) => t.topicPath.trim() === normalizedPath)
      ) {
        return prevConfig;
      }

      const newId = `topic-${Date.now()}`;
      return {
        ...prevConfig,
        trackedTopics: [
          ...prevConfig.trackedTopics,
          { id: newId, topicPath: normalizedPath, customLabel: "" },
        ],
      };
    });
  }, []);

  const handleUpdateTrackedTopic = useCallback(
    (id: string, field: "topicPath" | "customLabel", value: string) => {
      setConfig((prevConfig) => ({
        ...prevConfig,
        trackedTopics: prevConfig.trackedTopics.map((topic) =>
          topic.id === id ? { ...topic, [field]: value } : topic,
        ),
      }));
    },
    [],
  );

  const handleRemoveTopic = useCallback((id: string) => {
    setConfig((prevConfig) => ({
      ...prevConfig,
      trackedTopics: prevConfig.trackedTopics.filter((t) => t.id !== id),
    }));
  }, []);

  const handleMoveUp = useCallback((id: string) => {
    setConfig((prevConfig) => {
      const index = prevConfig.trackedTopics.findIndex((t) => t.id === id);
      if (index <= 0) {
        return prevConfig;
      }

      const newTopics = [...prevConfig.trackedTopics];
      const currentTopic = newTopics[index];
      const previousTopic = newTopics[index - 1];
      if (!currentTopic || !previousTopic) {
        return prevConfig;
      }

      newTopics[index - 1] = currentTopic;
      newTopics[index] = previousTopic;

      return { ...prevConfig, trackedTopics: newTopics };
    });
  }, []);

  const handleMoveDown = useCallback((id: string) => {
    setConfig((prevConfig) => {
      const index = prevConfig.trackedTopics.findIndex((t) => t.id === id);
      if (index === -1 || index >= prevConfig.trackedTopics.length - 1) {
        return prevConfig;
      }

      const newTopics = [...prevConfig.trackedTopics];
      const currentTopic = newTopics[index];
      const nextTopic = newTopics[index + 1];
      if (!currentTopic || !nextTopic) {
        return prevConfig;
      }

      newTopics[index] = nextTopic;
      newTopics[index + 1] = currentTopic;

      return { ...prevConfig, trackedTopics: newTopics };
    });
  }, []);

  const handleSettingsAction = useCallback(
    (action: SettingsTreeAction) => {
      if (action.action === "update") {
        const { path, value } = action.payload;
        if (path[0] === "updateFrequency" && typeof value === "number") {
          setConfig((prevConfig) => ({ ...prevConfig, updateFrequency: value }));
        } else if (
          path[0] === "series" &&
          typeof path[1] === "string" &&
          (path[2] === "topicPath" || path[2] === "customLabel") &&
          typeof value === "string"
        ) {
          handleUpdateTrackedTopic(path[1], path[2], value);
        }
        return;
      }

      const { path, id } = action.payload;
      if (path[0] === "series" && id === "add") {
        handleAddTopic("");
        return;
      }

      if (path[0] === "series" && typeof path[1] === "string") {
        const topicId = path[1];
        if (id === "remove") {
          handleRemoveTopic(topicId);
        } else if (id === "moveUp") {
          handleMoveUp(topicId);
        } else if (id === "moveDown") {
          handleMoveDown(topicId);
        }
        return;
      }

      if (typeof path[0] === "string") {
        if (path[0] === "addTopic" && id === "add") {
          handleAddTopic("");
        } else if (id === "remove") {
          handleRemoveTopic(path[0]);
        } else if (id === "moveUp") {
          handleMoveUp(path[0]);
        } else if (id === "moveDown") {
          handleMoveDown(path[0]);
        }
      }
    },
    [handleAddTopic, handleMoveDown, handleMoveUp, handleRemoveTopic, handleUpdateTrackedTopic],
  );

  const updateSettingsTree = useCallback(() => {
    const nodes: SettingsTreeNodes = {};

    nodes.updateFrequency = {
      label: "Update Rate",
      fields: {
        updateFrequency: {
          label: "Update interval",
          input: "select",
          options: [
            { label: "Every frame (max)", value: 0 },
            { label: "30 Hz (0.033 s)", value: 30 },
            { label: "10 Hz (0.1 s)", value: 10 },
            { label: "5 Hz (0.2 s)", value: 5 },
            { label: "2 Hz (0.5 s)", value: 2 },
            { label: "1 Hz (1 s)", value: 1 },
            { label: "0.5 Hz (2 s)", value: 0.5 },
            { label: "0.2 Hz (5 s)", value: 0.2 },
          ],
          value: config.updateFrequency,
        },
      },
    };

    nodes.series = {
      label: "Series",
      actions: [
        {
          id: "add",
          type: "action",
          label: "Add series",
          icon: "Addchart",
          display: "inline",
        },
      ],
      children: {},
    };

    const seriesChildren = nodes.series.children;

    if (!seriesChildren) {
      return;
    }

    config.trackedTopics.forEach((trackedTopic, index) => {
      const actions: SettingsTreeNodeAction[] = [];

      if (index > 0) {
        actions.push({
          id: "moveUp",
          type: "action",
          label: "Move Up",
        });
      }

      if (index < config.trackedTopics.length - 1) {
        actions.push({
          id: "moveDown",
          type: "action",
          label: "Move Down",
        });
      }

      actions.push({
        id: "remove",
        type: "action",
        label: "Remove",
      });

      seriesChildren[trackedTopic.id] = {
        label:
          trackedTopic.customLabel.trim() || trackedTopic.topicPath.trim() || `Series ${index + 1}`,
        fields: {
          topicPath: {
            label: "Y-value path",
            input: "messagepath",
            validTopics: availableTopicNames,
            value: trackedTopic.topicPath,
          },
          customLabel: {
            label: "Label",
            input: "string",
            placeholder: "Optional custom label",
            value: trackedTopic.customLabel,
          },
        },
        actions,
      };
    });

    context.updatePanelSettingsEditor({
      actionHandler: handleSettingsAction,
      nodes,
    });
  }, [availableTopicNames, config, context, handleSettingsAction]);

  // Handle settings tree updates
  useLayoutEffect(() => {
    updateSettingsTree();
  }, [updateSettingsTree]);

  const normalizeForRawDisplay = (value: unknown): unknown => {
    if (typeof value === "number") {
      return Number(value.toFixed(3));
    }

    if (typeof value === "bigint") {
      return value.toString();
    }

    if (Array.isArray(value)) {
      return value.map((item) => normalizeForRawDisplay(item));
    }

    if (value != undefined && typeof value === "object") {
      const normalized: Record<string, unknown> = {};
      const record = value as Record<string, unknown>;
      for (const key of Object.keys(record)) {
        normalized[key] = normalizeForRawDisplay(record[key]);
      }
      return normalized;
    }

    if (typeof value === "symbol") {
      const description = (value as symbol & { description?: string }).description;
      return description != undefined ? `Symbol(${description})` : "Symbol";
    }

    return value;
  };

  const formatDisplayValue = (value: unknown): string => {
    if (typeof value === "undefined") {
      return "No data";
    }

    try {
      return JSON.stringify(normalizeForRawDisplay(value), null, 2) ?? "No data";
    } catch {
      return String(normalizeForRawDisplay(value));
    }
  };

  const handleMouseDown = useCallback(() => {
    setIsDragging(true);
  }, []);

  const handleMouseMove = useCallback(
    (e: MouseEvent) => {
      if (!isDragging || !containerRef) {
        return;
      }

      const rect = containerRef.getBoundingClientRect();
      const newWidth = ((e.clientX - rect.left) / rect.width) * 100;

      // Clamp between 20% and 80% for usability
      const clampedWidth = Math.max(20, Math.min(80, newWidth));

      setConfig((prevConfig) => ({
        ...prevConfig,
        leftColumnWidth: clampedWidth,
      }));
    },
    [isDragging, containerRef],
  );

  const handleMouseUp = useCallback(() => {
    setIsDragging(false);
  }, []);

  useEffect(() => {
    if (isDragging) {
      document.addEventListener("mousemove", handleMouseMove);
      document.addEventListener("mouseup", handleMouseUp);
      document.body.style.cursor = "col-resize";
      document.body.style.userSelect = "none";

      return () => {
        document.removeEventListener("mousemove", handleMouseMove);
        document.removeEventListener("mouseup", handleMouseUp);
        document.body.style.cursor = "";
        document.body.style.userSelect = "";
      };
    }
    return undefined;
  }, [isDragging, handleMouseMove, handleMouseUp]);

  return (
    <div style={{ display: "flex", flexDirection: "column", height: "100%", padding: "1rem" }}>
      <div ref={setContainerRef} style={{ flex: 1, overflowY: "auto", position: "relative" }}>
        <table
          style={{
            width: "100%",
            borderCollapse: "collapse",
            fontFamily: "monospace",
            tableLayout: "fixed",
          }}
        >
          <colgroup>
            <col style={{ width: `${config.leftColumnWidth}%` }} />
            <col style={{ width: `${100 - config.leftColumnWidth}%` }} />
          </colgroup>
          <tbody>
            {config.trackedTopics.length === 0 ? (
              <tr>
                <td colSpan={2} style={{ padding: "2rem", textAlign: "center", color: "#999" }}>
                  No topics configured. Add topics in the settings panel on the left.
                </td>
              </tr>
            ) : (
              config.trackedTopics.map((t) => {
                const selectionPath = t.topicPath;
                const { topicName: resolvedTopicName, fieldPath: resolvedFieldPath } =
                  splitTopicPath(selectionPath, availableTopicNames);
                const selectedValue = getNestedValue(
                  messages.get(resolvedTopicName),
                  resolvedFieldPath,
                );
                const displayName = t.customLabel.trim() || selectionPath.trim() || resolvedTopicName;

                return (
                  <tr
                    key={t.id}
                    style={{
                      borderBottom: "1px solid #444",
                      backgroundColor: "transparent",
                    }}
                  >
                    <td
                      style={{
                        padding: "0.25rem 0.35rem",
                        borderRight: "1px solid #444",
                        fontWeight: 500,
                        color: "white",
                        wordBreak: "break-word",
                      }}
                    >
                      {displayName}
                    </td>
                    <td
                      style={{
                        padding: "0.25rem 0.35rem",
                        color: "white",
                        opacity: 1,
                        wordBreak: "break-word",
                        whiteSpace: "pre-wrap",
                        fontFamily: "monospace",
                      }}
                    >
                      {formatDisplayValue(selectedValue)}
                    </td>
                  </tr>
                );
              })
            )}
          </tbody>
        </table>
        <div
          onMouseDown={handleMouseDown}
          onMouseEnter={() => {
            setIsDividerHovered(true);
          }}
          onMouseLeave={() => {
            setIsDividerHovered(false);
          }}
          style={{
            position: "absolute",
            left: `calc(${config.leftColumnWidth}% - 4px)`,
            top: 0,
            bottom: 0,
            width: 8,
            cursor: "col-resize",
            zIndex: 20,
            backgroundColor: isDragging || isDividerHovered ? "#4488ff66" : "transparent",
            transition: "background-color 0.15s",
          }}
        />
      </div>
    </div>
  );
}

export function initExamplePanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<TopicsTablePanel context={context} />);

  return () => {
    root.unmount();
  };
}
