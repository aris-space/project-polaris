import {
  Immutable,
  PanelExtensionContext,
  SettingsTreeAction,
  SettingsTreeNodeAction,
  SettingsTreeNodes,
  Topic,
} from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

interface TopicConfig {
  id: string;
  topicName: string;
}

interface PanelSettings {
  trackedTopics: TopicConfig[];
  updateFrequency: number; // Hz (0 = every frame)
  showHeader: boolean;
  staleAfterMissedUpdates: number; // Number of missed updates before marking stale (0 = disabled)
  showFullTopicPath: boolean;
  leftColumnWidth: number; // Percentage width of left column (0-100)
  showRawMessage: boolean; // Show original JSON format vs formatted
  compactMode: boolean; // Use compact row spacing
}

function TopicsTablePanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [topics, setTopics] = useState<undefined | Immutable<Topic[]>>();
  const [messages, setMessages] = useState<Map<string, unknown>>(new Map());
  const [lastUpdateTimes, setLastUpdateTimes] = useState<Map<string, number>>(new Map());
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [config, setConfig] = useState<PanelSettings>({
    trackedTopics: [],
    updateFrequency: 5, // 5 Hz
    showHeader: true,
    staleAfterMissedUpdates: 5, // stale after 5 missed updates by default
    showFullTopicPath: false,
    leftColumnWidth: 40, // Default to 40%
    showRawMessage: false, // Default to formatted view
    compactMode: false, // Default to normal spacing
  });
  const [isDragging, setIsDragging] = useState(false);
  const [isDividerHovered, setIsDividerHovered] = useState(false);
  const [containerRef, setContainerRef] = useState<HTMLDivElement | null>(null);
  const [lastUpdateTime, setLastUpdateTime] = useState<number>(0);
  const [, setForceUpdate] = useState(0); // For forcing re-renders to update staleness

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
        const timestampMap = new Map<string, number>();
        const currentTime = Date.now();

        for (const message of renderState.currentFrame) {
          messageMap.set(message.topic, message.message);
          timestampMap.set(message.topic, currentTime);
        }

        setMessages((prevMessages) => {
          const newMessages = new Map(prevMessages);
          messageMap.forEach((msg, topic) => {
            newMessages.set(topic, msg);
          });
          return newMessages;
        });

        setLastUpdateTimes((prevTimes) => {
          const newTimes = new Map(prevTimes);
          timestampMap.forEach((time, topic) => {
            newTimes.set(topic, time);
          });
          return newTimes;
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
        trackedTopics: savedSettings.trackedTopics,
        updateFrequency: savedSettings.updateFrequency ?? 5,
        showHeader: savedSettings.showHeader ?? true,
        staleAfterMissedUpdates: savedSettings.staleAfterMissedUpdates ?? 5,
        showFullTopicPath: savedSettings.showFullTopicPath ?? false,
        leftColumnWidth: savedSettings.leftColumnWidth ?? 40,
        showRawMessage: savedSettings.showRawMessage ?? false,
        compactMode: savedSettings.compactMode ?? false,
      };
      setConfig(settingsWithFrequency);
    }
  }, [context]);

  // Force re-render periodically to update staleness indicators
  useEffect(() => {
    if (config.staleAfterMissedUpdates === 0) {
      return;
    }

    const interval = setInterval(() => {
      setForceUpdate((prev) => prev + 1);
    }, 500); // Update every 500ms to refresh staleness

    return () => {
      clearInterval(interval);
    };
  }, [config.staleAfterMissedUpdates]);

  // Handle settings changes
  useEffect(() => {
    context.subscribe(config.trackedTopics.map((t) => ({ topic: t.topicName })));
    context.saveState(config);
  }, [config, context]);

  // Invoke the done callback
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const handleAddTopic = useCallback((topicName: string) => {
    if (!topicName) {
      return;
    }

    setConfig((prevConfig) => {
      if (prevConfig.trackedTopics.some((t) => t.topicName === topicName)) {
        return prevConfig;
      }

      const newId = `topic-${Date.now()}`;
      return {
        ...prevConfig,
        trackedTopics: [...prevConfig.trackedTopics, { id: newId, topicName }],
      };
    });
  }, []);

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
          const fieldKey = path[path.length - 1];
          if (fieldKey === "staleAfterMissedUpdates") {
            setConfig((prevConfig) => ({ ...prevConfig, staleAfterMissedUpdates: value }));
          } else {
            setConfig((prevConfig) => ({ ...prevConfig, updateFrequency: value }));
          }
        } else if (path[0] === "showHeader" && typeof value === "boolean") {
          const fieldKey = path[path.length - 1];
          if (fieldKey === "showFullTopicPath") {
            setConfig((prevConfig) => ({ ...prevConfig, showFullTopicPath: value }));
          } else if (fieldKey === "showRawMessage") {
            setConfig((prevConfig) => ({ ...prevConfig, showRawMessage: value }));
          } else if (fieldKey === "compactMode") {
            setConfig((prevConfig) => ({ ...prevConfig, compactMode: value }));
          } else {
            setConfig((prevConfig) => ({ ...prevConfig, showHeader: value }));
          }
        } else if (path[0] === "addTopic" && typeof value === "string" && value) {
          handleAddTopic(value);
        }
        return;
      }

      const { path, id } = action.payload;
      if (typeof path[0] === "string") {
        if (id === "remove") {
          handleRemoveTopic(path[0]);
        } else if (id === "moveUp") {
          handleMoveUp(path[0]);
        } else if (id === "moveDown") {
          handleMoveDown(path[0]);
        }
      }
    },
    [handleAddTopic, handleMoveDown, handleMoveUp, handleRemoveTopic],
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
        staleAfterMissedUpdates: {
          label: "Stale threshold",
          input: "select",
          options: [
            { label: "Disabled", value: 0 },
            { label: "1 missed update", value: 1 },
            { label: "2 missed updates", value: 2 },
            { label: "3 missed updates", value: 3 },
            { label: "5 missed updates", value: 5 },
            { label: "10 missed updates", value: 10 },
            { label: "15 missed updates", value: 15 },
            { label: "20 missed updates", value: 20 },
            { label: "25 missed updates", value: 25 },
            { label: "30 missed updates", value: 30 },
            { label: "50 missed updates", value: 50 },
          ],
          value: config.staleAfterMissedUpdates,
        },
      },
    };

    nodes.showHeader = {
      label: "Display",
      fields: {
        showHeader: {
          label: "Header row",
          input: "boolean",
          value: config.showHeader,
        },
        showFullTopicPath: {
          label: "Full topic path",
          input: "boolean",
          value: config.showFullTopicPath,
        },
        showRawMessage: {
          label: "Raw format",
          input: "boolean",
          value: config.showRawMessage,
        },
        compactMode: {
          label: "Compact spacing",
          input: "boolean",
          value: config.compactMode,
        },
      },
    };

    const availableTopicNames = (topics ?? [])
      .filter((t) => !config.trackedTopics.some((ct) => ct.topicName === t.name))
      .map((topic) => topic.name);

    nodes.addTopic = {
      label: "Add Topic",
      fields: {
        value: {
          label: "Select Topic",
          input: "select",
          options: availableTopicNames.map((topicName) => ({
            label: topicName,
            value: topicName,
          })),
          value: "",
        },
      },
    };

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

      nodes[trackedTopic.id] = {
        label: trackedTopic.topicName,
        actions,
      };
    });

    context.updatePanelSettingsEditor({
      actionHandler: handleSettingsAction,
      nodes,
    });
  }, [config, context, handleSettingsAction, topics]);

  // Handle settings tree updates
  useLayoutEffect(() => {
    updateSettingsTree();
  }, [updateSettingsTree]);

  const formatRawValue = (value: unknown): string => {
    if (typeof value === "undefined") {
      return "No data";
    }

    try {
      return JSON.stringify(value, null, 2);
    } catch {
      return String(value);
    }
  };

  const isComplexObject = (value: unknown): boolean => {
    if (typeof value !== "object" || value == null) {
      return false;
    }
    
    if (Array.isArray(value)) {
      return value.length > 3 || value.some((v) => typeof v === "object" && v != null);
    }
    
    const obj = value as Record<string, unknown>;
    const keys = Object.keys(obj);
    
    // Complex if more than 3 keys or has nested objects/arrays
    return keys.length > 3 || keys.some((key) => {
      const val = obj[key];
      return typeof val === "object" && val != null;
    });
  };

  const formatValue = (value: unknown, indent = ""): string => {
    if (typeof value === "undefined") {
      return "No data";
    }

    if (value == null) {
      return "null";
    }

    if (typeof value === "object") {
      if (Array.isArray(value)) {
        // Show array contents if small and simple
        if (value.length === 0) {
          return "[]";
        }
        
        const hasComplexItems = value.some((v) => typeof v === "object" && v != null);
        
        if (value.length <= 3 && !hasComplexItems) {
          return `[${value.map((v) => formatValue(v, indent)).join(", ")}]`;
        }
        
        // For larger or complex arrays, show with line breaks
        const items = value.map((v, i) => {
          const formatted = formatValue(v, indent + "  ");
          return `${indent}  [${i}]: ${formatted}`;
        });
        return `\n${items.join("\n")}`;
      }

      // Handle common ROS message patterns
      const obj = value as Record<string, unknown>;

      // Check for std_msgs pattern (single 'data' field)
      if (Object.keys(obj).length === 1 && "data" in obj) {
        return formatValue(obj.data, indent);
      }

      try {
        const keys = Object.keys(obj);
        if (keys.length === 0) {
          return "{}";
        }
        
        // Check if this is a complex object
        if (isComplexObject(obj)) {
          // Format with line breaks
          const lines = keys.map((key) => {
            const val = obj[key];
            const formattedValue = formatValue(val, indent + "  ");
            // If the formatted value starts with a newline, it's a nested structure
            if (formattedValue.startsWith("\n")) {
              return `${indent}  ${key}:${formattedValue}`;
            }
            return `${indent}  ${key}: ${formattedValue}`;
          });
          return `\n${lines.join("\n")}`;
        }
        
        // Simple object - show inline
        if (keys.length <= 3) {
          return keys.map((key) => `${key}: ${formatValue(obj[key], indent)}`).join(", ");
        }
        return `${keys.slice(0, 3).join(", ")}...`;
      } catch {
        return "Object";
      }
    }

    // Handle primitives
    if (typeof value === "number") {
      // Format numbers with reasonable precision
      return Number.isInteger(value) ? String(value) : value.toFixed(3);
    }

    if (typeof value === "string") {
      return value;
    }

    if (typeof value === "boolean" || typeof value === "bigint") {
      return String(value);
    }

    if (typeof value === "symbol") {
      return value.description != undefined ? `Symbol(${value.description})` : "Symbol";
    }

    return "Unsupported";
  };

  const isStale = (topicName: string): boolean => {
    if (config.staleAfterMissedUpdates === 0) {
      return false;
    }

    const lastUpdate = lastUpdateTimes.get(topicName);
    if (lastUpdate == undefined) {
      return false; // No data yet, not considered stale
    }

    const expectedUpdateIntervalMs =
      config.updateFrequency > 0 ? 1000 / config.updateFrequency : 1000;
    const staleAfterMs = expectedUpdateIntervalMs * config.staleAfterMissedUpdates;

    const now = Date.now();
    const ageMs = now - lastUpdate;
    return ageMs > staleAfterMs;
  };

  const formatTopicName = (topicName: string): string => {
    if (config.showFullTopicPath) {
      return topicName;
    }

    const parts = topicName.split("/").filter((part) => part.length > 0);
    return parts.length > 0 ? parts[parts.length - 1]! : topicName;
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
      <div 
        ref={setContainerRef} 
        style={{ flex: 1, overflowY: "auto", position: "relative" }}
      >
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
          {config.showHeader && (
            <thead>
              <tr
                style={{
                  backgroundColor: "#333333",
                  borderBottom: "2px solid #555",
                }}
              >
                <th
                  style={{
                    textAlign: "left",
                    padding: config.compactMode ? "0.25rem 0.35rem" : "0.5rem",
                    borderRight: "1px solid #555",
                    fontWeight: "bold",
                    color: "white",
                    overflow: "hidden",
                    textOverflow: "ellipsis",
                    whiteSpace: "nowrap",
                    maxWidth: 0,
                  }}
                >
                  Topic Name
                </th>
                <th
                  style={{
                    textAlign: "left",
                    padding: config.compactMode ? "0.25rem 0.35rem" : "0.5rem",
                    fontWeight: "bold",
                    color: "white",
                    overflow: "hidden",
                    textOverflow: "ellipsis",
                  }}
                >
                  Current Value
                </th>
              </tr>
            </thead>
          )}
          <tbody>
            {config.trackedTopics.length === 0 ? (
              <tr>
                <td colSpan={2} style={{ padding: "2rem", textAlign: "center", color: "#999" }}>
                  No topics configured. Add topics in the settings panel on the left.
                </td>
              </tr>
            ) : (
              config.trackedTopics.map((t) => {
                const stale = isStale(t.topicName);

                return (
                  <tr
                    key={t.id}
                    style={{
                      borderBottom: "1px solid #444",
                      backgroundColor: stale ? "rgba(255, 100, 0, 0.1)" : "transparent",
                    }}
                  >
                    <td
                      style={{
                        padding: config.compactMode ? "0.25rem 0.35rem" : "0.5rem",
                        borderRight: "1px solid #444",
                        fontWeight: 500,
                        color: stale ? "#ff8800" : "white",
                        wordBreak: "break-word",
                      }}
                    >
                      {stale && "⚠️ "}
                      {formatTopicName(t.topicName)}
                    </td>
                    <td
                      style={{
                        padding: config.compactMode ? "0.25rem 0.35rem" : "0.5rem",
                        color: stale ? "#ff8800" : "white",
                        opacity: stale ? 0.6 : 1,
                        wordBreak: "break-word",
                        whiteSpace: "pre-wrap",
                        fontFamily: config.showRawMessage ? "monospace" : "inherit",
                      }}
                    >
                      {config.showRawMessage
                        ? formatRawValue(messages.get(t.topicName))
                        : formatValue(messages.get(t.topicName)).replace(/^\n/, "")}
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
