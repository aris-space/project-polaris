import {
  Immutable,
  PanelExtensionContext,
  Topic,
  SettingsTreeAction,
} from "@foxglove/extension";
import { ReactElement, useEffect, useLayoutEffect, useState } from "react";
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
}

function TopicsTablePanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [topics, setTopics] = useState<undefined | Immutable<Topic[]>>();
  const [messages, setMessages] = useState<Map<string, unknown>>(new Map());
  const [lastUpdateTimes, setLastUpdateTimes] = useState<Map<string, number>>(new Map());
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [config, setConfig] = useState<PanelSettings>({
    trackedTopics: [],
    updateFrequency: 0, // 0 = every frame
    showHeader: true,
    staleAfterMissedUpdates: 3, // stale after 3 missed updates by default
    showFullTopicPath: true,
  });
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
    const savedSettings = context.initialState as PanelSettings | undefined;
    if (savedSettings?.trackedTopics) {
      const settingsWithFrequency: PanelSettings = {
        trackedTopics: savedSettings.trackedTopics,
        updateFrequency: savedSettings.updateFrequency ?? 0,
        showHeader: savedSettings.showHeader ?? true,
        staleAfterMissedUpdates: savedSettings.staleAfterMissedUpdates ?? 3,
        showFullTopicPath: savedSettings.showFullTopicPath ?? true,
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
      setForceUpdate(prev => prev + 1);
    }, 500); // Update every 500ms to refresh staleness
    
    return () => clearInterval(interval);
  }, [config.staleAfterMissedUpdates]);

  // Handle settings tree updates
  useLayoutEffect(() => {
    updateSettingsTree();
  }, [config, topics]);

  // Handle settings changes
  useEffect(() => {
    if (config.trackedTopics.length > 0) {
      context.subscribe(config.trackedTopics.map((t) => ({ topic: t.topicName })));
    }
    context.saveState(config);
  }, [config, context]);

  // Invoke the done callback
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const updateSettingsTree = () => {
    const nodes: Record<string, any> = {};

    // Add update frequency selector
    nodes.updateFrequency = {
      label: "Update Rate",
      fields: {
        updateFrequency: {
          label: "How often to update values",
          input: "select",
          options: [
            { label: "Every frame (max)", value: 0 },
            { label: "30 Hz", value: 30 },
            { label: "10 Hz", value: 10 },
            { label: "5 Hz", value: 5 },
            { label: "2 Hz", value: 2 },
            { label: "1 Hz", value: 1 },
            { label: "0.5 Hz", value: 0.5 },
            { label: "0.2 Hz", value: 0.2 },
          ],
          value: config.updateFrequency,
        },
        staleAfterMissedUpdates: {
          label: "Mark stale after missed updates",
          input: "select",
          options: [
            { label: "Disabled", value: 0 },
            { label: "1 missed update", value: 1 },
            { label: "2 missed updates", value: 2 },
            { label: "3 missed updates", value: 3 },
            { label: "5 missed updates", value: 5 },
            { label: "10 missed updates", value: 10 },
          ],
          value: config.staleAfterMissedUpdates,
        },
      },
    };

    // Add header visibility toggle
    nodes.showHeader = {
      label: "Display",
      fields: {
        showHeader: {
          label: "Show header row",
          input: "boolean",
          value: config.showHeader,
        },
        showFullTopicPath: {
          label: "Show full topic path",
          input: "boolean",
          value: config.showFullTopicPath,
        },
      },
    };

    // Add topic selector
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

    // Add tracked topics with remove and reorder actions
    config.trackedTopics.forEach((trackedTopic, index) => {
      const actions: any[] = [];
      
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
  };

  const handleSettingsAction = (action: SettingsTreeAction) => {
    if (action.action === "update") {
      const { path, value } = action.payload;
      if (path[0] === "updateFrequency" && typeof value === "number") {
        const fieldKey = path[path.length - 1];
        if (fieldKey === "staleAfterMissedUpdates") {
          setConfig({ ...config, staleAfterMissedUpdates: value });
        } else {
          setConfig({ ...config, updateFrequency: value });
        }
      } else if (path[0] === "showHeader" && typeof value === "boolean") {
        const fieldKey = path[path.length - 1];
        if (fieldKey === "showFullTopicPath") {
          setConfig({ ...config, showFullTopicPath: value });
        } else {
          setConfig({ ...config, showHeader: value });
        }
      } else if (path[0] === "addTopic" && typeof value === "string" && value) {
        handleAddTopic(value);
      }
    } else if (action.action === "perform-node-action") {
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
    }
  };

  const handleAddTopic = (topicName: string) => {
    if (topicName && !config.trackedTopics.some((t) => t.topicName === topicName)) {
      const newId = `topic-${Date.now()}`;
      setConfig({
        ...config,
        trackedTopics: [...config.trackedTopics, { id: newId, topicName }],
      });
    }
  };

  const handleRemoveTopic = (id: string) => {
    setConfig({
      ...config,
      trackedTopics: config.trackedTopics.filter((t) => t.id !== id),
    });
  };

  const handleMoveUp = (id: string) => {
    const index = config.trackedTopics.findIndex((t) => t.id === id);
    if (index <= 0) {
      return;
    }

    const newTopics = [...config.trackedTopics];
    const currentTopic = newTopics[index];
    const previousTopic = newTopics[index - 1];

    if (!currentTopic || !previousTopic) {
      return;
    }

    newTopics[index - 1] = currentTopic;
    newTopics[index] = previousTopic;
    setConfig({ ...config, trackedTopics: newTopics });
  };

  const handleMoveDown = (id: string) => {
    const index = config.trackedTopics.findIndex((t) => t.id === id);
    if (index === -1 || index >= config.trackedTopics.length - 1) {
      return;
    }

    const newTopics = [...config.trackedTopics];
    const currentTopic = newTopics[index];
    const nextTopic = newTopics[index + 1];

    if (!currentTopic || !nextTopic) {
      return;
    }

    newTopics[index] = nextTopic;
    newTopics[index + 1] = currentTopic;
    setConfig({ ...config, trackedTopics: newTopics });
  };

  const formatValue = (value: unknown): string => {
    if (value === undefined) {
      return "No data";
    }

    if (value === null) {
      return "null";
    }
    
    if (typeof value === "object") {
      if (Array.isArray(value)) {
        // Show array contents if small, otherwise show length
        if (value.length <= 3) {
          return `[${value.map(v => formatValue(v)).join(", ")}]`;
        }
        return `Array[${value.length}]`;
      }
      
      // Handle common ROS message patterns
      const obj = value as Record<string, unknown>;
      
      // Check for std_msgs pattern (single 'data' field)
      if (Object.keys(obj).length === 1 && "data" in obj) {
        return formatValue(obj.data);
      }
      
      // Try to stringify if it's a small object
      try {
        const str = JSON.stringify(value);
        if (str.length <= 50) {
          return str;
        }
        // Show a summary of the object
        const keys = Object.keys(obj);
        if (keys.length <= 3) {
          return `{${keys.map(k => `${k}: ${formatValue(obj[k])}`).join(", ")}}`;
        }
        return `Object {${keys.slice(0, 3).join(", ")}...}`;
      } catch {
        return "Object";
      }
    }
    
    // Handle primitives
    if (typeof value === "number") {
      // Format numbers with reasonable precision
      return Number.isInteger(value) ? String(value) : value.toFixed(3);
    }
    
    return String(value);
  };

  const isStale = (topicName: string): boolean => {
    if (config.staleAfterMissedUpdates === 0) {
      return false;
    }
    
    const lastUpdate = lastUpdateTimes.get(topicName);
    if (!lastUpdate) {
      return false; // No data yet, not considered stale
    }
    
    const expectedUpdateIntervalMs = config.updateFrequency > 0 ? 1000 / config.updateFrequency : 1000;
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

  return (
    <div style={{ display: "flex", flexDirection: "column", height: "100%", padding: "1rem" }}>
      <div style={{ flex: 1, overflowY: "auto" }}>
        <table
          style={{
            width: "100%",
            borderCollapse: "collapse",
            fontFamily: "monospace",
            tableLayout: "fixed",
          }}
        >
          <colgroup>
            <col style={{ width: "40%" }} />
            <col style={{ width: "60%" }} />
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
                    padding: "0.75rem",
                    borderRight: "1px solid #555",
                    fontWeight: "bold",
                    color: "white",
                    overflow: "hidden",
                    textOverflow: "ellipsis",
                  }}
                >
                  Topic Name
                </th>
                <th
                  style={{
                    textAlign: "left",
                    padding: "0.75rem",
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
                        padding: "0.75rem",
                        borderRight: "1px solid #444",
                        fontWeight: 500,
                        color: stale ? "#ff8800" : "white",
                        overflow: "hidden",
                        textOverflow: "ellipsis",
                        whiteSpace: "nowrap",
                      }}
                    >
                      {stale && "⚠️ "}{formatTopicName(t.topicName)}
                    </td>
                    <td style={{ 
                      padding: "0.75rem", 
                      wordBreak: "break-all", 
                      color: stale ? "#ff8800" : "white",
                      opacity: stale ? 0.6 : 1,
                    }}>
                      {formatValue(messages.get(t.topicName))}
                    </td>
                  </tr>
                );
              })
            )}
          </tbody>
        </table>
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
