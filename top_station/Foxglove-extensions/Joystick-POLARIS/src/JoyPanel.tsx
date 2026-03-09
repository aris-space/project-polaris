import { fromDate } from "@foxglove/rostime";
import {
  Immutable,
  MessageEvent,
  PanelExtensionContext,
  Topic,
  SettingsTreeAction,
} from "@foxglove/studio";
import { FormGroup, FormControlLabel, Switch } from "@mui/material";
import { useEffect, useLayoutEffect, useState, useCallback } from "react";
import { createRoot } from "react-dom/client";

// import { GamepadDebug } from "./components/GamepadDebug";
import { GamepadView } from "./components/GamepadView";
import { SimpleButtonView } from "./components/SimpleButtonView";
import { JoyDataDisplay } from "./components/JoyDataDisplay";
import kbmappingKeyboardButtons from "./components/kbmapping-keyboard_buttons.json";
import kbmappingKeyboardMovement from "./components/kbmapping-keyboard_movement.json";
import kbmapping1 from "./components/kbmapping1.json";
import { useGamepad } from "./hooks/useGamepad";
import { gamepadToRosJoy } from "./utils/gamepadToRosJoy";
import { Config, buildSettingsTree, settingsActionReducer } from "./panelSettings";
import { Joy } from "./types";

type KbMap = {
  button: number;
  axis: number;
  direction: number;
  value: number;
  toggle: boolean;
  toggled: boolean;
};

type RawKbMap = {
  button: number;
  axis: number;
  direction: string | null;
  toggle?: boolean;
};

const keyboardMappings: Record<string, Record<string, RawKbMap>> = {
  default: kbmapping1,
  keyboard_movement: kbmappingKeyboardMovement,
  keyboard_buttons: kbmappingKeyboardButtons,
};

function buildKeyMap(mapping: Record<string, RawKbMap>): Map<string, KbMap> {
  const keyMap = new Map<string, KbMap>();

  for (const [key, value] of Object.entries(mapping)) {
    const k: KbMap = {
      button: value.button,
      axis: value.axis,
      direction: value.direction === "+" ? 1 : value.direction === "-" ? -1 : 0,
      value: 0,
      toggle: value.toggle ?? false,
      toggled: false,
    };
    keyMap.set(key, k);
  }

  return keyMap;
}

function JoyPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [topics, setTopics] = useState<undefined | Immutable<Topic[]>>();
  const [messages, setMessages] = useState<undefined | Immutable<MessageEvent[]>>();
  const [joy, setJoy] = useState<Joy | undefined>();
  const [pubTopic, setPubTopic] = useState<string | undefined>();
  const [kbEnabled, setKbEnabled] = useState<boolean>(true);
  const [trackedKeys, setTrackedKeys] = useState<Map<string, KbMap> | undefined>(() =>
    buildKeyMap(kbmapping1),
  );
  const [currentKbMapping, setCurrentKbMapping] = useState<Record<string, RawKbMap> | undefined>();

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const [config, setConfig] = useState<Config>(() => {
    const partialConfig = context.initialState as Partial<Config>;
    partialConfig.subJoyTopic ??= "/joy";
    partialConfig.publishMode ??= false;
    partialConfig.publishFrameId ??= "joystick_frame";
    if (partialConfig.publishFrameId.trim() === "") {
      partialConfig.publishFrameId = "joystick_frame";
    }
    partialConfig.dataSource ??= "sub-joy-topic";
    partialConfig.layoutName ??= "ps4";
    partialConfig.mapping_name ??= "TODO";
    partialConfig.keyboardMapping ??= "default";
    partialConfig.gamepadId ??= 0;
    partialConfig.uiScale ??= 1;
    
    // Set default pubJoyTopic based on data source and keyboard mapping
    if (partialConfig.pubJoyTopic == undefined) {
      if (partialConfig.dataSource === "gamepad") {
        partialConfig.pubJoyTopic = "/joy_controller";
      } else if (partialConfig.dataSource === "keyboard") {
        if (partialConfig.keyboardMapping === "keyboard_movement") {
          partialConfig.pubJoyTopic = "/joy_keyboard";
        } else if (partialConfig.keyboardMapping === "keyboard_buttons") {
          partialConfig.pubJoyTopic = "/joy_mode";
        } else {
          partialConfig.pubJoyTopic = "/joy";
        }
      } else {
        partialConfig.pubJoyTopic = "/joy";
      }
    }
    
    return partialConfig as Config;
  });

  const settingsActionHandler = useCallback(
    (action: SettingsTreeAction) => {
      setConfig((prevConfig) => settingsActionReducer(prevConfig, action));
    },
    [setConfig],
  );

  // Auto-update pubJoyTopic based on data source and keyboard mapping
  useEffect(() => {
    setConfig((prevConfig) => {
      let newPubJoyTopic = prevConfig.pubJoyTopic;

      if (prevConfig.dataSource === "gamepad") {
        newPubJoyTopic = "/joy_controller";
      } else if (prevConfig.dataSource === "keyboard") {
        if (prevConfig.keyboardMapping === "keyboard_movement") {
          newPubJoyTopic = "/joy_keyboard";
        } else if (prevConfig.keyboardMapping === "keyboard_buttons") {
          newPubJoyTopic = "/joy_mode";
        } else {
          newPubJoyTopic = "/joy"; // default for keyboard
        }
      }

      if (newPubJoyTopic !== prevConfig.pubJoyTopic) {
        return { ...prevConfig, pubJoyTopic: newPubJoyTopic };
      }
      return prevConfig;
    });
  }, [config.dataSource, config.keyboardMapping]);

  // Register the settings tree
  useEffect(() => {
    context.updatePanelSettingsEditor({
      actionHandler: settingsActionHandler,
      nodes: buildSettingsTree(config, topics),
    });
  }, [config, context, settingsActionHandler, topics]);

  // We use a layout effect to setup render handling for our panel. We also setup some topic subscriptions.
  useLayoutEffect(() => {
    // The render handler is run by the broader studio system during playback when your panel
    // needs to render because the fields it is watching have changed. How you handle rendering depends on your framework.
    // You can only setup one render handler - usually early on in setting up your panel.
    //
    // Without a render handler your panel will never receive updates.
    //
    // The render handler could be invoked as often as 60hz during playback if fields are changing often.
    context.onRender = (renderState, done) => {
      // render functions receive a _done_ callback. You MUST call this callback to indicate your panel has finished rendering.
      // Your panel will not receive another render callback until _done_ is called from a prior render. If your panel is not done
      // rendering before the next render call, studio shows a notification to the user that your panel is delayed.
      //
      // Set the done callback into a state variable to trigger a re-render.
      setRenderDone(() => done);

      // We may have new topics - since we are also watching for messages in the current frame, topics may not have changed
      // It is up to you to determine the correct action when state has not changed.
      setTopics(renderState.topics);

      // currentFrame has messages on subscribed topics since the last render call
      setMessages(renderState.currentFrame);
    };

    // After adding a render handler, you must indicate which fields from RenderState will trigger updates.
    // If you do not watch any fields then your panel will never render since the panel context will assume you do not want any updates.

    // tell the panel context that we care about any update to the _topic_ field of RenderState
    context.watch("topics");

    // tell the panel context we want messages for the current frame for topics we've subscribed to
    // This corresponds to the _currentFrame_ field of render state.
    context.watch("currentFrame");
  }, [context]);

  // Or subscribe to the relevant topic when in a recorded session
  useEffect(() => {
    if (config.dataSource === "sub-joy-topic") {
      context.subscribe([config.subJoyTopic]);
    } else {
      context.unsubscribeAll();
    }
  }, [config.subJoyTopic, context, config.dataSource]);

  // If subscribing
  useEffect(() => {
    const latestJoy = messages?.[messages.length - 1]?.message as Joy | undefined;
    if (latestJoy) {
      // Validate array sizes for safety
      const axes = Array.from(latestJoy.axes || []);
      const buttons = Array.from(latestJoy.buttons || []);
      
      // Ensure we don't have malformed data
      if (axes.length > 100 || buttons.length > 100) {
        console.error("[POLARIS Joystick] Received malformed Joy message with excessive array sizes. Ignoring.");
        return;
      }
      
      const tmpMsg = {
        header: {
          stamp: latestJoy.header.stamp,
          frame_id: config.publishFrameId || "joystick_frame",
        },
        axes,
        buttons,
      };
      setJoy(tmpMsg);
    }
  }, [messages, config.publishFrameId]);

  useGamepad({
    didConnect: useCallback((gp: Gamepad) => {
      // eslint-disable-next-line no-warning-comments
      // TODO update the gamepad ID list
      console.log("Gamepad " + gp.index + " connected!");
    }, []),

    didDisconnect: useCallback((gp: Gamepad) => {
      // eslint-disable-next-line no-warning-comments
      // TODO update the gamepad ID list
      console.log("Gamepad " + gp.index + " discconnected!");
    }, []),

    didUpdate: useCallback(
      (gp: Gamepad) => {
        if (config.dataSource !== "gamepad") {
          return;
        }

        if (config.gamepadId !== gp.index) {
          return;
        }

        // Convert Gamepad API to ROS /joy format using the mapping
        const { buttons, axes } = gamepadToRosJoy(gp);

        const tmpJoy = {
          header: {
            frame_id: config.publishFrameId || "joystick_frame",
            stamp: fromDate(new Date()),
          },
          axes,
          buttons,
        } as Joy;

        setJoy(tmpJoy);
      },
      [config.dataSource, config.gamepadId, config.publishFrameId],
    ),
  });

  // Keyboard mode
  const normalizeKey = useCallback((event: KeyboardEvent): string => {
    // Prevent keyboard input if we're typing in an input field
    const target = event.target as HTMLElement;
    if (target && (target.tagName === "INPUT" || target.tagName === "TEXTAREA" || target.isContentEditable)) {
      return "";
    }
    
    const { code, key } = event;
    if (code.startsWith("Key")) {
      return code.slice(3).toLowerCase();
    }
    if (code.startsWith("Digit")) {
      return code.slice(5);
    }
    if (code === "Space") {
      return "Space";
    }
    return key;
  }, []);

  const handleKeyDown = useCallback(
    (event: KeyboardEvent) => {
      if (!kbEnabled) {
        return;
      }
      
      const normalizedKey = normalizeKey(event);
      if (!normalizedKey) {
        return; // Ignore if in input field
      }
      
      setTrackedKeys((oldTrackedKeys) => {
        if (oldTrackedKeys && oldTrackedKeys.has(normalizedKey)) {
          event.preventDefault(); // Prevent default browser behavior for mapped keys
          const newKeys = new Map(oldTrackedKeys);
          const k = newKeys.get(normalizedKey);
          if (k) {
            if (k.toggle) {
              // Toggle mode - switch toggled state
              k.toggled = !k.toggled;
              k.value = k.toggled ? 1 : 0;
            } else {
              // Regular mode - just set value
              k.value = 1;
            }
          }
          return newKeys;
        }
        return oldTrackedKeys;
      });
    },
    [normalizeKey, kbEnabled],
  );

  const handleKeyUp = useCallback(
    (event: KeyboardEvent) => {
      if (!kbEnabled) {
        return;
      }
      
      const normalizedKey = normalizeKey(event);
      if (!normalizedKey) {
        return; // Ignore if in input field
      }
      
      setTrackedKeys((oldTrackedKeys) => {
        if (oldTrackedKeys && oldTrackedKeys.has(normalizedKey)) {
          event.preventDefault(); // Prevent default browser behavior for mapped keys
          const newKeys = new Map(oldTrackedKeys);
          const k = newKeys.get(normalizedKey);
          if (k) {
            if (!k.toggle) {
              // Only set value to 0 for non-toggle keys
              k.value = 0;
            }
            // For toggle keys, value stays as is (toggled or not)
          }
          return newKeys;
        }
        return oldTrackedKeys;
      });
    },
    [normalizeKey, kbEnabled],
  );

  // Key down Listener - only active when keyboard mode is enabled
  useEffect(() => {
    if (config.dataSource === "keyboard") {
      document.addEventListener("keydown", handleKeyDown);
      return () => {
        document.removeEventListener("keydown", handleKeyDown);
      };
    }
    return undefined;
  }, [handleKeyDown, config.dataSource]);

  // Key up Listener - only active when keyboard mode is enabled
  useEffect(() => {
    if (config.dataSource === "keyboard") {
      document.addEventListener("keyup", handleKeyUp);
      return () => {
        document.removeEventListener("keyup", handleKeyUp);
      };
    }
    return undefined;
  }, [handleKeyUp, config.dataSource]);

  // Reload mapping when selection changes
  useEffect(() => {
    const mapping = keyboardMappings[config.keyboardMapping] ?? kbmapping1;
    setTrackedKeys(buildKeyMap(mapping));
    setCurrentKbMapping(mapping as Record<string, RawKbMap>);
  }, [config.keyboardMapping]);

  // Generate Joy from Keys
  useEffect(() => {
    if (config.dataSource !== "keyboard") {
      return;
    }
    if (!kbEnabled) {
      return;
    }

    // Initialize with fixed array sizes so the raw display is always complete
    const axes: number[] = new Array(6).fill(0);
    const buttons: number[] = new Array(18).fill(0);
    const triggerAxes = new Set<number>();

    // Default trigger axes (L2/R2) to -1 when idle
    axes[4] = -1;
    axes[5] = -1;

    trackedKeys?.forEach((value) => {
      if (value.axis >= 0 && value.direction !== 0 && value.button >= 0) {
        triggerAxes.add(value.axis);
      }
    });

    triggerAxes.forEach((axis) => {
      axes[axis] = -1;
    });

    trackedKeys?.forEach((value) => {
      // Bounds checking for safety
      if (value.button >= 0 && value.button < buttons.length) {
        buttons[value.button] = value.value;
      } else if (value.button >= buttons.length) {
        console.warn(`[POLARIS Joystick] Button index ${value.button} out of bounds (max: ${buttons.length - 1})`);
      }

      if (value.axis >= 0 && value.axis < axes.length && value.direction !== 0) {
        const direction = value.direction > 0 ? 1 : -1;
        if (triggerAxes.has(value.axis)) {
          axes[value.axis] = -1 + 2 * (direction * value.value);
        } else {
          axes[value.axis] = (axes[value.axis] ?? 0) + direction * value.value;
        }
      } else if (value.axis >= axes.length) {
        console.warn(`[POLARIS Joystick] Axis index ${value.axis} out of bounds (max: ${axes.length - 1})`);
      }
    });

    // Only update if values actually changed
    setJoy((prevJoy) => {
      const axesChanged = !prevJoy || axes.some((val, idx) => val !== (prevJoy.axes[idx] ?? 0));
      const buttonsChanged =
        !prevJoy || buttons.some((val, idx) => val !== (prevJoy.buttons[idx] ?? 0));

      if (axesChanged || buttonsChanged) {
        return {
          header: {
            frame_id: config.publishFrameId || "joystick_frame",
            stamp: fromDate(new Date()),
          },
          axes,
          buttons,
        } as Joy;
      }
      return prevJoy;
    });
  }, [config.dataSource, trackedKeys, config.publishFrameId, kbEnabled]);

  // Advertise the topic to publish
  useEffect(() => {
    // Don't allow publish mode when subscribing to a topic
    if (config.dataSource === "sub-joy-topic" && config.publishMode) {
      console.warn("[POLARIS Joystick] Publish mode is not allowed when subscribing to a topic. Disabling publish mode.");
      setConfig((prev) => ({ ...prev, publishMode: false }));
      return;
    }

    setPubTopic((oldTopic) => {
      // Clean up old topic if it exists and is different
      if (oldTopic && oldTopic !== config.pubJoyTopic) {
        try {
          context.unadvertise?.(oldTopic);
        } catch (error) {
          console.error(`[POLARIS Joystick] Failed to unadvertise topic ${oldTopic}:`, error);
        }
      }

      // Advertise new topic if publish mode is enabled
      if (config.publishMode) {
        // Validate topic name
        if (!config.pubJoyTopic || config.pubJoyTopic.trim() === "") {
          console.error("[POLARIS Joystick] Cannot publish: topic name is empty");
          return oldTopic || "";
        }
        
        try {
          context.advertise?.(config.pubJoyTopic, "sensor_msgs/Joy");
          return config.pubJoyTopic;
        } catch (error) {
          console.error(`[POLARIS Joystick] Failed to advertise topic ${config.pubJoyTopic}:`, error);
          return oldTopic || "";
        }
      } else {
        // Unadvertise if publish mode is disabled
        if (oldTopic) {
          try {
            context.unadvertise?.(oldTopic);
          } catch (error) {
            console.error(`[POLARIS Joystick] Failed to unadvertise topic ${oldTopic}:`, error);
          }
        }
        return "";
      }
    });
  }, [config.pubJoyTopic, config.publishMode, config.dataSource, context]);

  // Publish the joy message
  useEffect(() => {
    if (!config.publishMode) {
      return;
    }

    // Safety check: don't publish if we're in subscribe mode
    if (config.dataSource === "sub-joy-topic") {
      return;
    }

    // Validate joy message exists and has required fields
    if (!joy || !joy.axes || !joy.buttons) {
      return;
    }

    // Validate topic is properly advertised
    if (pubTopic && pubTopic === config.pubJoyTopic) {
      try {
        context.publish?.(pubTopic, joy);
      } catch (error) {
        console.error(`[POLARIS Joystick] Failed to publish to topic ${pubTopic}:`, error);
      }
    }
  }, [context, config.pubJoyTopic, config.publishMode, config.dataSource, joy, pubTopic]);

  // Invoke the done callback once the render is complete
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const handleKbSwitch = (event: React.ChangeEvent<HTMLInputElement>) => {
    const enabled = event.target.checked;
    setKbEnabled(enabled);

    // Clear all key values when disabled for safety
    if (!enabled) {
      setTrackedKeys((oldTrackedKeys) => {
        if (!oldTrackedKeys) return oldTrackedKeys;
        const newKeys = new Map(oldTrackedKeys);
        newKeys.forEach((value) => {
          value.value = 0;
          value.toggled = false;
        });
        return newKeys;
      });
    }
  };

  const interactiveCb = useCallback(
    (interactiveJoy: Joy) => {
      if (config.dataSource !== "interactive") {
        return;
      }
      
      // Validate input
      if (!interactiveJoy || !interactiveJoy.axes || !interactiveJoy.buttons) {
        console.error("[POLARIS Joystick] Invalid interactive joy message");
        return;
      }
      
      // Check for reasonable array sizes
      if (interactiveJoy.axes.length > 100 || interactiveJoy.buttons.length > 100) {
        console.error("[POLARIS Joystick] Interactive joy message has excessive array sizes. Ignoring.");
        return;
      }
      
      const tmpJoy = {
        header: {
          frame_id: config.publishFrameId || "joystick_frame",
          stamp: fromDate(new Date()),
        },
        axes: interactiveJoy.axes,
        buttons: interactiveJoy.buttons,
      } as Joy;

      setJoy(tmpJoy);
    },
    [config.publishFrameId, config.dataSource, setJoy],
  );

  useEffect(() => {
    context.saveState(config);
  }, [context, config]);

  // Cleanup effect - unadvertise topic on unmount
  useEffect(() => {
    return () => {
      if (pubTopic) {
        try {
          context.unadvertise?.(pubTopic);
        } catch (error) {
          console.error(`[POLARIS Joystick] Failed to unadvertise topic on cleanup:`, error);
        }
      }
    };
  }, [pubTopic, context]);

  return (
    <div style={{ 
      height: "100%", 
      width: "100%", 
      overflow: "auto", 
      boxSizing: "border-box",
      padding: `${10 * config.uiScale}px`
    }}>
      <div style={{
        display: "flex",
        flexDirection: "column",
        gap: `${8 * config.uiScale}px`
      }}>
        {config.dataSource === "keyboard" ? (
          <FormGroup sx={{ margin: 0 }}>
            <FormControlLabel
              control={<Switch checked={kbEnabled} onChange={handleKbSwitch} />}
              label={`Enable ${
                config.keyboardMapping === "default"
                  ? "Default"
                  : config.keyboardMapping === "keyboard_movement"
                    ? "Keyboard Movement"
                    : config.keyboardMapping === "keyboard_buttons"
                      ? "Keyboard Buttons"
                      : config.keyboardMapping
              }`}
            />
          </FormGroup>
        ) : null}
        {config.layoutName !== "rawjoy" ? (
          <GamepadView
            joy={joy}
            cbInteractChange={interactiveCb}
            layoutName={config.layoutName}
            kbMapping={config.dataSource === "keyboard" ? currentKbMapping : undefined}
            uiScale={config.uiScale}
          />
        ) : null}
        {config.layoutName === "rawjoy" || config.layoutName === "ps4rawjoy" ? (
          <JoyDataDisplay joy={joy} kbMapping={config.dataSource === "keyboard" ? currentKbMapping : undefined} uiScale={config.uiScale} />
        ) : null}
      </div>
    </div>
  );
}

export function initJoyPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<JoyPanel context={context} />);

  // Return a function to run when the panel is removed
  return () => {
    root.unmount();
  };
}
