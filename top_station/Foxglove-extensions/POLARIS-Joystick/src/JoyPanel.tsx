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
    partialConfig.pubJoyTopic ??= "/joy";
    partialConfig.publishMode ??= false;
    partialConfig.publishFrameId ??= "";
    partialConfig.dataSource ??= "sub-joy-topic";
    partialConfig.displayMode ??= "auto";
    partialConfig.debugGamepad ??= false;
    partialConfig.layoutName ??= "ps4";
    partialConfig.mapping_name ??= "TODO";
    partialConfig.keyboardMapping ??= "default";
    partialConfig.gamepadId ??= 0;
    return partialConfig as Config;
  });

  const settingsActionHandler = useCallback(
    (action: SettingsTreeAction) => {
      setConfig((prevConfig) => settingsActionReducer(prevConfig, action));
    },
    [setConfig],
  );

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
      const tmpMsg = {
        header: {
          stamp: latestJoy.header.stamp,
          frame_id: config.publishFrameId,
        },
        axes: Array.from(latestJoy.axes),
        buttons: Array.from(latestJoy.buttons),
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
            frame_id: config.publishFrameId,
            // eslint-disable-next-line no-warning-comments
            // TODO: /clock
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
      const normalizedKey = normalizeKey(event);
      setTrackedKeys((oldTrackedKeys) => {
        if (oldTrackedKeys && oldTrackedKeys.has(normalizedKey)) {
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
    [normalizeKey],
  );

  const handleKeyUp = useCallback(
    (event: KeyboardEvent) => {
      const normalizedKey = normalizeKey(event);
      setTrackedKeys((oldTrackedKeys) => {
        if (oldTrackedKeys && oldTrackedKeys.has(normalizedKey)) {
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
    [normalizeKey],
  );

  // Key down Listener
  useEffect(() => {
    document.addEventListener("keydown", handleKeyDown);
    return () => {
      document.removeEventListener("keydown", handleKeyDown);
    };
  }, [handleKeyDown]);

  // Key up Listener
  useEffect(() => {
    document.addEventListener("keyup", handleKeyUp);
    return () => {
      document.removeEventListener("keyup", handleKeyUp);
    };
  }, [handleKeyUp]);

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

    // Initialize with proper array sizes to prevent flickering
    const axes: number[] = [0, 0, 0, 0]; // 4 axes for sticks
    const buttons: number[] = [];

    trackedKeys?.forEach((value) => {
      if (value.button >= 0) {
        while (buttons.length <= value.button) {
          buttons.push(0);
        }
        buttons[value.button] = value.value;
      } else if (value.axis >= 0 && value.direction !== 0) {
        while (axes.length <= value.axis) {
          axes.push(0);
        }
        // Safe to index because we've grown the array to accommodate
        const direction = value.direction > 0 ? 1 : -1;
        axes[value.axis] = (axes[value.axis] ?? 0) + direction * value.value;
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
            frame_id: config.publishFrameId,
            // eslint-disable-next-line no-warning-comments
            // TODO: /clock
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
    if (config.publishMode) {
      setPubTopic((oldTopic) => {
        if (config.publishMode) {
          if (oldTopic) {
            context.unadvertise?.(oldTopic);
          }
          context.advertise?.(config.pubJoyTopic, "sensor_msgs/Joy");
          return config.pubJoyTopic;
        } else {
          if (oldTopic) {
            context.unadvertise?.(oldTopic);
          }
          return "";
        }
      });
    }
  }, [config.pubJoyTopic, config.publishMode, context]);

  // Publish the joy message
  useEffect(() => {
    if (!config.publishMode) {
      return;
    }

    if (pubTopic && pubTopic === config.pubJoyTopic) {
      context.publish?.(pubTopic, joy);
    }
  }, [context, config.pubJoyTopic, config.publishMode, joy, pubTopic]);

  // Invoke the done callback once the render is complete
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  const handleKbSwitch = (event: React.ChangeEvent<HTMLInputElement>) => {
    setKbEnabled(event.target.checked);

    // eslint-disable-next-line no-warning-comments
    // TODO Clear key values when disabled
    // setTrackedKeys((oldTrackedKeys) => {
    //   const newKeys = new Map(oldTrackedKeys);
    //   newKeys.forEach((value, key, map) => {
    //     const k = map.get(key);
    //     if (k) {
    //       k.value = 0;
    //     }
    //   });
    //   return newKeys;
    // });
  };

  const interactiveCb = useCallback(
    (interactiveJoy: Joy) => {
      if (config.dataSource !== "interactive") {
        return;
      }
      const tmpJoy = {
        header: {
          frame_id: config.publishFrameId,
          // eslint-disable-next-line no-warning-comments
          // TODO: /clock
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

  return (
    <div>
      {config.dataSource === "keyboard" ? (
        <FormGroup>
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
      {config.displayMode === "auto" ? <SimpleButtonView joy={joy} /> : null}
      {config.displayMode === "custom" && config.layoutName !== "rawjoy" ? (
        <GamepadView
          joy={joy}
          cbInteractChange={interactiveCb}
          layoutName={config.layoutName}
          kbMapping={config.dataSource === "keyboard" ? currentKbMapping : undefined}
        />
      ) : null}
      {config.displayMode === "custom" || config.layoutName === "rawjoy" ? <JoyDataDisplay joy={joy} /> : null}
      {/* {config.debugGamepad ? <GamepadDebug gamepads={gamepads} /> : null} */}
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
