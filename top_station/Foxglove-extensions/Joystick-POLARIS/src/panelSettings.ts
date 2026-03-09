import { Topic, SettingsTreeNodes, SettingsTreeFields, SettingsTreeAction } from "@foxglove/studio";
import { produce } from "immer";
import * as _ from "lodash-es";

export type Config = {
  dataSource: string;
  subJoyTopic: string;
  gamepadId: number;
  publishMode: boolean;
  pubJoyTopic: string;
  publishFrameId: string;
  layoutName: string;
  mapping_name: string;
  keyboardMapping: string;
  uiScale: number;
};

export function settingsActionReducer(prevConfig: Config, action: SettingsTreeAction): Config {
  return produce(prevConfig, (draft) => {
    if (action.action === "update") {
      const { path, value } = action.payload;
      const pathStr = path.join(".");
      // Handle compact mode toggle conversion
      if (pathStr.includes("uiScale") && typeof value === "boolean") {
        draft.uiScale = value ? 0.6 : 1;
      } else {
        _.set(draft, path.slice(1), value);
      }
    }
  });
}

export function buildSettingsTree(config: Config, topics?: readonly Topic[]): SettingsTreeNodes {
  const dataSourceFields: SettingsTreeFields = {
    dataSource: {
      label: "Data Source",
      input: "select",
      value: config.dataSource,
        help: "Select where joystick data comes from",
      options: [
        {
          label: "Subscribed Joy Topic",
          value: "sub-joy-topic",
        },
        {
          label: "Gamepad",
          value: "gamepad",
        },
        {
          label: "Interactive",
          value: "interactive",
        },
        {
          label: "Keyboard",
          value: "keyboard",
        },
      ],
    },
    subJoyTopic: {
      label: "Subsc. Joy Topic",
      input: "select",
      value: config.subJoyTopic,
      disabled: config.dataSource !== "sub-joy-topic",
        help: "Select ROS Joy topic to monitor",
      options: (topics ?? [])
        .filter((topic) => topic.datatype === "sensor_msgs/msg/Joy")
        .map((topic) => ({
          label: topic.name,
          value: topic.name,
        })),
    },
    gamepadId: {
      label: "Gamepad ID",
      input: "select",
      value: config.gamepadId.toString(),
      disabled: config.dataSource !== "gamepad",
        help: "Select which gamepad to use (0 is primary)",
      options: [
        {
          label: "0",
          value: "0",
        },
        {
          label: "1",
          value: "1",
        },
        {
          label: "2",
          value: "2",
        },
      ],
    },
    keyboardMapping: {
      label: "KB->Joy Mapping",
      input: "select",
      value: config.keyboardMapping,
      disabled: config.dataSource !== "keyboard",
        help: "Select how keyboard keys map to Joy messages",
      options: [
        {
          label: "Default",
          value: "default",
        },
        {
          label: "Keyboard Movement",
          value: "keyboard_movement",
        },
        {
          label: "Keyboard Buttons",
          value: "keyboard_buttons",
        },
      ],
    },
  };
  const publishFields: SettingsTreeFields = {
    publishMode: {
      label: "Publish Mode",
      input: "boolean",
      value: config.publishMode,
      disabled: config.dataSource === "sub-joy-topic",
      help: "Publish mode is disabled when subscribing to a topic for safety",
    },
    pubJoyTopic: {
      label: "Pub Joy Topic",
      input: "string",
      value: config.pubJoyTopic,
      error: config.publishMode && (!config.pubJoyTopic || config.pubJoyTopic.trim() === "") 
        ? "Topic name cannot be empty in publish mode" 
        : undefined,
      help: "ROS topic name to publish Joy messages to",
    },
    publishFrameId: {
      label: "Joy Frame ID",
      input: "string",
      value: config.publishFrameId,
      error: config.publishMode && (!config.publishFrameId || config.publishFrameId.trim() === "")
        ? "Frame ID cannot be empty"
        : undefined,
      help: "TF frame_id for published Joy messages (default: joystick_frame)",
    },
  };
  const displayFields: SettingsTreeFields = {
    layoutName: {
      label: "Layout",
      input: "select",
      value: config.layoutName,
      options: [
        {
          label: "Empty",
          value: "empty",
        },
        {
          label: "PS4",
          value: "ps4",
        },
        {
          label: "PS4 + Raw Joy",
          value: "ps4rawjoy",
        },
        {
          label: "Raw Joy",
          value: "rawjoy",
        },
      ],
    },
    uiScale: {
      label: "Compact Mode",
      input: "boolean",
      value: config.uiScale !== 1,
    },

    // mapping: {
    //   label: "Mapping",
    //   input: "select",
    //   value: config.mapping_name,
    //   disabled: true, // config.displayMode === "auto",
    //   options: [
    //     {
    //       label: "Custom",
    //       value: "custom",
    //     },
    //   ],
    // },
  };

  const settings: SettingsTreeNodes = {
    dataSource: {
      label: "Data Source",
      fields: dataSourceFields,
    },
    publish: {
      label: "Publish",
      fields: publishFields,
    },
    display: {
      label: "Display",
      fields: displayFields,
    },
  };

  return settings;
}
