import { Topic, SettingsTreeNodes, SettingsTreeFields, SettingsTreeAction } from "@foxglove/studio";
import { produce } from "immer";
import * as _ from "lodash-es";
import packageJson from "../package.json";

export type Config = {
  dataSource: string;
  subJoyTopic: string;
  gamepadId: number;
  publishMode: boolean;
  pubJoyTopic: string;
  publishFrameId: string;
  displayMode: string;
  debugGamepad: boolean;
  layoutName: string;
  mapping_name: string;
  keyboardMapping: string;
};

export function settingsActionReducer(prevConfig: Config, action: SettingsTreeAction): Config {
  return produce(prevConfig, (draft) => {
    if (action.action === "update") {
      const { path, value } = action.payload;
      _.set(draft, path.slice(1), value);
    }
  });
}

export function buildSettingsTree(config: Config, topics?: readonly Topic[]): SettingsTreeNodes {
  const dataSourceFields: SettingsTreeFields = {
    dataSource: {
      label: "Data Source",
      input: "select",
      value: config.dataSource,
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
      options: (topics ?? [])
        .filter((topic) => topic.datatype === "sensor_msgs/msg/Joy")
        .map((topic) => ({
          label: topic.name,
          value: topic.name,
        })),
      // error: (!config.topic ? "Topic name is empty" : null),
    },
    gamepadId: {
      label: "Gamepad ID",
      input: "select",
      value: config.gamepadId.toString(),
      disabled: config.dataSource !== "gamepad",
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
        {
          label: "TODO Make this auto populate",
          value: "3",
        },
      ],
    },
    gamepadMapping: {
      label: "GP->Joy Mapping",
      input: "select",
      value: "default",
      disabled: config.dataSource !== "gamepad",
      options: [
        {
          label: "Default",
          value: "default",
        },
        {
          label: "TODO Make selectable",
          value: "todo",
        },
      ],
    },
    keyboardMapping: {
      label: "KB->Joy Mapping",
      input: "select",
      value: config.keyboardMapping,
      disabled: config.dataSource !== "keyboard",
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
      // eslint-disable-next-line no-warning-comments
      // TODO also need to force publish mode to false when in sub mode
      disabled: config.dataSource === "sub-joy-topic",
    },
    pubJoyTopic: {
      label: "Pub Joy Topic",
      input: "string",
      value: config.pubJoyTopic,
    },
    publishFrameId: {
      label: "Joy Frame ID",
      input: "string",
      value: config.publishFrameId,
    },
  };
  const displayFields: SettingsTreeFields = {
    displayMode: {
      label: "Display Mode",
      input: "select",
      value: config.displayMode,
      options: [
        {
          label: "Auto-Generated",
          value: "auto",
        },
        {
          label: "Custom Display",
          value: "custom",
        },
      ],
    },
    layoutName: {
      label: "Layout",
      input: "select",
      disabled: config.displayMode === "auto",
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
          label: "Raw Joy",
          value: "rawjoy",
        },
      ],
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
    debugGamepad: {
      label: "Debug Gamepad",
      input: "boolean",
      value: config.debugGamepad,
    },
  };

  const aboutFields: SettingsTreeFields = {
    version: {
      label: "Version",
      input: "string",
      value: packageJson.version,
      readonly: true,
    },
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
    about: {
      label: "About",
      fields: aboutFields,
    },
  };

  return settings;
}
