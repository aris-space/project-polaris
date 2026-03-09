import {
  Topic,
  SettingsTreeNodes,
  SettingsTreeFields,
  SettingsTreeAction,
  SettingsTreeNodeAction,
} from "@foxglove/studio";
import { produce } from "immer";
import * as _ from "lodash-es";

export const PS_BUTTON_OPTIONS: { label: string; value: string }[] = [
  { label: "None", value: "-1" },
  { label: "X (0)", value: "0" },
  { label: "O (1)", value: "1" },
  { label: "Square (2)", value: "2" },
  { label: "Triangle (3)", value: "3" },
  { label: "L1 (4)", value: "4" },
  { label: "R1 (5)", value: "5" },
  { label: "L2 (6)", value: "6" },
  { label: "R2 (7)", value: "7" },
  { label: "Share (8)", value: "8" },
  { label: "Options (9)", value: "9" },
  { label: "L3 (10)", value: "10" },
  { label: "R3 (11)", value: "11" },
  { label: "D-Pad Up (12)", value: "12" },
  { label: "D-Pad Down (13)", value: "13" },
  { label: "D-Pad Left (14)", value: "14" },
  { label: "D-Pad Right (15)", value: "15" },
  { label: "PS (16)", value: "16" },
  { label: "Touchpad (17)", value: "17" },
];

export type VisualButtonMapping = {
  label: string;
  primaryButton: number;
  secondaryButton: number;
  color: string;
};

const BUTTON_NODE_PREFIX = "button-";

function parseButtonIndex(nodeKey: string | number | undefined): number | undefined {
  if (typeof nodeKey !== "string") {
    return undefined;
  }
  const match = nodeKey.match(/^button-(\d+)$/);
  if (!match) {
    return undefined;
  }
  return Number(match[1]);
}

function parseButtonIndexFromPath(path: readonly string[]): number | undefined {
  if (path[0] !== "buttons") {
    return undefined;
  }
  return parseButtonIndex(path[1]);
}

export function defaultVisualButtons(): VisualButtonMapping[] {
  return [
    { label: "X", primaryButton: 12, secondaryButton: 2, color: "primary" },
    { label: "Y", primaryButton: 13, secondaryButton: 1, color: "secondary" },
    { label: "A", primaryButton: 14, secondaryButton: -1, color: "info" },
    { label: "B", primaryButton: 15, secondaryButton: -1, color: "warning" },
    { label: "L", primaryButton: 4, secondaryButton: -1, color: "error" },
    { label: "R", primaryButton: 5, secondaryButton: -1, color: "success" },
  ];
}

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
  visualButtons: VisualButtonMapping[];
};

export function settingsActionReducer(prevConfig: Config, action: SettingsTreeAction): Config {
  return produce(prevConfig, (draft) => {
    if (action.action !== "update") {
      const { id, path } = action.payload;
      const nodeKey = path[0];
      const buttonIndex = parseButtonIndexFromPath(path);

      if (id === "add" && nodeKey === "buttons") {
        draft.visualButtons.push({
          label: `B${draft.visualButtons.length + 1}`,
          primaryButton: -1,
          secondaryButton: -1,
          color: "primary",
        });
        return;
      }

      if (id === "remove" && buttonIndex != undefined) {
        draft.visualButtons.splice(buttonIndex, 1);
        return;
      }

      if (id === "moveUp" && buttonIndex != undefined && buttonIndex > 0) {
        const temp = draft.visualButtons[buttonIndex - 1];
        draft.visualButtons[buttonIndex - 1] = draft.visualButtons[buttonIndex]!;
        draft.visualButtons[buttonIndex] = temp!;
        return;
      }

      if (id === "moveDown" && buttonIndex != undefined && buttonIndex < draft.visualButtons.length - 1) {
        const temp = draft.visualButtons[buttonIndex + 1];
        draft.visualButtons[buttonIndex + 1] = draft.visualButtons[buttonIndex]!;
        draft.visualButtons[buttonIndex] = temp!;
        return;
      }
      return;
    }

    if (action.action === "update") {
      const { path, value } = action.payload;
      const pathStr = path.join(".");
      const buttonIndex = parseButtonIndexFromPath(path);
      const buttonField = path[2];

      // Handle compact mode toggle conversion
      if (pathStr.includes("uiScale") && typeof value === "boolean") {
        draft.uiScale = value ? 0.6 : 1;
      } else if (buttonIndex != undefined) {
        const target = draft.visualButtons[buttonIndex];
        if (!target) {
          return;
        }

        if (buttonField === "label") {
          target.label = String(value);
        } else if (buttonField === "primaryButton") {
          target.primaryButton = Number(value);
        } else if (buttonField === "secondaryButton") {
          target.secondaryButton = Number(value);
        } else if (buttonField === "color") {
          target.color = String(value);
        }
      } else if (
        pathStr.includes("gamepadId") ||
        pathStr.includes("primaryButton") ||
        pathStr.includes("secondaryButton")
      ) {
        _.set(draft, path.slice(1), Number(value));
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
        {
          label: "Buttons",
          value: "buttons",
        },
      ],
    },
  };

  if (config.dataSource === "sub-joy-topic") {
    dataSourceFields.subJoyTopic = {
      label: "Subsc. Joy Topic",
      input: "select",
      value: config.subJoyTopic,
      help: "Select ROS Joy topic to monitor",
      options: (topics ?? [])
        .filter((topic) => topic.datatype === "sensor_msgs/msg/Joy")
        .map((topic) => ({
          label: topic.name,
          value: topic.name,
        })),
    };
  }

  if (config.dataSource === "gamepad") {
    dataSourceFields.gamepadId = {
      label: "Gamepad ID",
      input: "select",
      value: config.gamepadId.toString(),
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
    };
  }

  if (config.dataSource === "keyboard") {
    dataSourceFields.keyboardMapping = {
      label: "KB->Joy Mapping",
      input: "select",
      value: config.keyboardMapping,
      help: "Select how keyboard keys map to Joy messages",
      options: [
        {
          label: "Keyboard Movement",
          value: "keyboard_movement",
        },
        {
          label: "Keyboard Buttons",
          value: "keyboard_buttons",
        },
      ],
    };
  }

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

  if (config.dataSource === "buttons") {
    settings.buttons = {
      label: "Buttons Mapping",
      actions: [
        {
          id: "add",
          type: "action",
          label: "Add Button",
        } satisfies SettingsTreeNodeAction,
      ],
      children: {},
    };
  }
  const buttonChildren = settings.buttons?.children;

  if (!buttonChildren) {
    return settings;
  }

  config.visualButtons.forEach((mapping, idx) => {
    const actions: SettingsTreeNodeAction[] = [];

    // Add Move Up action if not first button
    if (idx > 0) {
      actions.push({
        id: "moveUp",
        type: "action",
        label: "Move Up",
      });
    }

    // Add Move Down action if not last button
    if (idx < config.visualButtons.length - 1) {
      actions.push({
        id: "moveDown",
        type: "action",
        label: "Move Down",
      });
    }

    // Add Remove action
    actions.push({
      id: "remove",
      type: "action",
      label: "Remove",
    });

    buttonChildren[`${BUTTON_NODE_PREFIX}${idx}`] = {
      label: mapping.label?.trim() ? mapping.label : `Button ${idx + 1}`,
      fields: {
        label: {
          label: "Label",
          input: "string",
          value: mapping.label ?? `B${idx + 1}`,
          disabled: config.dataSource !== "buttons",
        },
        color: {
          label: "Color",
          input: "select",
          value: mapping.color ?? "primary",
          disabled: config.dataSource !== "buttons",
          options: [
            { label: "Blue", value: "primary" },
            { label: "Purple", value: "secondary" },
            { label: "Green", value: "success" },
            { label: "Red", value: "error" },
            { label: "Light Blue", value: "info" },
            { label: "Orange", value: "warning" },
          ],
        },
        primaryButton: {
          label: "Primary",
          input: "select",
          value: String(mapping.primaryButton ?? -1),
          disabled: config.dataSource !== "buttons",
          options: PS_BUTTON_OPTIONS,
        },
        secondaryButton: {
          label: "Secondary",
          input: "select",
          value: String(mapping.secondaryButton ?? -1),
          disabled: config.dataSource !== "buttons",
          options: PS_BUTTON_OPTIONS,
        },
      },
      actions,
    };
  });

  return settings;
}
