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

// Hierarchical button section (used in buttonContent)
export type ButtonSectionHierarchical = {
  title: string;
  buttons: VisualButtonMapping[];
  color: string;
};

// Flat button section reference (used for rendering)
export type ButtonSection = {
  title: string;
  buttonIndices: number[];
  color: string;
};

export type ButtonContent = ButtonSectionHierarchical;

export type ButtonPreset = "uuv-settings" | "empty";

const BUTTON_NODE_PREFIX = "button-";
const SECTION_NODE_PREFIX = "section-";

function isButtonSection(item: ButtonContent): item is ButtonSectionHierarchical {
  return "buttons" in item;
}

function parseContentIndex(nodeKey: string | number | undefined): number | undefined {
  if (typeof nodeKey !== "string") {
    return undefined;
  }
  const match = nodeKey.match(/^button-(\d+)$/);
  if (!match) {
    return undefined;
  }
  return Number(match[1]);
}

function parseSectionButtonIndex(path: readonly string[]): { sectionIdx: number; buttonIdx: number } | undefined {
  if (path[0] !== "buttons" || !path[1]?.startsWith(SECTION_NODE_PREFIX)) {
    return undefined;
  }
  const sectionMatch = path[1].match(/^section-(\d+)$/);
  if (!sectionMatch) {
    return undefined;
  }
  const sectionIdx = Number(sectionMatch[1]);
  const buttonIdx = parseContentIndex(path[2]);
  if (buttonIdx === undefined) {
    return undefined;
  }
  return { sectionIdx, buttonIdx };
}

function parseSectionIndex(path: readonly string[]): number | undefined {
  if (path[0] !== "buttons" || typeof path[1] !== "string") {
    return undefined;
  }
  const sectionMatch = path[1].match(/^section-(\d+)$/);
  if (!sectionMatch) {
    return undefined;
  }
  return Number(sectionMatch[1]);
}

export function defaultButtonContent(): ButtonContent[] {
  return [
    {
      title: "Face Buttons",
      buttons: [
        { label: "X", primaryButton: 12, secondaryButton: 2, color: "primary" },
        { label: "Y", primaryButton: 13, secondaryButton: 1, color: "primary" },
        { label: "A", primaryButton: 14, secondaryButton: -1, color: "primary" },
        { label: "B", primaryButton: 15, secondaryButton: -1, color: "primary" },
      ],
      color: "primary",
    },
    {
      title: "Shoulder Buttons",
      buttons: [
        { label: "L", primaryButton: 4, secondaryButton: -1, color: "primary" },
        { label: "R", primaryButton: 5, secondaryButton: -1, color: "primary" },
      ],
      color: "primary",
    },
  ];
}

export function buttonContentFromPreset(preset: ButtonPreset): ButtonContent[] {
  if (preset === "empty") {
    return [];
  }
  if (preset === "uuv-settings") {
    return defaultButtonContent().map((item) => ({
      ...item,
      buttons: [...item.buttons],
    }));
  }
  return [];
}

export function normalizeButtonContent(content: unknown): ButtonContent[] {
  if (!Array.isArray(content)) {
    return defaultButtonContent();
  }

  const sections: ButtonContent[] = [];
  const looseButtons: VisualButtonMapping[] = [];

  for (const item of content) {
    if (item && typeof item === "object" && "buttons" in (item as Record<string, unknown>)) {
      const maybeSection = item as Partial<ButtonSectionHierarchical>;
      sections.push({
        title: String(maybeSection.title ?? `Section ${sections.length + 1}`),
        color: String(maybeSection.color ?? "primary"),
        buttons: Array.isArray(maybeSection.buttons)
          ? maybeSection.buttons.map((button, buttonIdx) => ({
              label: String(button?.label ?? `B${buttonIdx + 1}`),
              primaryButton: Number(button?.primaryButton ?? -1),
              secondaryButton: Number(button?.secondaryButton ?? -1),
              color: "primary",
            }))
          : [],
      });
      continue;
    }

    if (item && typeof item === "object") {
      const maybeButton = item as Partial<VisualButtonMapping>;
      looseButtons.push({
        label: String(maybeButton.label ?? `B${looseButtons.length + 1}`),
        primaryButton: Number(maybeButton.primaryButton ?? -1),
        secondaryButton: Number(maybeButton.secondaryButton ?? -1),
        color: "primary",
      });
    }
  }

  if (looseButtons.length > 0) {
    sections.unshift({
      title: "Buttons",
      color: "primary",
      buttons: looseButtons,
    });
  }

  return sections;
}

// Helper function to flatten buttonContent into a list of all buttons
export function flattenButtonContent(content: ButtonContent[]): VisualButtonMapping[] {
  const flattened: VisualButtonMapping[] = [];
  content.forEach((item) => {
    flattened.push(...item.buttons.map((button) => ({ ...button, color: "primary" })));
  });
  return flattened;
}

// Helper function to build section references from hierarchical content
export function buildSectionReferences(content: ButtonContent[]): ButtonSection[] {
  const sections: ButtonSection[] = [];
  let currentButtonIndex = 0;

  content.forEach((item) => {
    const buttonIndices: number[] = [];
    const itemButtonCount = item.buttons.length;
    for (let i = 0; i < itemButtonCount; i++) {
      buttonIndices.push(currentButtonIndex + i);
    }
    sections.push({
      title: item.title,
      buttonIndices,
      color: "primary",
    });
    currentButtonIndex += itemButtonCount;
  });

  return sections;
}

export type Config = {
  dataSource: string;
  buttonsPreset: ButtonPreset;
  subJoyTopic: string;
  gamepadId: number;
  publishMode: boolean;
  pubJoyTopic: string;
  publishFrameId: string;
  layoutName: string;
  mapping_name: string;
  keyboardMapping: string;
  uiScale: number;
  buttonContent: ButtonContent[];
};

export function settingsActionReducer(prevConfig: Config, action: SettingsTreeAction): Config {
  return produce(prevConfig, (draft) => {
    if (action.action !== "update") {
      const { id, path } = action.payload;

      // Handle top-level button/section actions
      if (path[0] === "buttons") {
        const sectionIdx = parseSectionIndex(path);
        const sectionButtonIdx = parseSectionButtonIndex(path);

        // Add section at top level
        if (id === "add-section" && path.length === 1) {
          draft.buttonContent.push({
            title: `Section ${draft.buttonContent.length + 1}`,
            buttons: [],
            color: "primary",
          });
          return;
        }

        // Add button to a section
        if (id === "add-button" && sectionIdx !== undefined && path.length === 2) {
          const section = draft.buttonContent[sectionIdx];
          if (section && isButtonSection(section)) {
            section.buttons.push({
              label: `B${section.buttons.length + 1}`,
              primaryButton: -1,
              secondaryButton: -1,
              color: "primary",
            });
          }
          return;
        }

        // Remove top-level content (button or section)
        // Remove button from section
        if (id === "remove" && sectionButtonIdx && path.length >= 3) {
          const section = draft.buttonContent[sectionButtonIdx.sectionIdx];
          if (section && isButtonSection(section)) {
            section.buttons.splice(sectionButtonIdx.buttonIdx, 1);
          }
          return;
        }

        // Remove section
        if (id === "remove" && sectionIdx !== undefined && path.length === 2) {
          draft.buttonContent.splice(sectionIdx, 1);
          return;
        }

        // Move up section
        if (id === "moveUp" && sectionIdx !== undefined && path.length === 2 && sectionIdx > 0) {
          const temp = draft.buttonContent[sectionIdx - 1]!;
          draft.buttonContent[sectionIdx - 1] = draft.buttonContent[sectionIdx]!;
          draft.buttonContent[sectionIdx] = temp;
          return;
        }

        // Move down section
        if (
          id === "moveDown" &&
          sectionIdx !== undefined &&
          path.length === 2 &&
          sectionIdx < draft.buttonContent.length - 1
        ) {
          const temp = draft.buttonContent[sectionIdx + 1]!;
          draft.buttonContent[sectionIdx + 1] = draft.buttonContent[sectionIdx]!;
          draft.buttonContent[sectionIdx] = temp;
          return;
        }

        // Move up within section
        if (id === "moveUp" && sectionButtonIdx && path.length >= 3 && sectionButtonIdx.buttonIdx > 0) {
          const section = draft.buttonContent[sectionButtonIdx.sectionIdx];
          if (section && isButtonSection(section)) {
            const temp = section.buttons[sectionButtonIdx.buttonIdx - 1]!;
            section.buttons[sectionButtonIdx.buttonIdx - 1] =
              section.buttons[sectionButtonIdx.buttonIdx]!;
            section.buttons[sectionButtonIdx.buttonIdx] = temp;
          }
          return;
        }

        // Move down within section
        if (
          id === "moveDown" &&
          sectionButtonIdx &&
          path.length >= 3 &&
          sectionButtonIdx.buttonIdx < ((draft.buttonContent[sectionButtonIdx.sectionIdx] as ButtonSectionHierarchical)?.buttons.length ?? 0) - 1
        ) {
          const section = draft.buttonContent[sectionButtonIdx.sectionIdx];
          if (section && isButtonSection(section)) {
            const temp = section.buttons[sectionButtonIdx.buttonIdx + 1]!;
            section.buttons[sectionButtonIdx.buttonIdx + 1] =
              section.buttons[sectionButtonIdx.buttonIdx]!;
            section.buttons[sectionButtonIdx.buttonIdx] = temp;
          }
          return;
        }
      }
      return;
    }

    if (action.action === "update") {
      const { path, value } = action.payload;
      const pathStr = path.join(".");

      // Handle compact mode toggle conversion
      if (pathStr.includes("uiScale") && typeof value === "boolean") {
        draft.uiScale = value ? 0.6 : 1;
        return;
      }

      if (pathStr.includes("buttonsPreset")) {
        const preset = value === "empty" ? "empty" : "uuv-settings";
        draft.buttonsPreset = preset;
        draft.buttonContent = buttonContentFromPreset(preset);
        return;
      }

      // Handle updates to top-level items
      if (path[0] === "buttons") {
        const sectionIdx = parseSectionIndex(path);
        const sectionButtonIdx = parseSectionButtonIndex(path);

        // Update section title
        if (sectionIdx !== undefined && path.length >= 3 && path[2] === "title") {
          const item = draft.buttonContent[sectionIdx];
          if (item && isButtonSection(item)) {
            item.title = String(value);
          }
          return;
        }

        // Update button properties within section
        if (sectionButtonIdx) {
          const section = draft.buttonContent[sectionButtonIdx.sectionIdx];
          if (section && isButtonSection(section)) {
            const button = section.buttons[sectionButtonIdx.buttonIdx];
            if (button) {
              if (path[3] === "label") {
                button.label = String(value);
              } else if (path[3] === "primaryButton") {
                button.primaryButton = Number(value);
              } else if (path[3] === "secondaryButton") {
                button.secondaryButton = Number(value);
              } else if (path[3] === "color") {
                button.color = "primary";
              }
            }
          }
          return;
        }
      }

      // Fallback for other updates
      if (
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

  if (config.dataSource === "buttons") {
    dataSourceFields.buttonsPreset = {
      label: "Buttons Preset",
      input: "select",
      value: config.buttonsPreset,
      help: "Prefill button mapping from a preset; you can still edit afterward",
      options: [
        {
          label: "UUV Settings",
          value: "uuv-settings",
        },
        {
          label: "Empty",
          value: "empty",
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
          id: "add-section",
          type: "action",
          label: "Add Section",
        } satisfies SettingsTreeNodeAction,
      ],
      children: {},
    };
  }
  const buttonChildren = settings.buttons?.children;

  if (!buttonChildren) {
    return settings;
  }

  // Render all top-level content (buttons and sections)
  config.buttonContent.forEach((item, idx) => {
    const contentKey = `${SECTION_NODE_PREFIX}${idx}`;
    const actions: SettingsTreeNodeAction[] = [];

    // Add Move Up action if not first
    if (idx > 0) {
      actions.push({
        id: "moveUp",
        type: "action",
        label: "Move Up",
      });
    }

    // Add Move Down action if not last
    if (idx < config.buttonContent.length - 1) {
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

    // Render section
    const sectionActions: SettingsTreeNodeAction[] = [
      {
        id: "add-button",
        type: "action",
        label: "Add Button",
      },
      ...actions,
    ];

    buttonChildren[contentKey] = {
      label: item.title,
      fields: {
        title: {
          label: "Title",
          input: "string",
          value: item.title,
          disabled: config.dataSource !== "buttons",
        },
      },
      actions: sectionActions,
      children: {},
    };

    // Render buttons inside section
    const sectionButtonChildren = buttonChildren[contentKey]!.children!;
    item.buttons.forEach((button, buttonIdx) => {
      const buttonKey = `${BUTTON_NODE_PREFIX}${buttonIdx}`;
      const buttonActions: SettingsTreeNodeAction[] = [];

      if (buttonIdx > 0) {
        buttonActions.push({
          id: "moveUp",
          type: "action",
          label: "Move Up",
        });
      }

      if (buttonIdx < item.buttons.length - 1) {
        buttonActions.push({
          id: "moveDown",
          type: "action",
          label: "Move Down",
        });
      }

      buttonActions.push({
        id: "remove",
        type: "action",
        label: "Remove",
      });

      sectionButtonChildren[buttonKey] = {
        label: button.label?.trim() ? button.label : `Button ${buttonIdx + 1}`,
        fields: {
          label: {
            label: "Label",
            input: "string",
            value: button.label ?? `B${buttonIdx + 1}`,
            disabled: config.dataSource !== "buttons",
          },
          primaryButton: {
            label: "Primary",
            input: "select",
            value: String(button.primaryButton ?? -1),
            disabled: config.dataSource !== "buttons",
            options: PS_BUTTON_OPTIONS,
          },
          secondaryButton: {
            label: "Secondary",
            input: "select",
            value: String(button.secondaryButton ?? -1),
            disabled: config.dataSource !== "buttons",
            options: PS_BUTTON_OPTIONS,
          },
        },
        actions: buttonActions,
      };
    });
  });

  return settings;
}
