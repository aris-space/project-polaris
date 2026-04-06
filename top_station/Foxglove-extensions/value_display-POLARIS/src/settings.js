import { fontSizes, functions } from "./common/index.js";

const defaultSettings = {
  data: {
    label: "Data",
    topic: "",
  },
  display: {
    label: "Display",
    unit: "",
    fontSize: "auto",
    align: "center",
    bold: true,
    italic: false,
    fontColor: "#ffffff",
    backgroundColor: "#121212",
  },
  numerical: {
    label: "Numerical",
    precision: 0,
    function: "none",
  },
};

const updateSettingsEditor = (context, state, settingsActionHandler) => {
  context.updatePanelSettingsEditor({
    actionHandler: settingsActionHandler,
    nodes: {
      data: {
        label: state.data.label,
        // renamable: true,
        // visible: state.value.data.visible,
        icon: "Settings",
        fields: {
          topic: {
            label: "Topic",
            input: "messagepath",
            value: state.data.topic,
          },
        },
      },
      display: {
        label: state.display.label,
        // renamable: true,
        // visible: state.value.display.visible,
        icon: "Cells",
        fields: {
          unit: {
            label: "Unit",
            input: "string",
            value: state.display.unit,
          },
          fontSize: {
            label: "Font Size",
            input: "select",
            options: fontSizes,
            value: state.display.fontSize,
          },
          align: {
            label: "Align",
            input: "toggle",
            options: [
              { value: "left", label: "Left" },
              { value: "center", label: "Center" },
              { value: "right", label: "Right" },
            ],
            value: state.display.align,
          },
          bold: {
            label: "Bold",
            input: "boolean",
            value: state.display.bold,
          },
          italic: {
            label: "Italic",
            input: "boolean",
            value: state.display.italic,
          },
          fontColor: {
            label: "Font Color",
            input: "rgb",
            value: state.display.fontColor,
          },
          backgroundColor: {
            label: "Background Color",
            input: "rgb",
            value: state.display.backgroundColor,
          },
        },
      },
      numerical: {
        label: state.numerical.label,
        // renamable: true,
        // visible: state.value.numerical.visible,
        icon: "PrecisionManufacturing",
        fields: {
          precision: {
            label: "Precision",
            input: "number",
            value: state.numerical.precision,
          },
          function: {
            label: "Function",
            input: "select",
            options: functions,
            value: state.numerical.function,
          },
        },
      },
    },
  });
};

export { defaultSettings, updateSettingsEditor };
