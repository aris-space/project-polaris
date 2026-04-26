import { fontSizes, functions } from "./common/index.js";

const defaultSettings = {
  data: {
    label: "Data",
    topic: "",
  },
  display: {
    label: "Display",
    title: "",
    unit: "",
    fontSize: "auto",
    align: "center",
    bold: true,
    italic: false,
    fontColor: "#ffffff",
    useThresholdBackground: false,
    lowerThreshold: 0,
    upperThreshold: 100,
    belowThresholdColor: "#2e7d32",
    betweenThresholdColor: "#f9a825",
    aboveThresholdColor: "#c62828",
  },
  numerical: {
    label: "Numerical",
    precision: 0,
    function: "none",
  },
  background: {
    label: "Background Status",
    useThresholdColors: false,
    autoTextColorOnThresholds: true,
    normalColor: "#121212",
    naColor: "#121212",
    lowerThreshold: 0,
    upperThreshold: 100,
    belowColor: "#2e7d32",
    betweenColor: "#f9a825",
    aboveColor: "#c62828",
    fallbackColor: "#121212",
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
            supportsMathModifiers: true,
          },
        },
      },
      display: {
        label: state.display.label,
        // renamable: true,
        // visible: state.value.display.visible,
        icon: "Cells",
        fields: {
          title: {
            label: "Title",
            input: "string",
            value: state.display.title,
          },
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
      background: {
        label: state.background.label,
        icon: "Palette",
        fields: {
          normalColor: {
            label: "Background (Threshold Mode Off)",
            input: "rgb",
            value: state.background.normalColor,
          },
          useThresholdColors: {
            label: "Enable Threshold Mode",
            input: "boolean",
            value: state.background.useThresholdColors,
          },
          lowerThreshold: {
            label: "Low Limit",
            input: "number",
            value: state.background.lowerThreshold,
          },
          upperThreshold: {
            label: "High Limit",
            input: "number",
            value: state.background.upperThreshold,
          },
          belowColor: {
            label: "Color: Below Low",
            input: "rgb",
            value: state.background.belowColor,
          },
          betweenColor: {
            label: "Color: In Range",
            input: "rgb",
            value: state.background.betweenColor,
          },
          aboveColor: {
            label: "Color: Above High",
            input: "rgb",
            value: state.background.aboveColor,
          },
          naColor: {
            label: "No Data (N/A)",
            input: "rgb",
            value: state.background.naColor,
          },
          autoTextColorOnThresholds: {
            label: "Auto Text Contrast",
            input: "boolean",
            value: state.background.autoTextColorOnThresholds,
          },
        },
      },
    },
  });
};

export { defaultSettings, updateSettingsEditor };
