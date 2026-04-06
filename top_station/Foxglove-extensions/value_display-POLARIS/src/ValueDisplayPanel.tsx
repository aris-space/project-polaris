import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { merge, set } from "lodash";
import { ReactElement, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import { applyFunction, parseValue, splitTopic } from "./common/index.js";
import { defaultSettings, updateSettingsEditor } from "./settings.js";

type PanelState = typeof defaultSettings;

function formatNumberWithApostropheThousands(value: number, precision: number): string {
  const fixed = value.toFixed(Math.max(0, precision));
  const [integerPartRaw, fractionalPart] = fixed.split(".");
  const integerPart = integerPartRaw ?? "0";

  const sign = integerPart.startsWith("-") ? "-" : "";
  const integerDigits = sign ? integerPart.slice(1) : integerPart;
  const groupedInteger = integerDigits.replace(/\B(?=(\d{3})+(?!\d))/g, "'");

  return fractionalPart !== undefined
    ? `${sign}${groupedInteger}.${fractionalPart}`
    : `${sign}${groupedInteger}`;
}

function getReadableTextColor(backgroundColor: string): string {
  const hex = backgroundColor.trim().replace("#", "");
  if (!/^[0-9a-fA-F]{6}$/.test(hex)) {
    return "#ffffff";
  }

  const r = parseInt(hex.slice(0, 2), 16);
  const g = parseInt(hex.slice(2, 4), 16);
  const b = parseInt(hex.slice(4, 6), 16);
  const luminance = (0.299 * r + 0.587 * g + 0.114 * b) / 255;
  return luminance > 0.6 ? "#111111" : "#ffffff";
}

function getAutoFontSizePx({
  text,
  width,
  height,
  bold,
  italic,
}: {
  text: string;
  width: number;
  height: number;
  bold: boolean;
  italic: boolean;
}): number {
  if (width <= 0 || height <= 0) {
    return 16;
  }

  const horizontalPaddingPx = 16;
  const widthSafetyMarginPx = 6;
  const availableWidth = Math.max(1, width - horizontalPaddingPx - widthSafetyMarginPx);
  const maxByHeight = Math.max(1, height * 0.78);
  const minFontSizePx = 1;
  const maxFontSizePx = Math.max(minFontSizePx, maxByHeight);

  if (text.length === 0) {
    return maxFontSizePx;
  }

  const canvas = document.createElement("canvas");
  const context = canvas.getContext("2d");
  if (!context) {
    return maxFontSizePx;
  }

  let low = minFontSizePx;
  let high = maxFontSizePx;
  const weight = bold ? "700" : "400";
  const style = italic ? "italic" : "normal";

  // Binary-search the largest font that fits horizontally in one line.
  for (let i = 0; i < 18; i += 1) {
    const mid = (low + high) / 2;
    context.font = `${style} ${weight} ${mid}px sans-serif`;
    const measuredWidth = context.measureText(text).width;

    if (measuredWidth <= availableWidth) {
      low = mid;
    } else {
      high = mid;
    }
  }

  return Math.max(minFontSizePx, Math.min(low, maxFontSizePx));
}

function ValueDisplayPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [state, setState] = useState<PanelState>(() =>
    merge({}, defaultSettings, context.initialState || {}),
  );
  const [messages, setMessages] = useState<Array<{ message: unknown }>>([]);
  const [panelSize, setPanelSize] = useState({ width: 0, height: 0 });
  const containerRef = useRef<HTMLDivElement | null>(null);

  const { lastPart } = splitTopic(state.data.topic || "") || { lastPart: "" };
  const latestMessage = messages.length > 0 ? messages[messages.length - 1] : undefined;
  const parsedValue = latestMessage ? parseValue(latestMessage.message, lastPart || "") : undefined;
  const transformedValue =
    parsedValue === undefined ? "N/A" : applyFunction(parsedValue, state.numerical.function);
  const displayValue =
    typeof transformedValue === "number"
      ? formatNumberWithApostropheThousands(transformedValue, state.numerical.precision)
      : transformedValue;
  const normalizedUnit = state.display.unit.trim();
  const unitSuffix = normalizedUnit ? ` ${normalizedUnit}` : "";
  const displayText = `${displayValue}${unitSuffix}`;
  const autoFontSizePx = useMemo(
    () =>
      getAutoFontSizePx({
        text: displayText,
        width: panelSize.width,
        height: panelSize.height,
        bold: state.display.bold,
        italic: state.display.italic,
      }),
    [displayText, panelSize.width, panelSize.height, state.display.bold, state.display.italic],
  );
  const fontSize = state.display.fontSize === "auto" ? `${autoFontSizePx}px` : state.display.fontSize;
  const manualFontColor = transformedValue === "N/A" ? "#303030" : state.display.fontColor;
  const useThresholdColors =
    state.background?.useThresholdColors ?? state.display.useThresholdBackground ?? false;
  const autoTextColorOnThresholds = state.background?.autoTextColorOnThresholds ?? true;
  const lowerThresholdValue = state.background?.lowerThreshold ?? state.display.lowerThreshold ?? 0;
  const upperThresholdValue = state.background?.upperThreshold ?? state.display.upperThreshold ?? 100;
  const belowColor = state.background?.belowColor ?? state.display.belowThresholdColor ?? "#2e7d32";
  const betweenColor =
    state.background?.betweenColor ?? state.display.betweenThresholdColor ?? "#f9a825";
  const aboveColor = state.background?.aboveColor ?? state.display.aboveThresholdColor ?? "#c62828";
  const normalBackground = state.background?.normalColor ?? "#121212";
  const naBackground = state.background?.naColor ?? state.background?.fallbackColor ?? "#121212";

  let backgroundColor = transformedValue === "N/A" ? naBackground : normalBackground;

  if (useThresholdColors && typeof transformedValue === "number") {
    const lowerThreshold = Math.min(lowerThresholdValue, upperThresholdValue);
    const upperThreshold = Math.max(lowerThresholdValue, upperThresholdValue);

    if (transformedValue < lowerThreshold) {
      backgroundColor = belowColor;
    } else if (transformedValue > upperThreshold) {
      backgroundColor = aboveColor;
    } else {
      backgroundColor = betweenColor;
    }
  }

  const fontColor =
    useThresholdColors && typeof transformedValue === "number" && autoTextColorOnThresholds
      ? getReadableTextColor(backgroundColor)
      : manualFontColor;

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setMessages((previousMessages) => {
        const frameMessages = [...(renderState.currentFrame || [])] as Array<{ message: unknown }>;
        return frameMessages.length > 0 ? frameMessages : previousMessages;
      });
      done();
    };

    context.watch("currentFrame");
  }, [context]);

  useEffect(() => {
    const { firstPart } = splitTopic(state.data.topic || "") || { firstPart: "" };
    context.unsubscribeAll();
    setMessages([]);

    if (firstPart) {
      context.subscribe([{ topic: firstPart }]);
    }

    return () => {
      context.unsubscribeAll();
    };
  }, [context, state.data.topic]);

  useEffect(() => {
    context.saveState(state);
  }, [context, state]);

  useEffect(() => {
    updateSettingsEditor(context, state, (action: SettingsTreeAction) => {
      if (action.action !== "update") {
        return;
      }

      const { path, value } = action.payload;
      setState((previousState) => {
        const nextState = merge({}, previousState);
        set(nextState, path, value);
        return nextState;
      });
    });
  }, [context, state]);

  useEffect(() => {
    if (!containerRef.current) {
      return undefined;
    }

    const resizeObserver = new ResizeObserver((entries) => {
      for (const entry of entries) {
        setPanelSize({
          width: entry.contentRect.width,
          height: entry.contentRect.height,
        });
      }
    });

    resizeObserver.observe(containerRef.current);
    return () => resizeObserver.disconnect();
  }, []);

  return (
    <div
      ref={containerRef}
      className="value-display"
      style={{
        display: "flex",
        width: "100%",
        height: "100%",
        maxWidth: "none",
        maxHeight: "none",
        padding: "0 0.5rem",
        margin: 0,
        justifyContent: state.display.align,
        alignItems: "center",
        fontSize,
        fontWeight: state.display.bold ? "bold" : "normal",
        fontStyle: state.display.italic ? "italic" : "normal",
        color: fontColor,
        backgroundColor,
        overflow: "hidden",
      }}
    >
      <span style={{ whiteSpace: "nowrap", maxWidth: "100%" }}>{displayText}</span>
    </div>
  );
}

export function initValueDisplayPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<ValueDisplayPanel context={context} />);

  return () => {
    root.unmount();
  };
}