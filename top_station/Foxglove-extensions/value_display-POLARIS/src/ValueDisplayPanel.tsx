import { PanelExtensionContext, SettingsTreeAction } from "@foxglove/extension";
import { merge, set } from "lodash";
import { ReactElement, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";
import { applyFunction, parseValue, splitTopic } from "./common/index.js";
import { defaultSettings, updateSettingsEditor } from "./settings.js";

type PanelState = typeof defaultSettings;

function formatNumberWithApostrophes(value: number, precision: number): string {
  const fixedValue = value.toFixed(Math.max(0, precision));
  const [integerPart = "", fractionalPart] = fixedValue.split(".");
  const formattedIntegerPart = integerPart.replace(/\B(?=(\d{3})+(?!\d))/g, "'");

  return fractionalPart != undefined ? `${formattedIntegerPart}.${fractionalPart}` : formattedIntegerPart;
}

function ValueDisplayPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [state, setState] = useState<PanelState>(() =>
    merge({}, defaultSettings, context.initialState || {}),
  );
  const [messages, setMessages] = useState<Array<{ message: unknown }>>([]);
  const [height, setHeight] = useState(0);
  const containerRef = useRef<HTMLDivElement | null>(null);

  const { lastPart } = splitTopic(state.data.topic || "") || { lastPart: "" };
  const parsedValue = messages.length > 0 ? parseValue(messages[0]?.message, lastPart || "") : undefined;
  const transformedValue =
    parsedValue === undefined ? "N/A" : applyFunction(parsedValue, state.numerical.function);
  const displayValue =
    typeof transformedValue === "number"
      ? formatNumberWithApostrophes(transformedValue, state.numerical.precision)
      : transformedValue;
  const fontSize =
    state.display.fontSize === "auto" ? `${(height / 50) * 1.5}rem` : state.display.fontSize;
  const fontColor = transformedValue === "N/A" ? "#303030" : state.display.fontColor;
  const backgroundColor = transformedValue === "N/A" ? "#121212" : state.display.backgroundColor;

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setMessages([...(renderState.currentFrame || [])] as Array<{ message: unknown }>);
      done();
    };

    context.watch("currentFrame");
  }, [context]);

  useEffect(() => {
    const { firstPart } = splitTopic(state.data.topic || "") || { firstPart: "" };
    context.unsubscribeAll();

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
        setHeight(entry.contentRect.height);
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
      }}
    >
      <span>
        {displayValue}
        {state.display.unit ? ` ${state.display.unit}` : ""}
      </span>
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