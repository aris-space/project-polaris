import { PanelExtensionContext } from "@foxglove/extension";
import { RefObject, useEffect, useLayoutEffect, useRef, useState } from "react";

import { parseMessagePath, getValueAtPath } from "./utils";

export function useInstrumentPanel(
  context: PanelExtensionContext,
  messagePaths: string[],
): {
  getValue: (messagePath: string) => number | undefined;
  containerRef: RefObject<HTMLDivElement>;
  size: string;
} {
  const [latestMessages, setLatestMessages] = useState<Map<string, unknown>>(new Map());
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const containerRef = useRef<HTMLDivElement>(null);
  const [size, setSize] = useState("200px");

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const frame = renderState.currentFrame;
      if (frame && frame.length > 0) {
        setLatestMessages((prev) => {
          const next = new Map(prev);
          for (const event of frame) {
            next.set(event.topic, event.message);
          }
          return next;
        });
      }
    };
    context.watch("currentFrame");
  }, [context]);

  const pathsKey = messagePaths.join(",");
  useEffect(() => {
    const seen = new Set<string>();
    const topics: string[] = [];
    for (const mp of messagePaths) {
      const parsed = parseMessagePath(mp);
      if (parsed && !seen.has(parsed.topic)) {
        seen.add(parsed.topic);
        topics.push(parsed.topic);
      }
    }
    context.subscribe(topics.map((topic) => ({ topic })));
    setLatestMessages(new Map());
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [context, pathsKey]);

  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;
    const observer = new ResizeObserver((entries) => {
      const entry = entries[0];
      if (entry) {
        const { width, height } = entry.contentRect;
        setSize(`${Math.floor(Math.min(width, height)) - 4}px`);
      }
    });
    observer.observe(el);
    return () => { observer.disconnect(); };
  }, []);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return {
    getValue: (messagePath: string) => {
      const parsed = parseMessagePath(messagePath);
      if (!parsed) return undefined;
      const msg = latestMessages.get(parsed.topic);
      return getValueAtPath(msg, parsed.fieldPath);
    },
    containerRef,
    size,
  };
}
