export type SharedRecorderSettings = {
  recordMode?: "all" | "selection" | "exclude_selection";
  includeTopics?: string[];
  excludeTopics?: string[];
  touchscreenMode?: boolean;
};

const STORAGE_KEY = "record-polaris.shared-settings.v1";
const EVENT_NAME = "record-polaris:shared-settings-updated";
const BROADCAST_CHANNEL_NAME = "record-polaris-shared-settings";

let broadcastChannel: BroadcastChannel | undefined;

function getBroadcastChannel(): BroadcastChannel | undefined {
  if (typeof window === "undefined" || typeof BroadcastChannel === "undefined") {
    return undefined;
  }

  if (broadcastChannel == undefined) {
    broadcastChannel = new BroadcastChannel(BROADCAST_CHANNEL_NAME);
  }

  return broadcastChannel;
}

function cloneList(values: unknown): string[] | undefined {
  if (!Array.isArray(values)) {
    return undefined;
  }
  return values.filter((item): item is string => typeof item === "string");
}

function normalizeSharedSettings(raw: unknown): SharedRecorderSettings {
  if (typeof raw !== "object" || raw == undefined) {
    return {};
  }

  const record = raw as Record<string, unknown>;
  return {
    recordMode:
      record.recordMode === "selection" || record.recordMode === "exclude_selection" || record.recordMode === "all"
        ? record.recordMode
        : undefined,
    includeTopics: cloneList(record.includeTopics),
    excludeTopics: cloneList(record.excludeTopics),
    touchscreenMode: typeof record.touchscreenMode === "boolean" ? record.touchscreenMode : undefined,
  };
}

export function readSharedRecorderSettings(): SharedRecorderSettings {
  if (typeof window === "undefined") {
    return {};
  }

  try {
    const raw = window.localStorage.getItem(STORAGE_KEY);
    if (!raw) {
      return {};
    }
    return normalizeSharedSettings(JSON.parse(raw));
  } catch {
    return {};
  }
}

export function updateSharedRecorderSettings(patch: SharedRecorderSettings): SharedRecorderSettings {
  if (typeof window === "undefined") {
    return patch;
  }

  const next = {
    ...readSharedRecorderSettings(),
    ...patch,
  };

  try {
    window.localStorage.setItem(STORAGE_KEY, JSON.stringify(next));
    window.dispatchEvent(new CustomEvent(EVENT_NAME, { detail: next }));
    getBroadcastChannel()?.postMessage(next);
  } catch {
    // Best effort sync: panel should continue working even if local storage is unavailable.
  }

  return next;
}

export function subscribeSharedRecorderSettings(
  onUpdate: (settings: SharedRecorderSettings) => void,
): () => void {
  if (typeof window === "undefined") {
    return () => undefined;
  }

  const onCustomEvent = (event: Event) => {
    const detail = (event as CustomEvent<SharedRecorderSettings>).detail;
    onUpdate(normalizeSharedSettings(detail));
  };

  const onStorage = (event: StorageEvent) => {
    if (event.key !== STORAGE_KEY || !event.newValue) {
      return;
    }

    try {
      onUpdate(normalizeSharedSettings(JSON.parse(event.newValue)));
    } catch {
      // Ignore malformed values.
    }
  };

  const onBroadcast = (event: MessageEvent<SharedRecorderSettings>) => {
    onUpdate(normalizeSharedSettings(event.data));
  };

  const channel = getBroadcastChannel();

  window.addEventListener(EVENT_NAME, onCustomEvent as EventListener);
  window.addEventListener("storage", onStorage);
  channel?.addEventListener("message", onBroadcast as EventListener);

  return () => {
    window.removeEventListener(EVENT_NAME, onCustomEvent as EventListener);
    window.removeEventListener("storage", onStorage);
    channel?.removeEventListener("message", onBroadcast as EventListener);
  };
}