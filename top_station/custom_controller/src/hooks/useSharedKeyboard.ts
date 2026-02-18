import { useEffect, useState, useCallback, useRef } from "react";

type KbMap = {
  button: number;
  axis: number;
  direction: number;
  value: number;
  toggle: boolean;
  toggled: boolean;
};

type KeyboardStateListener = (state: Map<string, KbMap>) => void;

class SharedKeyboardManager {
  private static instance: SharedKeyboardManager;
  private trackedKeys: Map<string, KbMap> = new Map();
  private listeners: Set<KeyboardStateListener> = new Set();
  private listenerCount = 0;

  private constructor() {}

  static getInstance(): SharedKeyboardManager {
    if (!SharedKeyboardManager.instance) {
      SharedKeyboardManager.instance = new SharedKeyboardManager();
    }
    return SharedKeyboardManager.instance;
  }

  private setupListeners() {
    const handleKeyDown = (event: KeyboardEvent) => {
      const normalizedKey = this.normalizeKey(event);
      if (this.trackedKeys.has(normalizedKey)) {
        const k = this.trackedKeys.get(normalizedKey);
        if (k) {
          if (k.toggle) {
            k.toggled = !k.toggled;
            k.value = k.toggled ? 1 : 0;
          } else {
            k.value = 1;
          }
          this.notifyListeners();
        }
      }
    };

    const handleKeyUp = (event: KeyboardEvent) => {
      const normalizedKey = this.normalizeKey(event);
      if (this.trackedKeys.has(normalizedKey)) {
        const k = this.trackedKeys.get(normalizedKey);
        if (k) {
          if (!k.toggle) {
            k.value = 0;
          }
          this.notifyListeners();
        }
      }
    };

    document.addEventListener("keydown", handleKeyDown);
    document.addEventListener("keyup", handleKeyUp);

    return () => {
      document.removeEventListener("keydown", handleKeyDown);
      document.removeEventListener("keyup", handleKeyUp);
    };
  }

  private normalizeKey(event: KeyboardEvent): string {
    const { code, key } = event;
    if (code.startsWith("Key")) {
      return code.slice(3).toLowerCase();
    }
    if (code.startsWith("Digit")) {
      return code.slice(5);
    }
    if (code === "Space") {
      return "Space";
    }
    return key;
  }

  private notifyListeners() {
    this.listeners.forEach((listener) => {
      listener(new Map(this.trackedKeys));
    });
  }

  setKeyMap(keyMap: Map<string, KbMap>) {
    this.trackedKeys = new Map(keyMap);
    this.notifyListeners();
  }

  subscribe(listener: KeyboardStateListener) {
    this.listeners.add(listener);
    this.listenerCount++;

    if (this.listenerCount === 1) {
      this.setupListeners();
    }

    return () => {
      this.listeners.delete(listener);
      this.listenerCount--;
    };
  }
}

export function useSharedKeyboard(initialKeyMap: Map<string, KbMap>) {
  const manager = SharedKeyboardManager.getInstance();
  const [trackedKeys, setTrackedKeys] =
    useState<Map<string, KbMap>>(initialKeyMap);
  const unsubscribeRef = useRef<(() => void) | null>(null);

  useEffect(() => {
    manager.setKeyMap(initialKeyMap);
  }, [initialKeyMap, manager]);

  useEffect(() => {
    unsubscribeRef.current = manager.subscribe(setTrackedKeys);
    return () => {
      unsubscribeRef.current?.();
    };
  }, [manager]);

  return trackedKeys;
}
