import { DisplayMapping, ButtonConfig, BarConfig, StickConfig } from "../types";
import ps4Mapping from "../components/display-mappings/ps4.json";

/**
 * Converts Gamepad API input to ROS /joy topic format.
 * Uses the PS4 mapping configuration to properly convert
 * between Gamepad API indices and ROS Joy indices.
 */

// Transform functions for axis values
const AXIS_TRANSFORMS: Record<string, (value: number) => number> = {
  identity: (v: number) => v,
  l2_normalize: (v: number) => v * 2 - 1, // [0, 1] → [-1, 1]
  r2_normalize: (v: number) => v * 2 - 1, // [0, 1] → [-1, 1]
};

/**
 * Convert Gamepad API gamepad to ROS Joy message format
 * @param gamepad - The Gamepad API gamepad object
 * @returns Object with properly formatted buttons and axes arrays for ROS Joy
 */
export function gamepadToRosJoy(gamepad: Gamepad): {
  buttons: number[];
  axes: number[];
} {
    // Validate gamepad input
    if (!gamepad) {
      console.error("[POLARIS Joystick] Invalid gamepad object");
      return { buttons: new Array(18).fill(0), axes: new Array(6).fill(0) };
    }

  // Initialize arrays with proper size for ROS Joy
  const buttons: number[] = new Array(18).fill(0);
  const axes: number[] = new Array(6).fill(0);

  const config = (ps4Mapping as unknown as DisplayMapping) || [];

  // Process each mapping element from PS4 configuration
  for (const element of config) {
    if (element.type === "button") {
      const btn = element as any;
      const gamepadApi = btn.gamepadApi ?? -1;
      
      if (btn.transform === "dpad") {
        // D-pad button mapped to axis
        const joyAxis = btn.joyAxis ?? -1;
        const joyValue = btn.joyValue ?? 0;
        
        if (gamepadApi >= 0 && joyAxis >= 0) {
          const btn_obj = gamepad.buttons[gamepadApi];
          if (btn_obj && btn_obj.pressed) {
            axes[joyAxis] = joyValue;
          }
        }
      } else {
        // Regular button
        const joyButton = btn.joyButton ?? -1;
        
        if (gamepadApi >= 0 && joyButton >= 0) {
          const btn_obj = gamepad.buttons[gamepadApi];
          buttons[joyButton] = (btn_obj && btn_obj.pressed) ? 1 : 0;
        }
      }
    } else if (element.type === "bar") {
      const bar = element as BarConfig;
      const gamepadApi = bar.gamepadApi ?? -1;
      const joyAxis = bar.joyAxis ?? -1;
      const transform = bar.transform ?? "identity";
      const transformFunc = AXIS_TRANSFORMS[transform];

      if (gamepadApi >= 0 && joyAxis >= 0 && transformFunc) {
                // Bounds checking
                if (joyAxis >= axes.length) {
                  console.warn(`[POLARIS Joystick] Joy axis index ${joyAxis} out of bounds`);
                  continue;
                }
                if (gamepadApi >= gamepad.axes.length && gamepadApi >= gamepad.buttons.length) {
                  console.warn(`[POLARIS Joystick] Gamepad axis/button index ${gamepadApi} out of bounds`);
                  continue;
                }
        
        const value = gamepad.axes[gamepadApi] ?? gamepad.buttons[gamepadApi]?.value ?? 0;
        axes[joyAxis] = transformFunc(value);
        
        // Also set button state for triggers (L2/R2 at indices 6 and 7)
        if ((gamepadApi === 6 || gamepadApi === 7) && gamepadApi < gamepad.buttons.length && gamepadApi < buttons.length) {
          const btn_obj = gamepad.buttons[gamepadApi];
          buttons[gamepadApi] = (btn_obj && btn_obj.pressed) ? 1 : 0;
        }
      }
    } else if (element.type === "stick") {
      const stick = element as StickConfig;
      const gamepadApiX = stick.gamepadApiX ?? -1;
      const gamepadApiY = stick.gamepadApiY ?? -1;
      const gamepadApiButton = stick.gamepadApiButton ?? -1;
      const joyAxisX = stick.joyAxisX ?? -1;
      const joyAxisY = stick.joyAxisY ?? -1;
      const joyButton = stick.joyButton ?? -1;

      if (gamepadApiX >= 0 && joyAxisX >= 0) {
        if (gamepadApiX < gamepad.axes.length && joyAxisX < axes.length) {
          axes[joyAxisX] = gamepad.axes[gamepadApiX] ?? 0;
        }
      }

      if (gamepadApiY >= 0 && joyAxisY >= 0) {
        if (gamepadApiY < gamepad.axes.length && joyAxisY < axes.length) {
          axes[joyAxisY] = gamepad.axes[gamepadApiY] ?? 0;
        }
      }

      if (gamepadApiButton >= 0 && joyButton >= 0) {
        if (gamepadApiButton < gamepad.buttons.length && joyButton < buttons.length) {
          const btn_obj = gamepad.buttons[gamepadApiButton];
          buttons[joyButton] = (btn_obj && btn_obj.pressed) ? 1 : 0;
        }
      }
    }
  }

  return { buttons, axes };
}

