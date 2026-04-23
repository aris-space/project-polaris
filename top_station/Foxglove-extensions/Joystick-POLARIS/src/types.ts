import { Time } from "@foxglove/rostime";

type Header = {
  stamp: Time;
  frame_id: string;
};

// sensor_msgs/Joy message definition
// http://docs.ros.org/en/api/sensor_msgs/html/msg/Joy.html
export type Joy = {
  header: Header;
  axes: number[];
  buttons: number[];
};

export interface ButtonConfig {
  type: "button";
  text: string;
  x: number;
  y: number;
  rot: number;
  gamepadApi: number;
  joyButton: number;
  transform?: string;
  joyValue?: number;
}

export interface BarConfig {
  type: "bar";
  text: string;
  x: number;
  y: number;
  rot: number;
  gamepadApi: number;
  joyAxis: number;
  transform?: string;
}

export interface StickConfig {
  type: "stick";
  text?: string;
  x: number;
  y: number;
  gamepadApiX: number;
  gamepadApiY: number;
  gamepadApiButton: number;
  joyAxisX: number;
  joyAxisY: number;
  joyButton: number;
}

export interface DPadConfig {
  type: "button";
  text: string;
  x: number;
  y: number;
  rot: number;
  gamepadApi: number;
  joyAxis: number;
  joyValue: number;
  transform: "dpad";
}

export type DisplayMapping = (ButtonConfig | BarConfig | StickConfig | DPadConfig)[];
