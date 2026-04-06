import { get_cmap } from './js-colormaps.js';

const fontSizes = [
  'auto',
  '8px',
  '9px',
  '10px',
  '11px',
  '12px',
  '14px',
  '16px',
  '18px',
  '24px',
  '30px',
  '36px',
  '48px',
  '60px',
  '72px',
].map((key) => ({ value: key, label: key }));

const functions = [
  'none',
  'abs',
  'ceil',
  'floor',
  'round',
  'sqrt',
  'pow2',
  'exp',
  'log',
  'sin',
  'cos',
  'tan',
  '1/x',
].map((key) => ({ value: key, label: key }));

const splitTopic = (topic) => {
  const parts = topic.split('.');
  if (parts.length === 0) {
    return { firstPart: '', lastPart: '' };
  } else if (parts.length < 2 || parts[1] === '') {
    return { firstPart: parts[0], lastPart: '' };
  } else {
    return {
      firstPart: parts[0],
      lastPart: topic.substring(parts[0].length + 1),
    };
  }
};

const getTopicFromMessagePath = (messagePath) => {
  if (typeof messagePath !== 'string' || messagePath.length === 0 || !messagePath.startsWith('/')) {
    return '';
  }

  const dotIndex = messagePath.indexOf('.');
  const bracketIndex = messagePath.indexOf('[');
  const filterIndex = messagePath.indexOf('{');

  let firstSeparator = -1;
  for (const index of [dotIndex, bracketIndex, filterIndex]) {
    if (index !== -1 && (firstSeparator === -1 || index < firstSeparator)) {
      firstSeparator = index;
    }
  }

  return firstSeparator === -1 ? messagePath : messagePath.slice(0, firstSeparator);
};

const normalizeRpy = (value) => {
  if (!value || typeof value !== 'object') {
    return undefined;
  }

  const roll = value.roll;
  const pitch = value.pitch;
  const yaw = value.yaw;

  if (
    typeof roll !== 'number' ||
    typeof pitch !== 'number' ||
    typeof yaw !== 'number' ||
    !isFinite(roll) ||
    !isFinite(pitch) ||
    !isFinite(yaw)
  ) {
    return undefined;
  }

  return { roll, pitch, yaw };
};

const normalizeQuaternion = (value) => {
  if (!value || typeof value !== 'object') {
    return undefined;
  }

  const x = value.x;
  const y = value.y;
  const z = value.z;
  const w = value.w;

  if (
    typeof x !== 'number' ||
    typeof y !== 'number' ||
    typeof z !== 'number' ||
    typeof w !== 'number' ||
    !isFinite(x) ||
    !isFinite(y) ||
    !isFinite(z) ||
    !isFinite(w)
  ) {
    return undefined;
  }

  return { x, y, z, w };
};

const quaternionToRpy = (quat) => {
  const { x, y, z, w } = quat;

  const sinrCosp = 2 * (w * x + y * z);
  const cosrCosp = 1 - 2 * (x * x + y * y);
  const roll = Math.atan2(sinrCosp, cosrCosp);

  const sinp = 2 * (w * y - z * x);
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * Math.PI / 2 : Math.asin(sinp);

  const sinyCosp = 2 * (w * z + x * y);
  const cosyCosp = 1 - 2 * (y * y + z * z);
  const yaw = Math.atan2(sinyCosp, cosyCosp);

  return { roll, pitch, yaw };
};

const rpyToQuaternion = (rpy) => {
  const { roll, pitch, yaw } = rpy;

  const cy = Math.cos(yaw * 0.5);
  const sy = Math.sin(yaw * 0.5);
  const cp = Math.cos(pitch * 0.5);
  const sp = Math.sin(pitch * 0.5);
  const cr = Math.cos(roll * 0.5);
  const sr = Math.sin(roll * 0.5);

  return {
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy,
    w: cr * cp * cy + sr * sp * sy,
  };
};

const parseNumericOperand = (operandText) => {
  if (operandText === undefined) {
    return undefined;
  }

  const trimmed = operandText.trim();
  if (trimmed.length === 0) {
    return undefined;
  }

  const value = Number(trimmed);
  return Number.isFinite(value) ? value : undefined;
};

const applyMessagePathFunction = (value, functionName, operandText) => {
  const name = functionName.toLowerCase();
  const operand = parseNumericOperand(operandText);

  if (name === 'norm') {
    if (!value || typeof value !== 'object') {
      return undefined;
    }

    const x = value.x;
    const y = value.y;
    const z = value.z;
    if (typeof x !== 'number' || typeof y !== 'number') {
      return undefined;
    }
    if (z === undefined) {
      return Math.hypot(x, y);
    }
    if (typeof z !== 'number') {
      return undefined;
    }
    return Math.hypot(x, y, z);
  }

  if (name === 'rpy') {
    const quat = normalizeQuaternion(value);
    return quat ? quaternionToRpy(quat) : undefined;
  }

  if (name === 'quat') {
    const rpy = normalizeRpy(value);
    return rpy ? rpyToQuaternion(rpy) : undefined;
  }

  if (typeof value !== 'number' || !isFinite(value)) {
    return undefined;
  }

  switch (name) {
    case 'abs':
      return Math.abs(value);
    case 'degrees':
    case 'deg':
      return (value * 180) / Math.PI;
    case 'radians':
    case 'rad':
      return (value * Math.PI) / 180;
    case 'ceil':
      return Math.ceil(value);
    case 'floor':
      return Math.floor(value);
    case 'round':
      return Math.round(value);
    case 'sqrt':
      return Math.sqrt(value);
    case 'exp':
      return Math.exp(value);
    case 'log':
      return Math.log(value);
    case 'sin':
      return Math.sin(value);
    case 'cos':
      return Math.cos(value);
    case 'tan':
      return Math.tan(value);
    case 'neg':
      return -value;
    case 'add':
      return operand === undefined ? undefined : value + operand;
    case 'sub':
      return operand === undefined ? undefined : value - operand;
    case 'mul':
      return operand === undefined ? undefined : value * operand;
    case 'div':
      return operand === undefined ? undefined : value / operand;
    case 'mod':
      return operand === undefined ? undefined : value % operand;
    case 'pow':
      return operand === undefined ? undefined : Math.pow(value, operand);
    default:
      return undefined;
  }
};

const evaluateMessagePath = (message, messagePath) => {
  if (!messagePath || typeof messagePath !== 'string') {
    return undefined;
  }

  const topicName = getTopicFromMessagePath(messagePath);
  if (!topicName) {
    return undefined;
  }

  let remaining = messagePath.slice(topicName.length);
  if (remaining.startsWith('.')) {
    remaining = remaining.slice(1);
  }

  const functionStart = remaining.indexOf('.@');
  const functionHeadStart = remaining.startsWith('@') ? 0 : functionStart;
  const valuePath = functionHeadStart === -1 ? remaining : remaining.slice(0, functionHeadStart);
  const functionTail = functionHeadStart === -1 ? '' : remaining.slice(functionHeadStart);

  let currentValue = parseValue(message, valuePath);
  if (!functionTail) {
    return currentValue;
  }

  let cursor = functionTail;
  while (cursor.length > 0) {
    if (cursor.startsWith('.@')) {
      cursor = cursor.slice(1);
    }

    if (cursor.startsWith('@')) {
      const nameMatch = /^@([A-Za-z_][A-Za-z0-9_]*)/.exec(cursor);
      if (!nameMatch) {
        return undefined;
      }

      const fnName = nameMatch[1];
      cursor = cursor.slice(nameMatch[0].length);

      let operandText;
      if (cursor.startsWith('(')) {
        const closeParen = cursor.indexOf(')');
        if (closeParen === -1) {
          return undefined;
        }
        operandText = cursor.slice(1, closeParen);
        cursor = cursor.slice(closeParen + 1);
      }

      currentValue = applyMessagePathFunction(currentValue, fnName, operandText);
      continue;
    }

    if (cursor.startsWith('.')) {
      cursor = cursor.slice(1);
    }

    const nextFunction = cursor.indexOf('.@');
    const accessor = nextFunction === -1 ? cursor : cursor.slice(0, nextFunction);
    cursor = nextFunction === -1 ? '' : cursor.slice(nextFunction);

    if (accessor.length > 0) {
      currentValue = parseValue(currentValue, accessor);
    }
  }

  return currentValue;
};

const parseValue = (message, currentField) => {
  let value = message;
  if (currentField === '') {
    value = message;
  } else {
    for (const field of currentField.split('.')) {
      if (value && field && field !== '') {
        const index = field.match(/\[(\d+)\]/);
        if (index) {
          let temp = value[field.replace(index[0], '')];
          if (temp && index[1] !== undefined && index[1] != '') {
            value = temp[index[1]];
          } else {
            value = undefined;
          }
        } else {
          value = value[field];
        }
      } else {
        value = undefined;
      }
    }
  }
  return value;
};

const clamp = (value, min, max) => {
  if (
    isNaN(value) ||
    value === undefined ||
    value === null ||
    isNaN(min) ||
    isNaN(max)
  ) {
    return 0;
  }
  return Math.min(Math.max(value, min), max);
};

const applyFunction = (value, function_text) => {
  if (typeof value === 'number') {
    var fnc = (x) => x;
    switch (function_text) {
      case 'none':
        break;
      case 'abs':
        fnc = Math.abs;
        break;
      case 'ceil':
        fnc = Math.ceil;
        break;
      case 'floor':
        fnc = Math.floor;
        break;
      case 'round':
        fnc = Math.round;
        break;
      case 'sqrt':
        fnc = Math.sqrt;
        break;
      case 'pow2':
        fnc = (x) => Math.pow(x, 2);
        break;
      case 'exp':
        fnc = Math.exp;
        break;
      case 'log':
        fnc = Math.log;
        break;
      case 'sin':
        fnc = Math.sin;
        break;
      case 'cos':
        fnc = Math.cos;
        break;
      case 'tan':
        fnc = Math.tan;
        break;
      case '1/x':
        fnc = (x) => 1 / x;
        break;
    }
    return fnc(value);
  } else {
    return value;
  }
};

const getColorFromProgress = (x, min, max, colormap, reversed) => {
  if (isNaN(x) || x === undefined || x === null || isNaN(min) || isNaN(max)) {
    return '#303030';
  } else {
    const norm = clamp((x - min) / (max - min), 0, 1);
    if (norm === undefined) {
      return '#303030';
    }
    const cmap_name = reversed ? `${colormap}_r` : colormap;
    const cmap = get_cmap(cmap_name);
    const [r, g, b] = cmap(norm);
    return `rgb(${r}, ${g}, ${b})`;
  }
};

const subscribeToTopic = (context, value) => {
  context.unsubscribeAll();
  const { firstPart, lastPart } = splitTopic(value);
  if (firstPart) {
    context.subscribe([{ topic: firstPart }]);
    return { firstPart: firstPart, lastPart: lastPart };
  }
  return { firstPart: undefined, lastPart: undefined };
};

const generateRandomDivClass = (divType) => {
  return (
    Math.random().toString(36).substring(2, 15) +
    Math.random().toString(36).substring(2, 15) +
    document.getElementsByClassName(divType).length
  );
};

const getWidthHeight = (width, height, elementName) => {
  const elements = document.getElementsByClassName(elementName);
  if (elements.length > 0) {
    const element = elements[0]; // Get the first element with the class name
    const resizeObserver = new ResizeObserver((entries) => {
      for (let entry of entries) {
        width.value = entry.contentRect.width;
        height.value = entry.contentRect.height;
      }
    });
    // Start observing the element for size changes
    resizeObserver.observe(element);
  }
};

export {
  fontSizes,
  functions,
  splitTopic,
  getTopicFromMessagePath,
  clamp,
  getColorFromProgress,
  parseValue,
  evaluateMessagePath,
  applyFunction,
  subscribeToTopic,
  generateRandomDivClass,
  getWidthHeight,
};
