import { ReactElement, useEffect, useRef } from "react";

import { ensureCss } from "../BundledFlightIndicators";
import dualGaugeMechanicsRaw from "../assets/instruments/dual_gauge_mechanics.svg?raw";
import dualGaugeNeedleRaw from "../assets/instruments/dual_gauge_needle.svg?raw";
import fiCircleRaw from "../assets/instruments/fi_circle.svg?raw";

function toDataUrl(svgRaw: string): string {
  return `data:image/svg+xml;charset=utf-8,${encodeURIComponent(svgRaw)}`;
}

function mirrorSvgX(svgRaw: string): string {
  return svgRaw.replace(
    /(<svg[^>]*>)([\s\S]*)(<\/svg>\s*$)/,
    (_, open, body, close) =>
      `${open}<g transform="scale(-1,1) translate(-400,0)">${body}</g>${close}`,
  );
}

const FACE_URL     = toDataUrl(dualGaugeMechanicsRaw);
const NEEDLE_L_URL = toDataUrl(dualGaugeNeedleRaw);
const NEEDLE_R_URL = toDataUrl(mirrorSvgX(dualGaugeNeedleRaw));
const CIRCLE_URL   = toDataUrl(fiCircleRaw);

// Dual gauge: configurable display range, tick count, and color zones. 90° sweep per side.
//
// Needle SVGs: 400×400 canvas.
//   Left  needle: arm points RIGHT from x=50 (pivot at 12.5%).
//   Right needle: arm points LEFT  from x=350 (pivot at 87.5%), geometrically mirrored.
//
// Rotation formula (general for any [displayMin, displayMax]):
//   center    = (displayMin + displayMax) / 2
//   halfRange = (displayMax - displayMin) / 2
//   Left:  rot = (center - clamped) × (45 / halfRange)
//   Right: rot = (clamped - center) × (45 / halfRange)
//
// Tick / arc geometry (400×400 SVG canvas):
//   Left  gauge pivot: (30,  200)  Right gauge pivot: (370, 200)
//   Left  angle(v) = 45 − (v−min)/(max−min) × 90  degrees (clockwise from +x, screen Y-down)
//   Right angle(v) = 135 + (v−min)/(max−min) × 90 degrees
//   Major tick: r = 129–150 px, stroke-width 4
//   Minor tick: r = 135–150 px, stroke-width 2  (midpoint between adjacent major ticks)
//   Color arc:  r = 145 px, stroke-width 10
//   Label:      r = 107 px (inside the tick arc)
//
//   SVG arc sweep: left gauge sweep=0 (CCW in SVG / theta decreasing)
//                  right gauge sweep=1 (CW in SVG / theta increasing)

const MAX_TICKS = 20;
const ARC_R     = 145;

// Returns the scale factor (out of 400) for a side label based on its character count.
// 1 char → 22, 2–3 chars → 18, 4–7 chars → 14.
function sideLabelScale(text: string): number {
  const len = text.length;
  if (len <= 2) return 22;
  if (len <= 3) return 20;
  if (len <= 5) return 18;
  if (len <= 7) return 16;
  return 14;
}

// ── Geometry helpers ──────────────────────────────────────────────────────────

function leftAngleDeg(v: number, min: number, max: number): number {
  return 45 - ((v - min) / (max - min)) * 90;
}
function rightAngleDeg(v: number, min: number, max: number): number {
  return 135 + ((v - min) / (max - min)) * 90;
}

// Draw major + minor tick lines into a <g> element.
// majorAngles: one angle (degrees) per major tick; minor ticks at midpoint between adjacent majors.
function drawTickLines(
  g: SVGGElement,
  majorAngles: number[],
  pivotX: number,
  pivotY: number,
  showSubTicks: boolean,
): void {
  while (g.firstChild) g.removeChild(g.firstChild);
  if (majorAngles.length < 2) return;

  function addLine(x1: number, y1: number, x2: number, y2: number, sw: number): void {
    const ln = document.createElementNS("http://www.w3.org/2000/svg", "line");
    ln.setAttribute("x1", x1.toFixed(2)); ln.setAttribute("y1", y1.toFixed(2));
    ln.setAttribute("x2", x2.toFixed(2)); ln.setAttribute("y2", y2.toFixed(2));
    ln.setAttribute("stroke", "white");
    ln.setAttribute("stroke-width", String(sw));
    ln.setAttribute("stroke-linecap", "butt");
    g.appendChild(ln);
  }

  for (const deg of majorAngles) {
    const rad = (deg * Math.PI) / 180;
    addLine(pivotX + 129 * Math.cos(rad), pivotY + 129 * Math.sin(rad),
            pivotX + 150 * Math.cos(rad), pivotY + 150 * Math.sin(rad), 4);
  }
  if (showSubTicks) {
    for (let i = 0; i < majorAngles.length - 1; i++) {
      const mid = (majorAngles[i]! + majorAngles[i + 1]!) / 2;
      const rad = (mid * Math.PI) / 180;
      addLine(pivotX + 135 * Math.cos(rad), pivotY + 135 * Math.sin(rad),
              pivotX + 150 * Math.cos(rad), pivotY + 150 * Math.sin(rad), 2);
    }
  }
}

type ColorZone = { color: string; start: number; end: number };

// Draw color arc zones into a <g> element. Zones rendered in order (last = top).
// Arcs are clamped to [displayMin, displayMax]; zero-length zones are skipped.
// Left gauge: sweep=0 (SVG CCW), Right gauge: sweep=1 (SVG CW).
function drawColorArcs(
  g: SVGGElement,
  displayMin: number,
  displayMax: number,
  pivotX: number,
  pivotY: number,
  isLeft: boolean,
  zones: ColorZone[],
): void {
  while (g.firstChild) g.removeChild(g.firstChild);
  const range = displayMax - displayMin;
  if (range <= 0) return;

  const sweep     = isLeft ? 0 : 1;
  const angleFn   = isLeft ? leftAngleDeg : rightAngleDeg;

  for (const { color, start, end } of zones) {
    const cStart = Math.max(displayMin, Math.min(displayMax, start));
    const cEnd   = Math.max(displayMin, Math.min(displayMax, end));
    if (cEnd <= cStart) continue;

    const a1 = (angleFn(cStart, displayMin, displayMax) * Math.PI) / 180;
    const a2 = (angleFn(cEnd,   displayMin, displayMax) * Math.PI) / 180;
    const x1 = pivotX + ARC_R * Math.cos(a1);
    const y1 = pivotY + ARC_R * Math.sin(a1);
    const x2 = pivotX + ARC_R * Math.cos(a2);
    const y2 = pivotY + ARC_R * Math.sin(a2);

    const path = document.createElementNS("http://www.w3.org/2000/svg", "path");
    path.setAttribute("d", `M ${x1.toFixed(2)} ${y1.toFixed(2)} A ${ARC_R} ${ARC_R} 0 0 ${sweep} ${x2.toFixed(2)} ${y2.toFixed(2)}`);
    path.setAttribute("fill", "none");
    path.setAttribute("stroke", color);
    path.setAttribute("stroke-width", "10");
    g.appendChild(path);
  }
}

type Props = {
  leftPct: number | undefined;
  rightPct: number | undefined;
  size: string;
  topLabel: string;
  bottomLabel: string;
  leftLabel: string;
  leftLabelVertical: boolean;
  rightLabel: string;
  rightLabelVertical: boolean;
  leftDisplayMin: number;
  leftDisplayMax: number;
  leftClamp: boolean;
  leftClampMin: number;
  leftClampMax: number;
  leftTickPrecision: number;
  leftTickCount: number;
  leftSubTicks: boolean;
  leftTickLabels?: string[];
  leftTickPositions?: number[];
  leftZones: ColorZone[];
  rightDisplayMin: number;
  rightDisplayMax: number;
  rightClamp: boolean;
  rightClampMin: number;
  rightClampMax: number;
  rightTickPrecision: number;
  rightTickCount: number;
  rightSubTicks: boolean;
  rightTickLabels?: string[];
  rightTickPositions?: number[];
  rightZones: ColorZone[];
};

export function DualGauge({
  leftPct, rightPct, size,
  topLabel, bottomLabel, leftLabel, leftLabelVertical, rightLabel, rightLabelVertical,
  leftDisplayMin, leftDisplayMax, leftClamp, leftClampMin, leftClampMax, leftTickPrecision, leftTickCount, leftSubTicks, leftTickLabels, leftTickPositions, leftZones,
  rightDisplayMin, rightDisplayMax, rightClamp, rightClampMin, rightClampMax, rightTickPrecision, rightTickCount, rightSubTicks, rightTickLabels, rightTickPositions, rightZones,
}: Props): ReactElement {
  const containerRef   = useRef<HTMLDivElement>(null);
  const leftNeedleRef  = useRef<HTMLDivElement | null>(null);
  const rightNeedleRef = useRef<HTMLDivElement | null>(null);
  const labelTopRef    = useRef<HTMLDivElement | null>(null);
  const labelBottomRef = useRef<HTMLDivElement | null>(null);
  const labelLeftRef   = useRef<HTMLDivElement | null>(null);
  const labelRightRef  = useRef<HTMLDivElement | null>(null);
  const leftTickRefs   = useRef<(HTMLDivElement | null)[]>(Array(MAX_TICKS).fill(null));
  const rightTickRefs  = useRef<(HTMLDivElement | null)[]>(Array(MAX_TICKS).fill(null));
  const leftArcGRef    = useRef<SVGGElement | null>(null);
  const rightArcGRef   = useRef<SVGGElement | null>(null);
  const leftTickGRef   = useRef<SVGGElement | null>(null);
  const rightTickGRef  = useRef<SVGGElement | null>(null);
  const labelBottom2Ref = useRef<HTMLDivElement | null>(null);

  // Build DOM once
  useEffect(() => {
    const el = containerRef.current;
    if (!el) return;

    ensureCss();

    const instrument = document.createElement("div");
    instrument.className = "instrument dual-gauge";

    // ── Face ──────────────────────────────────────────────────────────────
    const face = document.createElement("img");
    face.className = "box";
    face.src = FACE_URL;
    instrument.appendChild(face);

    // ── Dynamic SVG overlay (above face, below needles) ───────────────────
    // Layer order within SVG: color arcs → tick lines (ticks render on top of arcs).
    const tickSvg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
    tickSvg.setAttribute("viewBox", "0 0 400 400");
    tickSvg.style.position      = "absolute";
    tickSvg.style.top           = "0";
    tickSvg.style.left          = "0";
    tickSvg.style.width         = "100%";
    tickSvg.style.height        = "100%";
    tickSvg.style.pointerEvents = "none";
    const leftArcG  = document.createElementNS("http://www.w3.org/2000/svg", "g");
    const rightArcG = document.createElementNS("http://www.w3.org/2000/svg", "g");
    const leftG     = document.createElementNS("http://www.w3.org/2000/svg", "g");
    const rightG    = document.createElementNS("http://www.w3.org/2000/svg", "g");
    tickSvg.appendChild(leftArcG);
    tickSvg.appendChild(rightArcG);
    tickSvg.appendChild(leftG);
    tickSvg.appendChild(rightG);
    instrument.appendChild(tickSvg);

    // ── Left needle ───────────────────────────────────────────────────────
    const leftNeedleDiv = document.createElement("div");
    leftNeedleDiv.className = "box";
    leftNeedleDiv.style.transformOrigin = "7.5% 50%";
    leftNeedleDiv.style.transform       = "rotate(45deg)";
    const leftNeedleImg = document.createElement("img");
    leftNeedleImg.className = "box";
    leftNeedleImg.src = NEEDLE_L_URL;
    leftNeedleDiv.appendChild(leftNeedleImg);
    instrument.appendChild(leftNeedleDiv);

    // ── Right needle ──────────────────────────────────────────────────────
    const rightNeedleDiv = document.createElement("div");
    rightNeedleDiv.className = "box";
    rightNeedleDiv.style.transformOrigin = "92.5% 50%";
    rightNeedleDiv.style.transform       = "rotate(-45deg)";
    const rightNeedleImg = document.createElement("img");
    rightNeedleImg.className = "box";
    rightNeedleImg.src = NEEDLE_R_URL;
    rightNeedleDiv.appendChild(rightNeedleImg);
    instrument.appendChild(rightNeedleDiv);

    // ── Bezel overlay ─────────────────────────────────────────────────────
    const circleDiv = document.createElement("div");
    circleDiv.className = "mechanics box";
    const circleImg = document.createElement("img");
    circleImg.className = "box";
    circleImg.src = CIRCLE_URL;
    circleDiv.appendChild(circleImg);
    instrument.appendChild(circleDiv);

    // ── Text overlay helper ───────────────────────────────────────────────
    function makeTextDiv(): HTMLDivElement {
      const d = document.createElement("div");
      d.style.position      = "absolute";
      d.style.transform     = "translate(-50%, -50%)";
      d.style.color         = "white";
      d.style.fontFamily    = "Noto Sans KR, sans-serif";
      d.style.fontWeight    = "bold";
      d.style.pointerEvents = "none";
      d.style.whiteSpace    = "nowrap";
      return d;
    }

    // ── Corner labels ─────────────────────────────────────────────────────
    const topEl    = makeTextDiv(); topEl.style.left    = "50%"; topEl.style.top    = "20%";
    const bottomEl = makeTextDiv(); bottomEl.style.left = "50%"; bottomEl.style.top = "80%";
    const leftEl   = makeTextDiv(); leftEl.style.left   = "25%"; leftEl.style.top   = "50%";
    const rightEl  = makeTextDiv(); rightEl.style.left  = "75%"; rightEl.style.top  = "50%";
    const bottomEl2 = makeTextDiv(); bottomEl2.style.display = "none";
    instrument.appendChild(topEl);
    instrument.appendChild(bottomEl);
    instrument.appendChild(bottomEl2);
    instrument.appendChild(leftEl);
    instrument.appendChild(rightEl);

    // ── Tick labels (MAX_TICKS per side, hidden initially) ────────────────
    const leftTicks: HTMLDivElement[] = [];
    for (let i = 0; i < MAX_TICKS; i++) {
      const d = makeTextDiv(); d.style.display = "none";
      instrument.appendChild(d); leftTicks.push(d);
    }
    const rightTicks: HTMLDivElement[] = [];
    for (let i = 0; i < MAX_TICKS; i++) {
      const d = makeTextDiv(); d.style.display = "none";
      instrument.appendChild(d); rightTicks.push(d);
    }

    el.appendChild(instrument);
    leftNeedleRef.current  = leftNeedleDiv;
    rightNeedleRef.current = rightNeedleDiv;
    labelTopRef.current    = topEl;
    labelBottomRef.current = bottomEl;
    labelLeftRef.current   = leftEl;
    labelRightRef.current  = rightEl;
    leftTickRefs.current   = leftTicks;
    rightTickRefs.current  = rightTicks;
    leftArcGRef.current    = leftArcG;
    rightArcGRef.current   = rightArcG;
    leftTickGRef.current   = leftG;
    rightTickGRef.current  = rightG;
    labelBottom2Ref.current = bottomEl2;

    return () => {
      el.innerHTML = "";
      leftNeedleRef.current  = null;
      rightNeedleRef.current = null;
      labelTopRef.current    = null;
      labelBottomRef.current = null;
      labelLeftRef.current   = null;
      labelRightRef.current  = null;
      leftTickRefs.current   = Array(MAX_TICKS).fill(null);
      rightTickRefs.current  = Array(MAX_TICKS).fill(null);
      leftArcGRef.current    = null;
      rightArcGRef.current   = null;
      leftTickGRef.current   = null;
      rightTickGRef.current  = null;
      labelBottom2Ref.current = null;
    };
  }, []);

  // ── Resize ────────────────────────────────────────────────────────────────
  useEffect(() => {
    const n = parseInt(size, 10);
    if (!isNaN(n)) {
      const instrument = containerRef.current?.querySelector<HTMLElement>(".instrument");
      if (instrument) {
        instrument.style.width  = `${n}px`;
        instrument.style.height = `${n}px`;
      }
      const cornerFs = `${(n * 22 / 400).toFixed(1)}px`;
      const tickFs   = `${(n * 18 / 400).toFixed(1)}px`;
      if (labelTopRef.current) labelTopRef.current.style.fontSize = cornerFs;
      if (labelLeftRef.current)  labelLeftRef.current.style.fontSize  = `${(n * sideLabelScale(leftLabel)  / 400).toFixed(1)}px`;
      if (labelRightRef.current) labelRightRef.current.style.fontSize = `${(n * sideLabelScale(rightLabel) / 400).toFixed(1)}px`;
      const bottomParts = bottomLabel.split(",").map((s) => s.trim());
      if (bottomParts.length >= 2) {
        if (labelBottomRef.current)  labelBottomRef.current.style.fontSize  = `${(n * sideLabelScale(bottomParts[0] ?? "") / 400).toFixed(1)}px`;
        if (labelBottom2Ref.current) labelBottom2Ref.current.style.fontSize = `${(n * sideLabelScale(bottomParts[1] ?? "") / 400).toFixed(1)}px`;
      } else {
        if (labelBottomRef.current) labelBottomRef.current.style.fontSize = cornerFs;
      }
      for (const ref of [...leftTickRefs.current, ...rightTickRefs.current]) {
        if (ref) ref.style.fontSize = tickFs;
      }
    }
  }, [size, leftLabel, rightLabel, bottomLabel]);

  // ── Left needle ───────────────────────────────────────────────────────────
  useEffect(() => {
    if (leftPct == null || !leftNeedleRef.current) return;
    if (leftDisplayMax <= leftDisplayMin) return;
    const center    = (leftDisplayMin + leftDisplayMax) / 2;
    const halfRange = (leftDisplayMax - leftDisplayMin) / 2;
    const clamped   = leftClamp ? Math.max(leftClampMin, Math.min(leftClampMax, leftPct)) : leftPct;
    leftNeedleRef.current.style.transform = `rotate(${(center - clamped) * (45 / halfRange)}deg)`;
  }, [leftPct, leftDisplayMin, leftDisplayMax, leftClamp, leftClampMin, leftClampMax]);

  // ── Right needle ──────────────────────────────────────────────────────────
  useEffect(() => {
    if (rightPct == null || !rightNeedleRef.current) return;
    if (rightDisplayMax <= rightDisplayMin) return;
    const center    = (rightDisplayMin + rightDisplayMax) / 2;
    const halfRange = (rightDisplayMax - rightDisplayMin) / 2;
    const clamped   = rightClamp ? Math.max(rightClampMin, Math.min(rightClampMax, rightPct)) : rightPct;
    rightNeedleRef.current.style.transform = `rotate(${(clamped - center) * (45 / halfRange)}deg)`;
  }, [rightPct, rightDisplayMin, rightDisplayMax, rightClamp, rightClampMin, rightClampMax]);

  // ── Corner label updates ──────────────────────────────────────────────────
  useEffect(() => { if (labelTopRef.current)    labelTopRef.current.textContent    = topLabel;    }, [topLabel]);
  useEffect(() => {
    const b1 = labelBottomRef.current;
    const b2 = labelBottom2Ref.current;
    if (!b1 || !b2) return;
    const parts = bottomLabel.split(",").map((s) => s.trim());
    if (parts.length >= 2) {
      b1.textContent = parts[0]!;
      b1.style.left  = "42.5%";
      b2.textContent = parts[1]!;
      b2.style.left  = "57.5%";
      b2.style.top   = "80%";
      b2.style.display = "";
    } else {
      b1.textContent = parts[0] ?? bottomLabel;
      b1.style.left  = "50%";
      b2.style.display = "none";
    }
  }, [bottomLabel]);
  useEffect(() => {
    if (!labelLeftRef.current) return;
    labelLeftRef.current.textContent  = leftLabelVertical ? leftLabel.split("").join("\n") : leftLabel;
    labelLeftRef.current.style.whiteSpace = leftLabelVertical ? "pre" : "nowrap";
  }, [leftLabel, leftLabelVertical]);
  useEffect(() => {
    if (!labelRightRef.current) return;
    labelRightRef.current.textContent  = rightLabelVertical ? rightLabel.split("").join("\n") : rightLabel;
    labelRightRef.current.style.whiteSpace = rightLabelVertical ? "pre" : "nowrap";
  }, [rightLabel, rightLabelVertical]);

  // ── Left color arcs ───────────────────────────────────────────────────────
  // Zones rendered in order (zone 1 bottom, last zone topmost).
  // Use JSON.stringify as dep key so effect fires when any zone value changes.
  const leftZonesKey       = JSON.stringify(leftZones);
  const rightZonesKey      = JSON.stringify(rightZones);
  const leftTickLabelsKey     = JSON.stringify(leftTickLabels);
  const rightTickLabelsKey    = JSON.stringify(rightTickLabels);
  const leftTickPositionsKey  = JSON.stringify(leftTickPositions);
  const rightTickPositionsKey = JSON.stringify(rightTickPositions);
  useEffect(() => {
    if (!leftArcGRef.current) return;
    drawColorArcs(leftArcGRef.current, leftDisplayMin, leftDisplayMax, 30, 200, true, leftZones);
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [leftDisplayMin, leftDisplayMax, leftZonesKey]);

  // ── Right color arcs ──────────────────────────────────────────────────────
  useEffect(() => {
    if (!rightArcGRef.current) return;
    drawColorArcs(rightArcGRef.current, rightDisplayMin, rightDisplayMax, 370, 200, false, rightZones);
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rightDisplayMin, rightDisplayMax, rightZonesKey]);

  // ── Left ticks (SVG lines + label positions + text + visibility) ──────────
  useEffect(() => {
    const leftTicks = leftTickRefs.current;
    if (leftDisplayMax <= leftDisplayMin) {
      if (leftTickGRef.current) { while (leftTickGRef.current.firstChild) leftTickGRef.current.removeChild(leftTickGRef.current.firstChild); }
      leftTicks.forEach((t) => { if (t) t.style.display = "none"; });
      return;
    }
    const count = Math.max(2, leftTickCount);
    const majorAngles = Array.from({ length: count }, (_, i) => {
      const val = leftTickPositions?.[i] ?? (leftDisplayMin + (i / (count - 1)) * (leftDisplayMax - leftDisplayMin));
      return leftAngleDeg(val, leftDisplayMin, leftDisplayMax);
    });
    if (leftTickGRef.current) drawTickLines(leftTickGRef.current, majorAngles, 30, 200, leftSubTicks);
    for (let i = 0; i < MAX_TICKS; i++) {
      const t = leftTicks[i];
      if (!t) continue;
      if (i < count) {
        const rad = (majorAngles[i]! * Math.PI) / 180;
        t.style.left    = `${((30  + 107 * Math.cos(rad)) / 400 * 100).toFixed(3)}%`;
        t.style.top     = `${((200 + 107 * Math.sin(rad)) / 400 * 100).toFixed(3)}%`;
        const autoVal = leftTickPositions?.[i] ?? (leftDisplayMin + (i / (count - 1)) * (leftDisplayMax - leftDisplayMin));
        t.textContent   = leftTickLabels?.[i] ?? autoVal.toFixed(leftTickPrecision);
        t.style.display = "";
      } else {
        t.style.display = "none";
      }
    }
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [leftTickCount, leftDisplayMin, leftDisplayMax, leftTickPrecision, leftSubTicks, leftTickLabelsKey, leftTickPositionsKey]);

  // ── Right ticks (SVG lines + label positions + text + visibility) ─────────
  useEffect(() => {
    const rightTicks = rightTickRefs.current;
    if (rightDisplayMax <= rightDisplayMin) {
      if (rightTickGRef.current) { while (rightTickGRef.current.firstChild) rightTickGRef.current.removeChild(rightTickGRef.current.firstChild); }
      rightTicks.forEach((t) => { if (t) t.style.display = "none"; });
      return;
    }
    const count = Math.max(2, rightTickCount);
    const majorAngles = Array.from({ length: count }, (_, i) => {
      const val = rightTickPositions?.[i] ?? (rightDisplayMin + (i / (count - 1)) * (rightDisplayMax - rightDisplayMin));
      return rightAngleDeg(val, rightDisplayMin, rightDisplayMax);
    });
    if (rightTickGRef.current) drawTickLines(rightTickGRef.current, majorAngles, 370, 200, rightSubTicks);
    for (let i = 0; i < MAX_TICKS; i++) {
      const t = rightTicks[i];
      if (!t) continue;
      if (i < count) {
        const rad = (majorAngles[i]! * Math.PI) / 180;
        t.style.left    = `${((370 + 107 * Math.cos(rad)) / 400 * 100).toFixed(3)}%`;
        t.style.top     = `${((200 + 107 * Math.sin(rad)) / 400 * 100).toFixed(3)}%`;
        const autoVal = rightTickPositions?.[i] ?? (rightDisplayMin + (i / (count - 1)) * (rightDisplayMax - rightDisplayMin));
        t.textContent   = rightTickLabels?.[i] ?? autoVal.toFixed(rightTickPrecision);
        t.style.display = "";
      } else {
        t.style.display = "none";
      }
    }
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [rightTickCount, rightDisplayMin, rightDisplayMax, rightTickPrecision, rightSubTicks, rightTickLabelsKey, rightTickPositionsKey]);



  return <div ref={containerRef} />;
}
