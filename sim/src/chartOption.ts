import type { EChartsOption } from "echarts";
import {
  axisOf,
  chartMarkers,
  columnLabel,
  columnUnit,
  durationOf,
  isFrequencyScene,
  plotOf,
  rowsFor,
  sceneById,
  springHeight,
  springRate,
  type Column,
  type Knobs,
  type SceneId,
} from "./scenes";
import { freqBode } from "./frequencyPlant";

const GOLD = "#e2a43a";
const INK = "#d7ded8";
const GRID = "#243038";
const MUTED = "#8b9690";
const PANEL = "rgba(12, 16, 20, 0.94)";

/** Data traces on #000: hues spaced so neighbors are never two warm tones. */
export const STAGE_DATA_COLORS = ["#e2a43a", "#5eead4", "#c084fc", "#86efac", "#f472b6", "#d7ded8"] as const;

const MUTED_COLUMNS = new Set<Column>(["reference", "truth", "command"]);

const DATA_LINE_WIDTH = 3;
const MUTED_LINE_WIDTH = 2;

type LineKind = "solid" | "dashed";

function dataColorIndex(columns: Column[], index: number): number {
  let slot = 0;
  for (let i = 0; i < index; i++) {
    if (!MUTED_COLUMNS.has(columns[i])) slot += 1;
  }
  return slot;
}

export function liveSeriesColor(scene: SceneId, column: Column): string | null {
  const columns = sceneById(scene).live;
  const index = columns.indexOf(column);
  if (index < 0) return null;
  return seriesLook(columns, index).color;
}

function seriesLook(columns: Column[], index: number): { color: string; width: number; kind: LineKind } {
  const column = columns[index];
  if (MUTED_COLUMNS.has(column)) {
    return { color: MUTED, width: MUTED_LINE_WIDTH, kind: "dashed" };
  }
  const slot = dataColorIndex(columns, index);
  return {
    color: STAGE_DATA_COLORS[slot % STAGE_DATA_COLORS.length],
    width: DATA_LINE_WIDTH,
    kind: "solid",
  };
}

function formatAxisValue(value: number): string {
  const abs = Math.abs(value);
  if (abs < 5e-4) return "0";
  // Long digit strings on runaway axes; keep ticks short.
  if (abs >= 1e5) return value.toExponential(1).replace(/e\+?/, "e");
  if (abs >= 100) return value.toFixed(0);
  if (abs >= 10) return value.toFixed(1);
  if (abs >= 1) return value.toFixed(2);
  return value.toFixed(3);
}

const LIVE_WINDOW_S = 1;

export function liveDomain(scene: SceneId, time: number): number {
  const duration = durationOf(scene);
  const minimum = Math.min(duration, LIVE_WINDOW_S);
  if (time <= minimum) return minimum;
  return Math.min(duration, time);
}

export function liveOption(scene: SceneId, knobs: Knobs, time: number): EChartsOption {
  const spec = sceneById(scene);
  // Spring and step: phase loops. The step scene overlays the numerical orbit
  // (spiraling under explicit Euler) on the true ellipse.
  if (scene === "state") {
    return phaseOption(scene, knobs, "height", ["rate"], spec.chartTitle, time);
  }
  if (scene === "step") {
    return stepPhaseOption(knobs, spec.chartTitle, time);
  }
  if (isFrequencyScene(scene)) return bodeOption(scene, knobs);
  return option(scene, spec.live, knobs, time, spec.chartTitle, time, true, liveDomain(scene, time), true);
}

/** Log-frequency Bode plot. The crossover mark is not a time, so this chart does not scrub. */
function bodeOption(scene: SceneId, knobs: Knobs): EChartsOption {
  if (!isFrequencyScene(scene)) return option(scene, ["height"], knobs, 0, "", 0);
  const curve = freqBode(scene, knobs);
  const margins = curve.margins;
  const labelFont = { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 };
  const axisLabel = { ...labelFont, formatter: (value: number) => formatAxisValue(value) };
  let phaseLo = -190;
  let phaseHi = -80;
  let magLo = -12;
  let magHi = 8;
  for (const point of curve.points) {
    if (point.w > 16) continue;
    phaseLo = Math.min(phaseLo, point.phaseDeg);
    phaseHi = Math.max(phaseHi, point.phaseDeg);
    magLo = Math.min(magLo, point.magDb);
    magHi = Math.max(magHi, point.magDb);
  }
  phaseHi = Math.min(10, phaseHi + 12);
  phaseLo = Math.max(-360, phaseLo - 12);
  magHi += 3;
  magLo = Math.max(-40, magLo - 3);
  const title = sceneById(scene).chartTitle;
  const magColor = STAGE_DATA_COLORS[0];
  const phaseColor = STAGE_DATA_COLORS[1];
  const marks: { xAxis?: number; yAxis?: number; label?: object; lineStyle?: object }[] = [
    {
      yAxis: 0,
      label: { show: false },
      lineStyle: { color: GRID, type: "dashed", width: 1 },
    },
  ];
  if (margins.wc !== null) {
    marks.push({
      xAxis: margins.wc,
      label: {
        formatter: "crossover",
        color: GOLD,
        fontFamily: "Fragment Mono, monospace",
        fontSize: 10,
        position: "insideEndTop",
      },
      lineStyle: { color: GOLD, type: "solid", width: 1 },
    });
  }

  return {
    animation: false,
    backgroundColor: "transparent",
    title: {
      text: title,
      left: 8,
      top: 4,
      textStyle: { color: INK, fontSize: 13, fontWeight: 500, fontFamily: "Barlow, Helvetica Neue, sans-serif" },
    },
    legend: {
      left: 8,
      top: 26,
      textStyle: { color: INK, fontFamily: "Fragment Mono, monospace", fontSize: 11, fontWeight: 700 },
      itemWidth: 26,
      itemHeight: 5,
      itemGap: 18,
    },
    grid: { left: 64, right: 64, top: 52, bottom: 36 },
    tooltip: {
      show: true,
      trigger: "axis",
      confine: true,
      backgroundColor: PANEL,
      borderColor: GRID,
      borderWidth: 1,
      padding: [8, 10],
      textStyle: { color: INK, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
      axisPointer: {
        type: "line",
        lineStyle: { color: GOLD, width: 1 },
        label: {
          show: true,
          backgroundColor: PANEL,
          borderColor: GRID,
          borderWidth: 1,
          color: INK,
          fontFamily: "Fragment Mono, monospace",
          fontSize: 10,
          formatter: (params) => {
            const value = typeof params.value === "number" ? params.value : Number(params.value);
            return `ω = ${formatAxisValue(value)}`;
          },
        },
      },
      formatter: (raw) => {
        const items = (Array.isArray(raw) ? raw : [raw]).filter(Boolean);
        if (!items.length) return "";
        const head = items[0] as { axisValue?: number; data?: [number, number] | number };
        const freq = typeof head.axisValue === "number" ? head.axisValue : 0;
        const lines = [`<span style="color:${MUTED}">ω</span> = ${formatAxisValue(freq)} rad/s`];
        for (const item of items) {
          const pair = (item as { data?: [number, number] | number }).data;
          const y = Array.isArray(pair) ? pair[1] : Number(pair);
          const name = (item as { seriesName?: string; marker?: string }).seriesName ?? "";
          const marker = (item as { marker?: string }).marker ?? "";
          const suffix = name.startsWith("Phase") ? "°" : " dB";
          lines.push(`${marker}<span style="color:${INK}">${name}</span>: ${formatAxisValue(y)}${suffix}`);
        }
        return lines.join("<br/>");
      },
    },
    xAxis: {
      id: "w",
      type: "log",
      min: 0.25,
      max: 30,
      logBase: 10,
      name: "ω (rad/s)",
      nameLocation: "middle",
      nameGap: 22,
      nameTextStyle: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
      axisLine: { lineStyle: { color: GRID } },
      axisTick: { show: false },
      axisLabel,
      splitLine: { lineStyle: { color: GRID } },
    },
    yAxis: [
      {
        id: "db",
        type: "value",
        min: magLo,
        max: magHi,
        name: "dB",
        nameLocation: "middle",
        nameGap: 40,
        nameTextStyle: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
        axisLine: { show: false },
        axisTick: { show: false },
        axisLabel,
        splitLine: { lineStyle: { color: GRID } },
      },
      {
        id: "deg",
        type: "value",
        min: phaseLo,
        max: phaseHi,
        name: "deg",
        nameLocation: "middle",
        nameGap: 40,
        nameTextStyle: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
        axisLine: { show: false },
        axisTick: { show: false },
        axisLabel,
        splitLine: { show: false },
      },
    ],
    series: [
      {
        name: "Magnitude",
        type: "line",
        yAxisIndex: 0,
        showSymbol: false,
        clip: true,
        data: curve.points.map((point) => [point.w, point.magDb]),
        color: magColor,
        lineStyle: { color: magColor, width: DATA_LINE_WIDTH },
        markLine: {
          silent: true,
          symbol: "none",
          data: marks,
        },
      },
      {
        name: "Phase",
        type: "line",
        yAxisIndex: 1,
        showSymbol: false,
        clip: true,
        data: curve.points.map((point) => [point.w, point.phaseDeg]),
        color: phaseColor,
        lineStyle: { color: phaseColor, width: DATA_LINE_WIDTH },
        markLine: {
          silent: true,
          symbol: "none",
          label: {
            formatter: "−180°",
            color: MUTED,
            fontFamily: "Fragment Mono, monospace",
            fontSize: 10,
          },
          lineStyle: { color: MUTED, type: "dashed", width: 1 },
          data: [{ yAxis: -180 }],
        },
      },
    ],
  };
}

export function chartOption(scene: SceneId, knobs: Knobs, columns: Column[], title: string, mark: number, x?: Column): EChartsOption {
  if (x) return phaseOption(scene, knobs, x, columns, title, mark);
  return option(scene, columns, knobs, durationOf(scene), title, mark);
}

function axisUnit(series: { column: Column; axis: number }[], axisIndex: number): string {
  const units = new Set<string>();
  for (const item of series) {
    if (item.axis !== axisIndex) continue;
    const unit = columnUnit(item.column);
    if (unit === "unitless") continue;
    if (unit === "m/s2") units.add("m/s²");
    else units.add(unit);
  }
  if (units.size !== 1) return "";
  return [...units][0];
}

function option(
  scene: SceneId,
  columns: Column[],
  knobs: Knobs,
  until: number,
  title: string,
  mark: number,
  interactive = true,
  xMax = durationOf(scene),
  tip = false,
): EChartsOption {
  const series = plotOf(scene, columns, knobs, until);
  const axis = axisOf(scene, columns, knobs);
  const hasSecond = series.some((item) => item.axis === 1);
  const leftUnit = axisUnit(series, 0);
  const rightUnit = axisUnit(series, 1);
  const markers = chartMarkers(scene, knobs).filter((item) => item.t <= xMax + 1e-9);
  const labelFont = { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 };
  const axisLabel = { ...labelFont, formatter: (value: number) => formatAxisValue(value) };
  // A hidden 0-to-1 axis sticks around and steals the series, so only the
  // scales a series actually uses go into the chart.
  const yAxis: EChartsOption["yAxis"] = [
    {
      id: "y",
      type: "value",
      min: axis.y[0],
      max: axis.y[1],
      // Keep left/right zero lines aligned when both are linear.
      alignTicks: true,
      name: leftUnit,
      nameLocation: "middle",
      nameGap: 40,
      nameTextStyle: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
      axisLine: { show: false },
      axisTick: { show: false },
      axisLabel,
      splitLine: { lineStyle: { color: GRID } },
    },
  ];
  if (hasSecond && axis.y2) {
    const logRight = Boolean(axis.y2Log);
    yAxis.push({
      id: "y2",
      type: logRight ? "log" : "value",
      min: axis.y2[0],
      max: axis.y2[1],
      alignTicks: !logRight,
      name: rightUnit,
      nameLocation: "middle",
      nameGap: 40,
      nameTextStyle: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
      axisLine: { show: false },
      axisTick: { show: false },
      axisLabel,
      splitLine: { show: false },
      ...(logRight ? { logBase: 10 } : {}),
    });
  }

  return {
    animation: false,
    backgroundColor: "transparent",
    title: {
      text: title,
      left: 8,
      top: 4,
      textStyle: { color: INK, fontSize: 13, fontWeight: 500, fontFamily: "Barlow, Helvetica Neue, sans-serif" },
    },
    legend: {
      left: 8,
      top: 26,
      textStyle: {
        color: INK,
        fontFamily: "Fragment Mono, monospace",
        fontSize: 11,
        fontWeight: 700,
      },
      itemWidth: 26,
      itemHeight: 5,
      itemGap: 18,
    },
    grid: {
      left: leftUnit ? 64 : 52,
      right: hasSecond ? (rightUnit ? 64 : 52) : 16,
      top: 52,
      bottom: 36,
    },
    tooltip: interactive ? {
      show: true,
      trigger: "axis",
      confine: true,
      backgroundColor: PANEL,
      borderColor: GRID,
      borderWidth: 1,
      padding: [8, 10],
      textStyle: { color: INK, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
      axisPointer: {
        type: "line",
        lineStyle: { color: GOLD, width: 1 },
        label: {
          show: true,
          backgroundColor: PANEL,
          borderColor: GRID,
          borderWidth: 1,
          color: INK,
          fontFamily: "Fragment Mono, monospace",
          fontSize: 10,
          formatter: (params) => {
            const value = typeof params.value === "number" ? params.value : Number(params.value);
            return `t = ${formatAxisValue(value)} s`;
          },
        },
      },
      formatter: (raw) => {
        const items = (Array.isArray(raw) ? raw : [raw]).filter(Boolean);
        if (!items.length) return "";
        const head = items[0];
        const axisT = (head as { axisValue?: number }).axisValue;
        const t =
          typeof axisT === "number"
            ? axisT
            : Array.isArray(head.data)
              ? (head.data as [number, number])[0]
              : 0;
        const lines = [`<span style="color:${MUTED}">t</span> = ${formatAxisValue(t)} s`];
        for (const item of items) {
          const pair = item.data as [number, number] | number;
          const y = Array.isArray(pair) ? pair[1] : Number(pair);
          lines.push(`${item.marker}<span style="color:${INK}">${item.seriesName}</span>: ${formatAxisValue(y)}`);
        }
        return lines.join("<br/>");
      },
    } : { show: false },
    xAxis: {
      id: "t",
      type: "value",
      min: 0,
      max: xMax,
      name: "t (s)",
      nameLocation: "middle",
      nameGap: 22,
      nameTextStyle: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
      axisLine: { lineStyle: { color: GRID } },
      axisTick: { show: false },
      axisLabel,
      splitLine: { lineStyle: { color: GRID } },
    },
    yAxis,
    color: [...STAGE_DATA_COLORS],
    series: series.map((item, index) => {
      const look = seriesLook(columns, index);
      return {
      name: item.name,
      type: "line" as const,
      yAxisIndex: item.axis,
      showSymbol: false,
      data: item.points,
      color: look.color,
      itemStyle: { color: look.color },
      lineStyle: { color: look.color, width: look.width, type: look.kind },
      markPoint:
        tip && item.points.length
          ? {
              silent: true,
              symbol: "circle",
              symbolSize: 8,
              itemStyle: { color: look.color },
              label: { show: false },
              data: [{ name: "", coord: item.points[item.points.length - 1] }],
            }
          : undefined,
      markLine:
        index === 0
          ? {
              silent: true,
              symbol: "none",
              lineStyle: { color: GOLD, type: "solid" as const, width: 1 },
              label: { ...labelFont, show: false },
              data: [
                {
                  xAxis: mark,
                  label: {
                    show: true,
                    formatter: `${formatAxisValue(mark)} s`,
                    position: "insideEndTop" as const,
                    color: GOLD,
                    fontFamily: "Fragment Mono, monospace",
                    fontSize: 10,
                  },
                },
                ...markers.map((marker, markerIndex) => ({
                  xAxis: marker.t,
                  name: marker.label,
                  lineStyle: { color: GRID, type: "dashed" as const, width: 1 },
                  label: {
                    // Sit above the plot so labels do not collide with x-axis ticks.
                    show: marker.label !== "sample",
                    formatter: marker.label,
                    position: "insideEndTop" as const,
                    distance: 4 + (markerIndex % 3) * 12,
                    ...labelFont,
                  },
                })),
              ],
            }
          : undefined,
    };
    }),
  };
}

function phaseOption(
  scene: SceneId,
  knobs: Knobs,
  x: Column,
  columns: Column[],
  title: string,
  mark: number,
): EChartsOption {
  const rows = rowsFor(scene, knobs);
  const selected: Column[] = columns.length ? columns : ["rate"];
  const current = rows.reduce((chosen, row) => (row.t <= mark + 1e-9 ? row : chosen), rows[0]);
  const xValues = rows.map((row) => row[x]);
  const yValues = selected.flatMap((column) => rows.map((row) => row[column]));
  const xAxis = padValues(xValues);
  const yAxis = padValues(yValues);

  return {
    animation: false,
    backgroundColor: "transparent",
    title: {
      text: title,
      left: 8,
      top: 4,
      textStyle: { color: INK, fontSize: 13, fontWeight: 500, fontFamily: "Barlow, Helvetica Neue, sans-serif" },
    },
    legend: {
      left: 8,
      top: 26,
      textStyle: { color: INK, fontFamily: "Fragment Mono, monospace", fontSize: 11, fontWeight: 700 },
      itemWidth: 26,
      itemHeight: 5,
      itemGap: 18,
    },
    grid: { left: 52, right: 16, top: 48, bottom: 36 },
    tooltip: {
      trigger: "item",
      confine: true,
      backgroundColor: PANEL,
      borderColor: GRID,
      borderWidth: 1,
      padding: [8, 10],
      textStyle: { color: INK, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
    },
    xAxis: {
      type: "value",
      min: xAxis[0],
      max: xAxis[1],
      name: columnLabel(x),
      nameLocation: "middle",
      nameGap: 24,
      nameTextStyle: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
      axisLine: { lineStyle: { color: GRID } },
      axisTick: { show: false },
      axisLabel: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 },
      splitLine: { lineStyle: { color: GRID } },
    },
    yAxis: {
      type: "value",
      min: yAxis[0],
      max: yAxis[1],
      axisLine: { show: false },
      axisTick: { show: false },
      axisLabel: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 },
      splitLine: { lineStyle: { color: GRID } },
    },
    color: [...STAGE_DATA_COLORS],
    series: [
      ...selected.map((column, index) => {
        const look = seriesLook(selected, index);
        return {
          name: columnLabel(column),
          type: "line" as const,
          showSymbol: false,
          data: rows.map((row) => [row[x], row[column]]),
          color: look.color,
          itemStyle: { color: look.color },
          lineStyle: { color: look.color, width: look.width, type: look.kind },
        };
      }),
      {
        name: "Current state",
        type: "scatter" as const,
        symbolSize: 8,
        data: selected.slice(0, 1).map((column) => [current[x], current[column]]),
        color: GOLD,
        itemStyle: { color: GOLD },
      },
    ],
  };
}

function padValues(values: number[]): [number, number] {
  let lo = Math.min(...values);
  let hi = Math.max(...values);
  const span = hi - lo || 1;
  lo -= span * 0.08;
  hi += span * 0.08;
  return [lo, hi];
}

function stepPhaseOption(knobs: Knobs, title: string, mark: number): EChartsOption {
  const rows = rowsFor("step", knobs);
  const current = rows.reduce((chosen, row) => (row.t <= mark + 1e-9 ? row : chosen), rows[0]);
  const truth: [number, number][] = [];
  const end = durationOf("step");
  for (let t = 0; t <= end + 1e-9; t += 0.02) {
    truth.push([springHeight(t), springRate(t)]);
  }
  const stepped = rows.map((row) => [row.height, row.rate] as [number, number]);
  const xValues = [...truth.map((point) => point[0]), ...stepped.map((point) => point[0])];
  const yValues = [...truth.map((point) => point[1]), ...stepped.map((point) => point[1])];
  const xAxis = padValues(xValues);
  const yAxis = padValues(yValues);

  return {
    animation: false,
    backgroundColor: "transparent",
    title: {
      text: title,
      left: 8,
      top: 4,
      textStyle: { color: INK, fontSize: 13, fontWeight: 500, fontFamily: "Barlow, Helvetica Neue, sans-serif" },
    },
    legend: {
      left: 8,
      top: 26,
      textStyle: { color: INK, fontFamily: "Fragment Mono, monospace", fontSize: 11, fontWeight: 700 },
      itemWidth: 26,
      itemHeight: 5,
      itemGap: 18,
    },
    grid: { left: 52, right: 16, top: 48, bottom: 36 },
    tooltip: {
      trigger: "item",
      confine: true,
      backgroundColor: PANEL,
      borderColor: GRID,
      borderWidth: 1,
      padding: [8, 10],
      textStyle: { color: INK, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
    },
    xAxis: {
      type: "value",
      min: xAxis[0],
      max: xAxis[1],
      name: "Height (m)",
      nameLocation: "middle",
      nameGap: 24,
      nameTextStyle: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
      axisLine: { lineStyle: { color: GRID } },
      axisTick: { show: false },
      axisLabel: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 },
      splitLine: { lineStyle: { color: GRID } },
    },
    yAxis: {
      type: "value",
      min: yAxis[0],
      max: yAxis[1],
      name: "Rate (m/s)",
      nameLocation: "middle",
      nameGap: 36,
      nameTextStyle: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
      axisLine: { show: false },
      axisTick: { show: false },
      axisLabel: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 },
      splitLine: { lineStyle: { color: GRID } },
    },
    color: [...STAGE_DATA_COLORS],
    series: [
      {
        name: "True ellipse",
        type: "line" as const,
        showSymbol: false,
        data: truth,
        color: MUTED,
        itemStyle: { color: MUTED },
        lineStyle: { color: MUTED, width: MUTED_LINE_WIDTH, type: "dashed" },
      },
      {
        name: "Step orbit",
        type: "line" as const,
        showSymbol: false,
        data: stepped,
        color: GOLD,
        itemStyle: { color: GOLD },
        lineStyle: { color: GOLD, width: DATA_LINE_WIDTH, type: "solid" },
      },
      {
        name: "Current state",
        type: "scatter" as const,
        symbolSize: 8,
        data: [[current.height, current.rate]],
        color: GOLD,
        itemStyle: { color: GOLD },
      },
    ],
  };
}
