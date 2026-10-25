import type { EChartsOption } from "echarts";
import { axisOf, chartMarkers, columnLabel, durationOf, plotOf, rowsFor, sceneById, type Column, type Knobs, type SceneId } from "./scenes";

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
  if (abs >= 100) return value.toFixed(0);
  if (abs >= 10) return value.toFixed(1);
  if (abs >= 1) return value.toFixed(2);
  return value.toFixed(3);
}

export function liveOption(scene: SceneId, knobs: Knobs, time: number): EChartsOption {
  const spec = sceneById(scene);
  return option(scene, spec.live, knobs, time, spec.chartTitle, time, false);
}

export function chartOption(scene: SceneId, knobs: Knobs, columns: Column[], title: string, mark: number, x?: Column): EChartsOption {
  if (x) return phaseOption(scene, knobs, x, columns, title, mark);
  return option(scene, columns, knobs, durationOf(scene), title, mark);
}

function option(
  scene: SceneId,
  columns: Column[],
  knobs: Knobs,
  until: number,
  title: string,
  mark: number,
  interactive = true,
): EChartsOption {
  const series = plotOf(scene, columns, knobs, until);
  const axis = axisOf(scene, columns, knobs);
  const hasSecond = series.some((item) => item.axis === 1);
  const markers = chartMarkers(scene, knobs).filter((item) => item.t <= durationOf(scene));

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
    grid: { left: 52, right: hasSecond ? 52 : 16, top: 48, bottom: 32 },
    tooltip: interactive ? {
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
      type: "value",
      min: 0,
      max: durationOf(scene),
      name: "t (s)",
      nameLocation: "middle",
      nameGap: 22,
      nameTextStyle: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 11 },
      axisLine: { lineStyle: { color: GRID } },
      axisTick: { show: false },
      axisLabel: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 },
      splitLine: { lineStyle: { color: GRID } },
    },
    yAxis: [
      {
        type: "value",
        min: axis.y[0],
        max: axis.y[1],
        axisLine: { show: false },
        axisTick: { show: false },
        axisLabel: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 },
        splitLine: { lineStyle: { color: GRID } },
      },
      {
        type: "value",
        min: axis.y2 ? axis.y2[0] : 0,
        max: axis.y2 ? axis.y2[1] : 1,
        show: Boolean(axis.y2),
        axisLine: { show: false },
        axisTick: { show: false },
        axisLabel: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 },
        splitLine: { show: false },
      },
    ],
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
      markLine:
        index === 0
          ? {
              silent: true,
              symbol: "none",
              lineStyle: { color: GOLD, type: "solid" as const, width: 1 },
              label: { show: false, color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 },
              data: [
                { xAxis: mark },
                ...markers.map((item) => ({
                  xAxis: item.t,
                  name: item.label,
                  lineStyle: { color: GRID, type: "dashed" as const, width: 1 },
                  label: { show: item.label !== "sample", formatter: item.label, position: "insideEndTop" as const },
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
