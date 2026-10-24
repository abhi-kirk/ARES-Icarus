import type { EChartsOption } from "echarts";
import { axisOf, durationOf, plotOf, sceneById, type Column, type Knobs, type SceneId } from "./scenes";

const GOLD = "#e2a43a";
const INK = "#d7ded8";
const GRID = "#243038";
const MUTED = "#8b9690";

export function liveOption(scene: SceneId, knobs: Knobs, time: number): EChartsOption {
  const spec = sceneById(scene);
  return option(scene, spec.live, knobs, time, spec.chartTitle, time);
}

export function chartOption(scene: SceneId, knobs: Knobs, columns: Column[], title: string, mark: number): EChartsOption {
  return option(scene, columns, knobs, durationOf(scene), title, mark);
}

function option(
  scene: SceneId,
  columns: Column[],
  knobs: Knobs,
  until: number,
  title: string,
  mark: number,
): EChartsOption {
  const series = plotOf(scene, columns, knobs, until);
  const axis = axisOf(scene, columns, knobs);
  const hasSecond = series.some((item) => item.axis === 1);

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
      top: 6,
      right: hasSecond ? 72 : 12,
      textStyle: { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 },
      itemWidth: 14,
      itemHeight: 2,
    },
    grid: { left: 52, right: hasSecond ? 52 : 16, top: 40, bottom: 32 },
    tooltip: { show: false },
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
    series: series.map((item, index) => ({
      name: item.name,
      type: "line" as const,
      yAxisIndex: item.axis,
      showSymbol: false,
      data: item.points,
      lineStyle: { color: item.gold ? GOLD : INK, width: item.gold ? 2 : 1.5 },
      markLine:
        index === 0
          ? {
              silent: true,
              symbol: "none",
              lineStyle: { color: GOLD, type: "solid" as const, width: 1 },
              label: { show: false },
              data: [{ xAxis: mark }],
            }
          : undefined,
    })),
  };
}
