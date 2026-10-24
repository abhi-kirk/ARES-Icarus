import type { EChartsOption } from "echarts";
import { BUFFER, type Column } from "./model";

const INK = "#d7ded8";
const DIM = "rgba(215, 222, 216, 0.45)";
const GOLD = "#e2a43a";
const HEIGHT = "#d7ded8";

const LABELS: Record<Column, string> = {
  height: "Height (m)",
  rate: "Rate (m/s)",
};

export function liveOption(t: number): EChartsOption {
  return buildOption(["height", "rate"], "Height and rate", t);
}

export function buildOption(columns: Column[], title: string, t: number): EChartsOption {
  const series = columns.map((column) => ({
    name: LABELS[column],
    type: "line" as const,
    showSymbol: false,
    data: BUFFER.map((row) => [row.t, row[column]]),
    lineStyle: { width: 2, color: column === "rate" ? GOLD : HEIGHT },
    itemStyle: { color: column === "rate" ? GOLD : HEIGHT },
    markLine: {
      symbol: "none",
      animation: false,
      lineStyle: { color: GOLD, width: 1 },
      label: { show: false },
      data: [{ xAxis: t }],
    },
  }));

  return {
    backgroundColor: "transparent",
    animation: false,
    title: {
      text: title,
      left: 8,
      top: 4,
      textStyle: { color: DIM, fontSize: 11, fontFamily: "Fragment Mono, ui-monospace, monospace", fontWeight: 400 },
    },
    legend: {
      top: 4,
      right: 8,
      textStyle: { color: INK, fontSize: 11, fontFamily: "Fragment Mono, ui-monospace, monospace" },
      icon: "rect",
      itemWidth: 10,
      itemHeight: 2,
    },
    grid: { left: 48, right: 16, top: 36, bottom: 28 },
    tooltip: { trigger: "axis" },
    xAxis: {
      type: "value",
      name: "t (s)",
      nameTextStyle: { color: DIM, fontFamily: "Fragment Mono, ui-monospace, monospace" },
      axisLabel: { color: DIM, fontFamily: "Fragment Mono, ui-monospace, monospace" },
      axisLine: { lineStyle: { color: DIM } },
      splitLine: { lineStyle: { color: "rgba(215, 222, 216, 0.08)" } },
    },
    yAxis: {
      type: "value",
      axisLabel: { color: DIM, fontFamily: "Fragment Mono, ui-monospace, monospace" },
      splitLine: { lineStyle: { color: "rgba(215, 222, 216, 0.08)" } },
    },
    series,
  };
}
