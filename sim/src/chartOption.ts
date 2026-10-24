import type { EChartsOption } from "echarts";
import { BUFFER, T_END, height, rate, type Column, type Sample } from "./model";

const INK = "#d7ded8";
const DIM = "rgba(215, 222, 216, 0.45)";
const GOLD = "#e2a43a";
const HEIGHT = "#d7ded8";

const LABELS: Record<Column, string> = {
  height: "Height (m)",
  rate: "Rate (m/s)",
};

export function liveOption(t: number): EChartsOption {
  return buildOption(["height", "rate"], "Height and rate", t, "live");
}

// The live strip is a pen. It draws only the samples the bead has reached.
// A chart asked for in chat is the whole series, with the same time marked.
function rowsFor(t: number, mode: "live" | "full"): Sample[] {
  if (mode === "full") return BUFFER;
  const rows = BUFFER.filter((row) => row.t <= t + 1e-9);
  const tip = rows[rows.length - 1];
  if (!tip || Math.abs(tip.t - t) > 1e-4) {
    rows.push({ t, height: height(t), rate: rate(t) });
  }
  return rows;
}

export function buildOption(
  columns: Column[],
  title: string,
  t: number,
  mode: "live" | "full" = "full",
): EChartsOption {
  const rows = rowsFor(t, mode);
  const series = columns.map((column) => {
    const color = column === "rate" ? GOLD : HEIGHT;
    const tip = rows[rows.length - 1];
    return {
      name: LABELS[column],
      type: "line" as const,
      showSymbol: false,
      data: rows.map((row) => [row.t, row[column]]),
      lineStyle: { width: 2, color },
      itemStyle: { color },
      markPoint: tip
        ? {
            symbol: "circle",
            symbolSize: 8,
            itemStyle: { color },
            label: { show: false },
            data: [{ name: column, coord: [tip.t, tip[column]] as [number, number] }],
          }
        : undefined,
      markLine: {
        symbol: "none",
        animation: false,
        lineStyle: { color: GOLD, width: 1 },
        label: { show: false },
        data: [{ xAxis: t }],
      },
    };
  });

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
    tooltip: { show: false },
    xAxis: {
      type: "value",
      min: 0,
      max: T_END,
      name: "t (s)",
      nameTextStyle: { color: DIM, fontFamily: "Fragment Mono, ui-monospace, monospace" },
      axisLabel: { color: DIM, fontFamily: "Fragment Mono, ui-monospace, monospace" },
      axisLine: { lineStyle: { color: DIM } },
      splitLine: { lineStyle: { color: "rgba(215, 222, 216, 0.08)" } },
    },
    yAxis: {
      type: "value",
      min: -4.8,
      max: 4.8,
      axisLabel: { color: DIM, fontFamily: "Fragment Mono, ui-monospace, monospace" },
      splitLine: { lineStyle: { color: "rgba(215, 222, 216, 0.08)" } },
    },
    series,
  };
}
