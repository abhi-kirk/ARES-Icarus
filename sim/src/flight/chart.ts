import type { EChartsOption } from "echarts";
import { STAGE_DATA_COLORS } from "../chartOption";
import type { Sample } from "./samples";

const GOLD = STAGE_DATA_COLORS[0];
const TEAL = STAGE_DATA_COLORS[1];
const GRID = "#1c2a32";
const MUTED = "#7f97a3";

export function flightTrace(samples: Sample[], time: number, kind: "height" | "rate"): EChartsOption {
  const shown = samples.filter((sample) => sample.t <= time + 1e-6);
  const color = kind === "height" ? GOLD : TEAL;
  const data = shown.map((sample) => [sample.t, kind === "height" ? sample.z : sample.vz]);
  const title = kind === "height" ? "Altitude" : "Rate";
  const unit = kind === "height" ? "m" : "m/s";
  const label = { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 };

  return {
    backgroundColor: "transparent",
    animation: false,
    title: {
      text: title,
      left: 4,
      top: 0,
      textStyle: { color: MUTED, fontSize: 11, fontWeight: 400, fontFamily: "Fragment Mono, monospace" },
    },
    grid: { left: 44, right: 12, top: 26, bottom: 22 },
    tooltip: { show: false },
    xAxis: {
      type: "value",
      min: 0,
      max: Math.max(1, samples.length > 0 ? samples[samples.length - 1].t : 1),
      axisLine: { lineStyle: { color: GRID } },
      axisTick: { show: false },
      axisLabel: label,
      splitLine: { lineStyle: { color: GRID } },
    },
    yAxis: {
      type: "value",
      name: unit,
      nameTextStyle: label,
      axisLine: { show: false },
      axisTick: { show: false },
      axisLabel: { ...label, color },
      splitLine: { lineStyle: { color: GRID } },
    },
    series: [
      {
        name: title,
        type: "line",
        showSymbol: false,
        data,
        color,
        lineStyle: { color, width: 2 },
        markLine: {
          silent: true,
          symbol: "none",
          lineStyle: { color, width: 1, opacity: 0.7 },
          label: { show: false },
          data: [{ xAxis: time }],
        },
      },
    ],
  };
}
