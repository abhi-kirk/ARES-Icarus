import type { EChartsOption } from "echarts";
import { STAGE_DATA_COLORS } from "../chartOption";
import type { Sample } from "./samples";

const GRID = "#1c2a32";
const MUTED = "#7f97a3";

export type FlightSignal = "height" | "rate" | "range" | "fuel" | "thrust" | "side";

export const FLIGHT_SIGNALS: { id: FlightSignal; label: string; unit: string }[] = [
  { id: "height", label: "Altitude", unit: "m" },
  { id: "rate", label: "Rate", unit: "m/s" },
  { id: "range", label: "Range", unit: "m" },
  { id: "fuel", label: "Fuel", unit: "kg" },
  { id: "thrust", label: "Thrust", unit: "kN" },
  { id: "side", label: "Side", unit: "kN" },
];

function signalOf(id: FlightSignal): { label: string; unit: string; color: string } {
  const index = FLIGHT_SIGNALS.findIndex((item) => item.id === id);
  const spec = FLIGHT_SIGNALS[index] ?? FLIGHT_SIGNALS[0];
  return { label: spec.label, unit: spec.unit, color: STAGE_DATA_COLORS[index] ?? STAGE_DATA_COLORS[0] };
}

function sampleValue(sample: Sample, id: FlightSignal): number {
  if (id === "height") return sample.z;
  if (id === "rate") return sample.vz;
  if (id === "range") return Math.hypot(sample.x - sample.padX, sample.y - sample.padY);
  if (id === "fuel") return sample.fuel;
  if (id === "thrust") return Math.hypot(sample.fx, sample.fy, sample.fz) / 1000;
  return Math.hypot(sample.fx, sample.fy) / 1000;
}

export function flightTrace(samples: Sample[], time: number, id: FlightSignal): EChartsOption {
  const shown = samples.filter((sample) => sample.t <= time + 1e-6);
  const spec = signalOf(id);
  const data = shown.map((sample) => [sample.t, sampleValue(sample, id)]);
  const planData =
    id === "height"
      ? shown
          .filter((sample) => typeof sample.planZ === "number")
          .map((sample) => [sample.t, sample.planZ as number])
      : [];
  const label = { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 };
  const series: EChartsOption["series"] = [
    {
      name: spec.label,
      type: "line",
      showSymbol: false,
      data,
      color: spec.color,
      lineStyle: { color: spec.color, width: 2 },
      markLine: {
        silent: true,
        symbol: "none",
        lineStyle: { color: spec.color, width: 1, opacity: 0.7 },
        label: { show: false },
        data: [{ xAxis: time }],
      },
    },
  ];
  if (planData.length > 1) {
    series.push({
      name: "Plan",
      type: "line",
      showSymbol: false,
      data: planData,
      color: "#f2c14e",
      lineStyle: { color: "#f2c14e", width: 1.5, type: "dashed" },
    });
  }

  return {
    backgroundColor: "transparent",
    animation: false,
    grid: { left: 46, right: 12, top: 28, bottom: 22 },
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
      name: spec.unit,
      nameTextStyle: label,
      axisLine: { show: false },
      axisTick: { show: false },
      axisLabel: { ...label, color: spec.color },
      splitLine: { lineStyle: { color: GRID } },
    },
    series,
  };
}
