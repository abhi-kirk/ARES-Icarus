import type { EChartsOption } from "echarts";
import { STAGE_DATA_COLORS } from "../chartOption";
import type { Sample } from "./samples";

const GRID = "#1c2a32";
const MUTED = "#7f97a3";

export type FlightSignal = "height" | "rate" | "range" | "fuel" | "thrust" | "side" | "gimbal";

export const FLIGHT_SIGNALS: { id: FlightSignal; label: string; unit: string }[] = [
  { id: "height", label: "Altitude", unit: "m" },
  { id: "rate", label: "Velocity", unit: "m/s" },
  { id: "range", label: "Range", unit: "m" },
  { id: "fuel", label: "Fuel", unit: "kg" },
  { id: "thrust", label: "Thrust", unit: "kN" },
  { id: "side", label: "Side", unit: "kN" },
  { id: "gimbal", label: "Gimbal", unit: "°" },
];

export type TraceBands = {
  floorKn: number;
  ceilingKn: number;
  gimbalDeg: number;
};

const GIMBAL_COLOR = "#7dd3fc";

export function signalSpec(id: FlightSignal): { label: string; unit: string; color: string } {
  const index = FLIGHT_SIGNALS.findIndex((item) => item.id === id);
  const spec = FLIGHT_SIGNALS[index] ?? FLIGHT_SIGNALS[0];
  const color = id === "gimbal" ? GIMBAL_COLOR : (STAGE_DATA_COLORS[index] ?? STAGE_DATA_COLORS[0]);
  return { label: spec.label, unit: spec.unit, color };
}

function sampleValue(sample: Sample, id: FlightSignal): number {
  if (id === "height") return sample.z;
  if (id === "rate") return sample.vz;
  if (id === "range") return Math.hypot(sample.x - sample.padX, sample.y - sample.padY);
  if (id === "fuel") return sample.fuel;
  if (id === "thrust") return Math.hypot(sample.fx, sample.fy, sample.fz) / 1000;
  if (id === "gimbal") return Math.hypot(sample.gimbalX, sample.gimbalY) * (180 / Math.PI);
  return Math.hypot(sample.fx, sample.fy) / 1000;
}

function formatReading(id: FlightSignal, value: number): string {
  if (id === "thrust" || id === "side" || id === "gimbal") return value.toFixed(2);
  if (id === "fuel") return value.toFixed(0);
  return value.toFixed(1);
}

function readingOf(name: string, value: number): string {
  if (name === "Plan") return `${value.toFixed(1)} m`;
  const found = FLIGHT_SIGNALS.find((item) => item.label === name);
  if (!found) return value.toFixed(1);
  return `${formatReading(found.id, value)} ${found.unit}`;
}

// One plot can carry several signals when they share a unit. Altitude and range
// are both meters. A single id keeps the old one-trace plot.
export function flightTrace(
  samples: Sample[],
  time: number,
  id: FlightSignal | FlightSignal[],
  bands?: TraceBands,
  timeLabels = true,
): EChartsOption {
  const ids = Array.isArray(id) ? id : [id];
  const shown = samples.filter((sample) => sample.t <= time + 1e-6);
  const primary = signalSpec(ids[0] ?? "height");
  const label = { color: MUTED, fontFamily: "Fragment Mono, monospace", fontSize: 10 };

  let yMin: number | undefined;
  let yMax: number | undefined;
  const guides: { y: number; name: string }[] = [];
  if (ids.length === 1 && ids[0] === "thrust" && bands) {
    let peak = 0;
    for (const sample of shown) peak = Math.max(peak, sampleValue(sample, "thrust"));
    yMin = 0;
    yMax = Math.ceil(Math.max(peak, bands.ceilingKn, 1) / 5) * 5;
    if (bands.floorKn > 0.05) guides.push({ y: bands.floorKn, name: "floor" });
    if (bands.ceilingKn > bands.floorKn + 0.05) guides.push({ y: bands.ceilingKn, name: "ceiling" });
  }
  if (ids.length === 1 && ids[0] === "gimbal" && bands) {
    let peak = 0;
    for (const sample of shown) peak = Math.max(peak, sampleValue(sample, "gimbal"));
    yMin = 0;
    yMax = Math.ceil(Math.max(peak, bands.gimbalDeg, 1) / 5) * 5;
    if (bands.gimbalDeg > 0.5) guides.push({ y: bands.gimbalDeg, name: "max" });
  }

  const series: EChartsOption["series"] = ids.map((item, index) => {
    const spec = signalSpec(item);
    const data = shown.map((sample) => [sample.t, sampleValue(sample, item)]);
    const guideLines =
      index === 0
        ? guides.map((guide) => ({
            yAxis: guide.y,
            lineStyle: { color: "#c5d0d6", width: 1, type: "dashed" as const },
            label: {
              show: true,
              formatter: guide.name,
              color: MUTED,
              fontFamily: "Fragment Mono, monospace",
              fontSize: 10,
              position: "insideEndTop" as const,
            },
          }))
        : [];
    return {
      name: spec.label,
      type: "line" as const,
      showSymbol: false,
      data,
      color: spec.color,
      lineStyle: { color: spec.color, width: 2 },
      markLine:
        index === 0
          ? {
              silent: true,
              symbol: "none",
              data: [
                {
                  xAxis: time,
                  lineStyle: { color: "#d5e4ec", width: 1, opacity: 0.7 },
                  label: { show: false },
                },
                ...guideLines,
              ],
            }
          : undefined,
    };
  });

  if (ids.includes("height")) {
    const planData = shown
      .filter((sample) => typeof sample.planZ === "number")
      .map((sample) => [sample.t, sample.planZ as number]);
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
  }

  const several = series.length > 1;
  const end = samples.length > 0 ? samples[samples.length - 1].t : 1;
  const xMax = Math.max(10, Math.ceil(end / 10) * 10);

  return {
    backgroundColor: "transparent",
    animation: false,
    legend: { show: false },
    grid: { left: 48, right: 18, top: 12, bottom: timeLabels ? 22 : 8 },
    tooltip: {
      trigger: "axis",
      confine: true,
      backgroundColor: "#12181e",
      borderColor: "rgba(180, 206, 216, 0.4)",
      textStyle: { color: "#e7eef2", fontFamily: "Fragment Mono, monospace", fontSize: 11 },
      axisPointer: { type: "line", lineStyle: { color: "#d5e4ec", width: 1 } },
      formatter: (params) => {
        const rows = Array.isArray(params) ? params : [params];
        const first = rows[0];
        const raw = first && "value" in first ? first.value : undefined;
        const t = Array.isArray(raw) ? Number(raw[0]) : 0;
        const lines = [`T+ ${t.toFixed(1)} s`];
        for (const row of rows) {
          if (!row || !("value" in row)) continue;
          const value = Array.isArray(row.value) ? Number(row.value[1]) : Number(row.value);
          if (!Number.isFinite(value)) continue;
          const marker = "marker" in row && typeof row.marker === "string" ? row.marker : "";
          const name = "seriesName" in row && typeof row.seriesName === "string" ? row.seriesName : primary.label;
          lines.push(`${marker}${name}  ${readingOf(name, value)}`);
        }
        return lines.join("<br/>");
      },
    },
    xAxis: {
      type: "value",
      min: 0,
      max: xMax,
      interval: 10,
      axisLine: { lineStyle: { color: GRID } },
      axisTick: { show: false },
      axisLabel: {
        ...label,
        show: timeLabels,
        showMinLabel: false,
        formatter: (value: number) => String(Math.round(value)),
      },
      splitLine: { lineStyle: { color: GRID } },
    },
    yAxis: {
      type: "value",
      splitNumber: 4,
      axisLine: { show: false },
      axisTick: { show: false },
      axisLabel: {
        ...label,
        color: several ? MUTED : primary.color,
        formatter: (value: number) => (Math.abs(value) >= 10 || Number.isInteger(value) ? String(Math.round(value)) : value.toFixed(1)),
      },
      splitLine: { lineStyle: { color: GRID } },
      ...(yMin !== undefined && yMax !== undefined ? { min: yMin, max: yMax } : {}),
    },
    series,
  };
}
