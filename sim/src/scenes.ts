// Rocket Dynamics. Four pictures, one clock.
// Each scene has a claim, a knob you can drag, and the columns the chart is allowed to draw.


export type SceneId = "state" | "step" | "mass" | "engine";

export type Column =
  | "height"
  | "rate"
  | "stepped"
  | "truth"
  | "accel"
  | "mass"
  | "command"
  | "actual";

export type Row = {
  t: number;
  height: number;
  rate: number;
  stepped: number;
  truth: number;
  accel: number;
  mass: number;
  command: number;
  actual: number;
};

export type Knobs = {
  dt: number;
  thrust: number;
  tau: number;
  ask: number;
};

export const KNOBS: Knobs = {
  dt: 0.4,
  thrust: 40,
  tau: 0.6,
  ask: 90,
};

// Undamped spring. Mass 1 kg, stiffness 1 N/m, so one swing takes 2π seconds.
// Released from rest, 1 m above the rest point: height = cos(t), rate = -sin(t).
const SPRING_M = 1;
const SPRING_K = 1;

const G = 9.81;
const DRY = 1.5;
const FUEL = 1.5;
const BURN = 4;
const LIMIT = 50;
const ASK_AT = 0.4;

export type KnobKey = "dt" | "thrust" | "tau" | "ask";

const LIMITS: Record<KnobKey, { min: number; max: number }> = {
  dt: { min: 0.05, max: 1 },
  thrust: { min: 20, max: 80 },
  tau: { min: 0.08, max: 2 },
  ask: { min: 0, max: 120 },
};

export function clampNamed(key: KnobKey, value: number): number {
  const limit = LIMITS[key];
  return clamp(value, limit.min, limit.max);
}

export const SCENES: {
  id: SceneId;
  name: string;
  title: string;
  lede: string;
  opener: string;
  live: Column[];
  chartTitle: string;
}[] = [
  {
    id: "state",
    name: "The spring",
    title: "The spring",
    lede: "A mass hangs from a spring with the damper open. Its height from the rest point is the state, and the rate is how fast that height is changing.",
    opener: "Height from the rest point is the state. The rate is how fast that height changes. Ask me about a symbol, or tell me a time.",
    live: ["height", "rate"],
    chartTitle: "Height and rate",
  },
  {
    id: "step",
    name: "The step",
    title: "The step",
    lede: "The spring force sets the rate, and each step adds that rate times the time step, then holds the new height. The wire box is the true height and the gold mass is the step, so dragging the gold mass changes the step size.",
    opener: "The gold mass is one step at a time. The wire box is the true height. Ask me what a symbol means, or set a step.",
    live: ["stepped", "truth"],
    chartTitle: "Step and truth",
  },
  {
    id: "mass",
    name: "The burn",
    title: "The burn",
    lede: "Thrust stays on while the tank empties, and at four seconds the tank is empty so the thrust stops. Height and rate come from integrating acceleration. Drag the rocket to change the thrust input.",
    opener: "Height and rate are the state. Thrust is an input until the tank is empty. Ask me what a symbol means, or set a thrust.",
    live: ["height", "rate"],
    chartTitle: "Height and rate",
  },
  {
    id: "engine",
    name: "The engine",
    title: "The engine",
    lede: "The command jumps, and the pale line is what you asked for while the gold line is what the engine does. Height and rate follow from that thrust. Drag the rocket to change the lag, and the slider sets the command input.",
    opener: "The command is an input. The gold line is thrust the engine makes. Height and rate are the state. Ask me what a symbol means, or set a lag.",
    live: ["actual", "command"],
    chartTitle: "Command and actual",
  },
];

export function sceneById(id: SceneId) {
  const found = SCENES.find((scene) => scene.id === id);
  if (!found) return SCENES[0];
  return found;
}

export function durationOf(id: SceneId): number {
  if (id === "state" || id === "step") return 2 * Math.PI;
  if (id === "mass") return 8;
  return 5;
}

export function clampKnob(id: SceneId, value: number): number {
  if (id === "step") return clampNamed("dt", value);
  if (id === "mass") return clampNamed("thrust", value);
  if (id === "engine") return clampNamed("tau", value);
  return value;
}

function clamp(value: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, value));
}

function blank(t: number): Row {
  return {
    t,
    height: 0,
    rate: 0,
    stepped: 0,
    truth: 0,
    accel: 0,
    mass: 0,
    command: 0,
    actual: 0,
  };
}

export function springHeight(t: number): number {
  return Math.cos(t);
}

export function springRate(t: number): number {
  return -Math.sin(t);
}

function springRows(): Row[] {
  const rows: Row[] = [];
  const end = durationOf("state");
  for (let t = 0; t <= end + 1e-9; t += 0.05) {
    const row = blank(t);
    row.height = springHeight(t);
    row.rate = springRate(t);
    row.truth = row.height;
    rows.push(row);
  }
  return rows;
}

// Explicit Euler on the spring, held between steps. Both updates use the old state.
function eulerStep(height: number, rate: number, step: number): { height: number; rate: number } {
  const accel = -SPRING_K * height;
  return {
    height: height + step * rate,
    rate: rate + step * accel,
  };
}

function eulerHold(t: number, dt: number): { height: number; rate: number } {
  let height = 1;
  let rate = 0;
  let tau = 0;
  const step = clampKnob("step", dt);
  while (tau + step <= t + 1e-9) {
    const next = eulerStep(height, rate, step);
    height = next.height;
    rate = next.rate;
    tau += step;
  }
  return { height, rate };
}

export function steppedHeight(t: number, dt: number): number {
  return eulerHold(t, dt).height;
}

export function steppedRate(t: number, dt: number): number {
  return eulerHold(t, dt).rate;
}

function stairPoints(dt: number, until: number): [number, number][] {
  const step = clampKnob("step", dt);
  const end = Math.min(until, durationOf("step"));
  const points: [number, number][] = [[0, 1]];
  let height = 1;
  let rate = 0;
  let tau = 0;
  while (tau + step <= end + 1e-9) {
    const nextTime = tau + step;
    points.push([nextTime, height]);
    const next = eulerStep(height, rate, step);
    points.push([nextTime, next.height]);
    height = next.height;
    rate = next.rate;
    tau = nextTime;
  }
  if (end > tau + 1e-6) points.push([end, height]);
  return points;
}

function massRows(thrust: number): Row[] {
  const rows: Row[] = [];
  const force = clampKnob("mass", thrust);
  let velocity = 0;
  let height = 0;
  const step = 0.02;
  for (let t = 0; t <= durationOf("mass") + 1e-9; t += step) {
    const fuelLeft = Math.max(0, FUEL * (1 - t / BURN));
    const mass = DRY + fuelLeft;
    const pushing = t < BURN ? force : 0;
    const accel = pushing / mass - G;
    const row = blank(t);
    row.height = height;
    row.rate = velocity;
    row.accel = accel;
    row.mass = mass;
    row.actual = pushing;
    rows.push(row);
    velocity += accel * step;
    height += velocity * step;
  }
  return rows;
}

function engineRows(tau: number, ask: number): Row[] {
  const rows: Row[] = [];
  const lag = clampKnob("engine", tau);
  const asked = clampNamed("ask", ask);
  const mass = 2.5;
  let thrust = 0;
  let velocity = 0;
  let height = 0;
  const step = 0.02;
  for (let t = 0; t <= durationOf("engine") + 1e-9; t += step) {
    const command = t < ASK_AT ? 0 : asked;
    thrust += ((command - thrust) / lag) * step;
    if (thrust > LIMIT) thrust = LIMIT;
    if (thrust < 0) thrust = 0;
    const accel = thrust / mass - G;
    const row = blank(t);
    row.height = height;
    row.rate = velocity;
    row.accel = accel;
    row.command = command;
    row.actual = thrust;
    rows.push(row);
    velocity += accel * step;
    height += velocity * step;
  }
  return rows;
}

let rowCache: { key: string; rows: Row[] } | null = null;

export function rowsFor(id: SceneId, knobs: Knobs): Row[] {
  const key = `${id}|${knobs.dt}|${knobs.thrust}|${knobs.tau}|${knobs.ask}`;
  if (rowCache && rowCache.key === key) return rowCache.rows;

  let rows: Row[];
  if (id === "state" || id === "step") rows = springRows();
  else if (id === "mass") rows = massRows(knobs.thrust);
  else rows = engineRows(knobs.tau, knobs.ask);

  rowCache = { key, rows };
  return rows;
}

export function rowAt(rows: Row[], t: number): Row {
  let chosen = rows[0];
  for (const row of rows) {
    if (row.t <= t + 1e-9) chosen = row;
    else break;
  }
  return chosen;
}

export function bodyAt(id: SceneId, t: number, knobs: Knobs): { height: number; truth: number; flame: number } {
  if (id === "state") {
    const height = springHeight(t);
    return { height, truth: height, flame: 0 };
  }
  if (id === "step") {
    return { height: steppedHeight(t, knobs.dt), truth: springHeight(t), flame: 0 };
  }
  const row = rowAt(rowsFor(id, knobs), t);
  if (id === "mass") {
    const burning = t < BURN && knobs.thrust > 0;
    return { height: row.height, truth: row.height, flame: burning ? knobs.thrust / LIMITS.thrust.max : 0 };
  }
  return { height: row.height, truth: row.height, flame: row.actual / LIMIT };
}

export type PlotSeries = {
  name: string;
  column: Column;
  gold: boolean;
  points: [number, number][];
  axis: 0 | 1;
};

const COLUMNS: Record<Column, { name: string; label: string; words: string[] }> = {
  height: { name: "Height", label: "Height (m)", words: ["height", "state", "position"] },
  rate: { name: "Rate", label: "Rate (m/s)", words: ["rate", "slope", "speed"] },
  stepped: { name: "Step", label: "Step (m)", words: ["step", "stair", "euler"] },
  truth: { name: "True height", label: "True height (m)", words: ["true", "truth", "exact"] },
  accel: { name: "Acceleration", label: "Acceleration (m/s²)", words: ["accel", "acceleration"] },
  mass: { name: "Mass", label: "Mass (kg)", words: ["mass", "weight", "tank"] },
  command: { name: "Command", label: "Command (N)", words: ["command", "ask"] },
  actual: { name: "Actual", label: "Actual (N)", words: ["actual", "thrust", "engine"] },
};

export function columnName(column: Column): string {
  return COLUMNS[column].name;
}

export function columnWords(column: Column): string[] {
  return COLUMNS[column].words;
}

export function seriesPoints(id: SceneId, column: Column, knobs: Knobs, until: number): [number, number][] {
  if (id === "step" && column === "stepped") return stairPoints(knobs.dt, until);
  const rows = rowsFor(id, knobs).filter((row) => row.t <= until + 1e-9);
  const points = rows.map((row) => [row.t, row[column]] as [number, number]);
  const wantsTip = id === "state" || (id === "step" && column === "truth");
  if (wantsTip) {
    const tip = points[points.length - 1];
    if (!tip || Math.abs(tip[0] - until) > 1e-4) {
      const value = column === "rate" ? springRate(until) : springHeight(until);
      points.push([until, value]);
    }
  }
  return points;
}

export function plotOf(id: SceneId, columns: Column[], knobs: Knobs, until: number): PlotSeries[] {
  return columns.map((column) => ({
    name: COLUMNS[column].label,
    column,
    gold: column === "rate" || column === "stepped" || column === "accel" || column === "actual",
    points: seriesPoints(id, column, knobs, until),
    axis: column === "mass" ? 1 : 0,
  }));
}

export function axisOf(id: SceneId, columns: Column[], knobs: Knobs): { y: [number, number]; y2?: [number, number] } {
  if (id === "state") return { y: [-1.25, 1.25] };
  const full = plotOf(id, columns, knobs, durationOf(id));
  const primary = full.filter((series) => series.axis === 0).flatMap((series) => series.points.map((point) => point[1]));
  const secondary = full.filter((series) => series.axis === 1).flatMap((series) => series.points.map((point) => point[1]));
  return {
    y: pad(primary, true),
    y2: secondary.length ? pad(secondary, false) : undefined,
  };
}

function pad(values: number[], includeZero: boolean): [number, number] {
  let lo = Math.min(...values);
  let hi = Math.max(...values);
  if (includeZero) {
    lo = Math.min(lo, 0);
    hi = Math.max(hi, 0);
  }
  const span = hi - lo || 1;
  lo -= span * 0.08;
  hi += span * 0.08;
  return nice(lo, hi);
}

function nice(lo: number, hi: number): [number, number] {
  const span = hi - lo || 1;
  const raw = span / 4;
  const exp = Math.pow(10, Math.floor(Math.log10(raw)));
  const base = raw / exp;
  let step = exp;
  if (base > 5) step = 10 * exp;
  else if (base > 2) step = 5 * exp;
  else if (base > 1) step = 2 * exp;
  const start = Math.floor(lo / step) * step;
  const end = Math.ceil(hi / step) * step;
  return [Math.round(start * 1000) / 1000, Math.round(end * 1000) / 1000];
}

export function readout(id: SceneId, t: number, knobs: Knobs): { text: string; gold: boolean }[] {
  if (id === "state") {
    return [
      { text: `${t.toFixed(2)} s`, gold: false },
      { text: `${springHeight(t).toFixed(2)} m`, gold: false },
      { text: `${springRate(t).toFixed(2)} m/s`, gold: true },
    ];
  }
  if (id === "step") {
    return [
      { text: `${t.toFixed(2)} s`, gold: false },
      { text: `${steppedHeight(t, knobs.dt).toFixed(2)} m`, gold: true },
      { text: `${springHeight(t).toFixed(2)} true`, gold: false },
    ];
  }
  const row = rowAt(rowsFor(id, knobs), t);
  if (id === "mass") {
    return [
      { text: `${t.toFixed(2)} s`, gold: false },
      { text: `${row.height.toFixed(1)} m`, gold: false },
      { text: `${row.rate.toFixed(1)} m/s`, gold: true },
    ];
  }
  return [
    { text: `${t.toFixed(2)} s`, gold: false },
    { text: `${row.height.toFixed(1)} m`, gold: false },
    { text: `${row.rate.toFixed(1)} m/s`, gold: true },
    { text: `${row.actual.toFixed(0)} N`, gold: false },
  ];
}

export function knobOf(
  id: SceneId,
  knobs: Knobs,
): { key: KnobKey; label: string; value: number; min: number; max: number; step: number; show: string; aside?: string } | null {
  if (id === "step") {
    return { key: "dt", label: "Step", value: knobs.dt, min: LIMITS.dt.min, max: LIMITS.dt.max, step: 0.05, show: `${knobs.dt.toFixed(2)} s` };
  }
  if (id === "mass") {
    return { key: "thrust", label: "Thrust", value: knobs.thrust, min: LIMITS.thrust.min, max: LIMITS.thrust.max, step: 1, show: `${knobs.thrust.toFixed(0)} N` };
  }
  if (id === "engine") {
    return {
      key: "ask",
      label: "Ask",
      value: knobs.ask,
      min: LIMITS.ask.min,
      max: LIMITS.ask.max,
      step: 1,
      show: `${knobs.ask.toFixed(0)} N`,
      aside: `lag ${knobs.tau.toFixed(2)} s`,
    };
  }
  return null;
}

export function viewSpan(id: SceneId, knobs: Knobs): { min: number; max: number } {
  if (id === "state") return { min: -1, max: 1 };
  if (id === "step") {
    let min = -1;
    let max = 1;
    for (const point of stairPoints(knobs.dt, durationOf("step"))) {
      min = Math.min(min, point[1]);
      max = Math.max(max, point[1]);
    }
    return { min, max };
  }
  const rows = rowsFor(id, knobs);
  let min = 0;
  let max = 0;
  for (const row of rows) {
    min = Math.min(min, row.height);
    max = Math.max(max, row.height);
  }
  if (max - min < 4) max = min + 4;
  return { min, max };
}

export function chatBrief(id: SceneId, knobs: Knobs): string {
  const scene = sceneById(id);
  if (id === "state") return `${scene.title}. Undamped spring, mass ${SPRING_M} kg, stiffness ${SPRING_K} N/m. Height from rest is cos(t) meters. Rate is -sin(t) meters per second. Allowed chart columns: height, rate.`;
  if (id === "step") return `${scene.title}. True height is cos(t). Stepped height is explicit Euler of the spring force with dt = ${knobs.dt.toFixed(2)} s. Allowed chart columns: stepped, truth.`;
  if (id === "mass") {
    return `${scene.title}. Thrust input is ${knobs.thrust.toFixed(0)} N until t = ${BURN} s, then zero. Dry mass ${DRY} kg, fuel ${FUEL} kg. Height and rate integrate acceleration. Allowed chart columns: height, rate, accel, mass.`;
  }
  return `${scene.title}. Command input steps to ${knobs.ask.toFixed(0)} N at ${ASK_AT} s. Lag is ${knobs.tau.toFixed(2)} s. Thrust is clamped to ${LIMIT} N. Height and rate integrate thrust. Allowed chart columns: height, rate, accel, command, actual.`;
}
