// Rocket Dynamics. One clock per scene. Each scene has a claim, a knob you can drag, and chart columns.

import { loopRowAt, loopRowsFor, LOOP_LIMIT, LOOP_M, LOOP_T0 } from "./loopPlant";


export type SceneId =
  | "state"
  | "step"
  | "mass"
  | "engine"
  | "loopOpen"
  | "loopError"
  | "loopEquil"
  | "loopHover"
  | "loopMode"
  | "loopSample";

export type Column =
  | "height"
  | "rate"
  | "stepped"
  | "truth"
  | "accel"
  | "mass"
  | "command"
  | "actual"
  | "reference"
  | "pHeight"
  | "iHeight"
  | "dHeight"
  | "nonlinear"
  | "linear"
  | "openHeight"
  | "closedHeight"
  | "measured"
  | "hiddenMode"
  | "continuous"
  | "discrete"
  | "thrust";

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
  reference: number;
  pHeight: number;
  iHeight: number;
  dHeight: number;
  nonlinear: number;
  linear: number;
  openHeight: number;
  closedHeight: number;
  measured: number;
  hiddenMode: number;
  continuous: number;
  discrete: number;
  thrust: number;
};

export type Knobs = {
  dt: number;
  thrust: number;
  tau: number;
  ask: number;
  kp: number;
  ki: number;
  kd: number;
  perturb: number;
  unstable: number;
  coupling: number;
  extraThrust: number;
};

export const KNOBS: Knobs = {
  dt: 0.4,
  thrust: 40,
  tau: 0.35,
  ask: 90,
  extraThrust: 15,
  kp: 12,
  ki: 8,
  kd: 15,
  perturb: 0.35,
  unstable: 4,
  coupling: 0.35,
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

export type KnobKey =
  | "dt"
  | "thrust"
  | "tau"
  | "ask"
  | "kp"
  | "ki"
  | "kd"
  | "perturb"
  | "unstable"
  | "coupling"
  | "extraThrust";

const LIMITS: Record<KnobKey, { min: number; max: number }> = {
  dt: { min: 0.05, max: 0.5 },
  thrust: { min: 20, max: 80 },
  tau: { min: 0.05, max: 0.8 },
  ask: { min: 0, max: 120 },
  extraThrust: { min: 0, max: 35 },
  kp: { min: 4, max: 24 },
  ki: { min: 2, max: 16 },
  kd: { min: 4, max: 28 },
  perturb: { min: 0.08, max: 0.8 },
  unstable: { min: 2, max: 7 },
  coupling: { min: 0.1, max: 0.7 },
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
  askable: Column[];
  chartTitle: string;
  visual: string;
}[] = [
  {
    id: "state",
    name: "The spring",
    title: "The spring",
    lede: "A mass hangs from a spring with the damper open. Its height from the rest point is the state, and the rate is how fast that height is changing.",
    opener: "Height from the rest point is the state. The rate is how fast that height changes. Ask me about a symbol, or tell me a time.",
    live: ["height", "rate"],
    askable: ["height", "rate"],
    chartTitle: "Height and rate",
    visual: "The mass shows height from the rest point. The live chart shows height and rate against time, and the phase chart plots rate against height.",
  },
  {
    id: "step",
    name: "The step",
    title: "The step",
    lede: "The spring force sets the rate, and each step adds that rate times the time step, then holds the new height. The wire box is the true height and the gold mass is the step, so dragging the gold mass changes the step size.",
    opener: "The gold mass is one step at a time. The wire box is the true height. Ask me what a symbol means, or set a step.",
    live: ["stepped", "truth"],
    askable: ["stepped", "truth", "height", "rate"],
    chartTitle: "Step and truth",
    visual: "The gold mass is the explicit Euler step. The wire box is the true spring height, so their gap is the numerical error.",
  },
  {
    id: "mass",
    name: "The burn",
    title: "The burn",
    lede: "Thrust stays on while the tank empties, and at four seconds the tank is empty so the thrust stops. Height and rate come from integrating acceleration. Drag the rocket to change the thrust input.",
    opener: "Height and rate are the state. Thrust is an input until the tank is empty. Ask me what a symbol means, or set a thrust.",
    live: ["height", "rate"],
    askable: ["height", "rate", "accel", "mass", "actual"],
    chartTitle: "Height and rate",
    visual: "The rocket rises or falls from the integrated motion. The chart can also show acceleration, mass, and thrust to connect the tank to the trajectory.",
  },
  {
    id: "engine",
    name: "The engine",
    title: "The engine",
    lede: "The command jumps, and the pale line is what you asked for while the gold line is what the engine does. Height and rate follow from that thrust. Drag the rocket to change the lag, and the slider sets the command input.",
    opener: "The command is an input. The gold line is thrust the engine makes. Height and rate are the state. Ask me what a symbol means, or set a lag.",
    live: ["actual", "command"],
    askable: ["actual", "command", "height", "rate", "accel"],
    chartTitle: "Command and actual",
    visual: "The pale command is what you ask from the engine. The colored actual thrust lags behind it, and motion follows from the thrust that actually exists.",
  },
  {
    id: "loopOpen",
    name: "Open loop",
    title: "Open loop",
    lede: "The planner picks a thrust schedule and the rocket follows it. Nothing measures height and pushes back on the command, so a fixed extra thrust after one second lets the rocket drift above the target.",
    opener: "Reference height is what you want. Height is what happens with no feedback. Drag the rocket to change the extra thrust.",
    live: ["reference", "height"],
    askable: ["reference", "height", "rate", "thrust"],
    chartTitle: "Reference and height",
    visual: "The reference is the desired height. The measured height does not feed back into the thrust, so a small extra push keeps accumulating motion.",
  },
  {
    id: "loopError",
    name: "Three reactions to an error",
    title: "Three reactions to an error",
    lede: "The target height steps up at two seconds. Three controllers each use one idea: proportional to error, integral of error, or derivative through the rate. Compare how each shape responds.",
    opener: "The gold lines are the three responses. Drag the rocket to change the proportional gain used in the P run.",
    live: ["pHeight", "iHeight", "dHeight"],
    askable: ["reference", "pHeight", "iHeight", "dHeight", "height", "rate", "thrust"],
    chartTitle: "P, I, and D",
    visual: "The reference steps up. The three height traces show how proportional, integral, and derivative terms react to the same error.",
  },
  {
    id: "loopEquil",
    name: "Equilibrium and linearization",
    title: "Equilibrium and linearization",
    lede: "Near hover, the same closed-loop law is shown twice: the full thrust-height relation and the linearized one. For a small starting offset they stay on top of each other.",
    opener: "Nonlinear is the real plant. Linear is the small-signal model. Drag the rocket to change the starting offset.",
    live: ["nonlinear", "linear"],
    askable: ["nonlinear", "linear", "height", "rate", "thrust"],
    chartTitle: "Nonlinear and linear",
    visual: "The nonlinear trace uses the full thrust law. The linear trace uses the local small-signal model near hover.",
  },
  {
    id: "loopHover",
    name: "Hover that is unstable and still lands",
    title: "Hover that is unstable and still lands",
    lede: "Open loop, a wrong-sign term makes height run away from equilibrium. With feedback on height, the same plant is driven to a landing profile and settles on the pad.",
    opener: "Open loop diverges. Closed loop lands. Drag the rocket to change how unstable the open loop is.",
    live: ["openHeight", "closedHeight"],
    askable: ["reference", "openHeight", "closedHeight", "height", "rate", "thrust"],
    chartTitle: "Open loop and closed loop",
    visual: "The open-loop trace runs away from hover. The closed-loop trace follows a landing reference and settles on the pad.",
  },
  {
    id: "loopMode",
    name: "A mode you cannot steer or see",
    title: "A mode you cannot steer or see",
    lede: "Height is what you measure. A hidden flexible mode grows in the background while feedback only acts on height. The chart shows both so you can see what the sensor misses.",
    opener: "Measured height is what the loop sees. Hidden mode is internal. Drag the rocket to change how strongly the mode pushes on motion.",
    live: ["measured", "hiddenMode"],
    askable: ["reference", "measured", "hiddenMode", "height", "rate", "thrust"],
    chartTitle: "Measured and hidden mode",
    visual: "Measured height is what the controller sees. The hidden mode is internal motion that can grow while the sensor looks calm.",
  },
  {
    id: "loopSample",
    name: "Sample time and delay",
    title: "Sample time and delay",
    lede: "Continuous feedback updates every instant. Discrete feedback holds thrust between samples, and delay uses an old height reading. The same gains feel different on the chart.",
    opener: "Continuous is the ideal loop. Discrete uses sample time and delay. Drag the rocket to change the lag on the delayed height.",
    live: ["continuous", "discrete"],
    askable: ["reference", "continuous", "discrete", "height", "rate", "command", "actual"],
    chartTitle: "Continuous and discrete",
    visual: "The continuous trace updates every instant. The discrete trace holds thrust between samples and may act on delayed height.",
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
  if (id === "engine") return 5;
  if (id === "loopOpen") return 10;
  if (id === "loopError") return 14;
  if (id === "loopEquil") return 10;
  if (id === "loopHover") return 14;
  if (id === "loopMode") return 12;
  if (id === "loopSample") return 12;
  return 5;
}

export function isLoopScene(id: SceneId): boolean {
  return id.startsWith("loop");
}

export function clampKnob(id: SceneId, value: number): number {
  if (id === "step") return clampNamed("dt", value);
  if (id === "mass") return clampNamed("thrust", value);
  if (id === "engine") return clampNamed("tau", value);
  if (id === "loopSample") return clampNamed("tau", value);
  if (id === "loopOpen") return clampNamed("extraThrust", value);
  if (id === "loopError") return clampNamed("kp", value);
  if (id === "loopEquil") return clampNamed("perturb", value);
  if (id === "loopHover") return clampNamed("unstable", value);
  if (id === "loopMode") return clampNamed("coupling", value);
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
    reference: 0,
    pHeight: 0,
    iHeight: 0,
    dHeight: 0,
    nonlinear: 0,
    linear: 0,
    openHeight: 0,
    closedHeight: 0,
    measured: 0,
    hiddenMode: 0,
    continuous: 0,
    discrete: 0,
    thrust: 0,
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

function fromLoopRow(row: ReturnType<typeof loopRowAt>): Row {
  return {
    t: row.t,
    height: row.height,
    rate: row.rate,
    stepped: 0,
    truth: 0,
    accel: 0,
    mass: 0,
    command: row.command,
    actual: row.actual,
    reference: row.reference,
    pHeight: row.pHeight,
    iHeight: row.iHeight,
    dHeight: row.dHeight,
    nonlinear: row.nonlinear,
    linear: row.linear,
    openHeight: row.openHeight,
    closedHeight: row.closedHeight,
    measured: row.measured,
    hiddenMode: row.hiddenMode,
    continuous: row.continuous,
    discrete: row.discrete,
    thrust: row.thrust,
  };
}

export function rowsFor(id: SceneId, knobs: Knobs): Row[] {
  const key = `${id}|${knobs.dt}|${knobs.thrust}|${knobs.tau}|${knobs.ask}|${knobs.kp}|${knobs.ki}|${knobs.kd}|${knobs.perturb}|${knobs.unstable}|${knobs.coupling}|${knobs.extraThrust}`;
  if (rowCache && rowCache.key === key) return rowCache.rows;

  let rows: Row[];
  if (id === "state" || id === "step") rows = springRows();
  else if (id === "mass") rows = massRows(knobs.thrust);
  else if (id === "engine") rows = engineRows(knobs.tau, knobs.ask);
  else rows = loopRowsFor(id, knobs).map(fromLoopRow);

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
  if (isLoopScene(id)) {
    const thrust = row.actual || row.thrust || LOOP_T0;
    return { height: row.height, truth: row.height, flame: Math.min(1, Math.max(0, (thrust - LOOP_T0 * 0.85) / (LOOP_LIMIT - LOOP_T0 * 0.85))) };
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

export type ColumnUnit = "m" | "m/s" | "m/s2" | "kg" | "N" | "unitless";

export const COLUMNS: Record<Column, { name: string; label: string; unit: ColumnUnit; words: string[] }> = {
  height: { name: "Height", label: "Height (m)", unit: "m", words: ["height", "state", "position"] },
  rate: { name: "Rate", label: "Rate (m/s)", unit: "m/s", words: ["rate", "slope", "speed"] },
  stepped: { name: "Step", label: "Step (m)", unit: "m", words: ["step", "stair", "euler"] },
  truth: { name: "True height", label: "True height (m)", unit: "m", words: ["true", "truth", "exact"] },
  accel: { name: "Acceleration", label: "Acceleration (m/s²)", unit: "m/s2", words: ["accel", "acceleration"] },
  mass: { name: "Mass", label: "Mass (kg)", unit: "kg", words: ["mass", "weight", "tank"] },
  command: { name: "Command", label: "Command (N)", unit: "N", words: ["command", "ask"] },
  actual: { name: "Actual", label: "Actual (N)", unit: "N", words: ["actual", "thrust", "engine"] },
  reference: { name: "Reference", label: "Reference (m)", unit: "m", words: ["reference", "target", "setpoint"] },
  pHeight: { name: "P response", label: "P (m)", unit: "m", words: ["proportional", "p"] },
  iHeight: { name: "I response", label: "I (m)", unit: "m", words: ["integral", "i"] },
  dHeight: { name: "D response", label: "D (m)", unit: "m", words: ["derivative", "d"] },
  nonlinear: { name: "Nonlinear", label: "Nonlinear (m)", unit: "m", words: ["nonlinear", "full"] },
  linear: { name: "Linear", label: "Linear (m)", unit: "m", words: ["linear", "linearized"] },
  openHeight: { name: "Open loop", label: "Open loop (m)", unit: "m", words: ["open", "unstable"] },
  closedHeight: { name: "Closed loop", label: "Closed loop (m)", unit: "m", words: ["closed", "feedback"] },
  measured: { name: "Measured", label: "Measured (m)", unit: "m", words: ["measured", "sensor"] },
  hiddenMode: { name: "Hidden mode", label: "Hidden mode", unit: "unitless", words: ["hidden", "mode", "internal"] },
  continuous: { name: "Continuous", label: "Continuous (m)", unit: "m", words: ["continuous", "analog"] },
  discrete: { name: "Discrete", label: "Discrete (m)", unit: "m", words: ["discrete", "sample", "digital"] },
  thrust: { name: "Thrust", label: "Thrust (N)", unit: "N", words: ["thrust", "force"] },
};

export function columnName(column: Column): string {
  return COLUMNS[column].name;
}

export function columnLabel(column: Column): string {
  return COLUMNS[column].label;
}

export function columnUnit(column: Column): ColumnUnit {
  return COLUMNS[column].unit;
}

export function columnWords(column: Column): string[] {
  return COLUMNS[column].words;
}

export function chartColumns(id: SceneId): Column[] {
  return sceneById(id).askable;
}

export function visualContext(id: SceneId): string {
  return sceneById(id).visual;
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
  const primaryUnit = COLUMNS[columns[0] ?? "height"].unit;
  return columns.map((column) => ({
    name: COLUMNS[column].label,
    column,
    gold:
      column === "rate" ||
      column === "stepped" ||
      column === "accel" ||
      column === "actual" ||
      column === "height" ||
      column === "pHeight" ||
      column === "nonlinear" ||
      column === "openHeight" ||
      column === "measured" ||
      column === "discrete",
    points: seriesPoints(id, column, knobs, until),
    axis: COLUMNS[column].unit === primaryUnit ? 0 : 1,
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
  if (isLoopScene(id)) {
    return [
      { text: `${t.toFixed(2)} s`, gold: false },
      { text: `${row.height.toFixed(2)} m`, gold: false },
      { text: `${row.rate.toFixed(2)} m/s`, gold: true },
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
  if (id === "loopOpen") {
    return {
      key: "extraThrust",
      label: "Extra",
      value: knobs.extraThrust,
      min: LIMITS.extraThrust.min,
      max: LIMITS.extraThrust.max,
      step: 1,
      show: `${knobs.extraThrust.toFixed(0)} N`,
    };
  }
  if (id === "loopError") {
    return {
      key: "kp",
      label: "Gain",
      value: knobs.kp,
      min: LIMITS.kp.min,
      max: LIMITS.kp.max,
      step: 1,
      show: `Kp ${knobs.kp.toFixed(0)}`,
      aside: `Ki ${knobs.ki.toFixed(0)} Kd ${knobs.kd.toFixed(0)}`,
    };
  }
  if (id === "loopEquil") {
    return {
      key: "perturb",
      label: "Offset",
      value: knobs.perturb,
      min: LIMITS.perturb.min,
      max: LIMITS.perturb.max,
      step: 0.02,
      show: `${knobs.perturb.toFixed(2)} m`,
    };
  }
  if (id === "loopHover") {
    return {
      key: "unstable",
      label: "Open μ",
      value: knobs.unstable,
      min: LIMITS.unstable.min,
      max: LIMITS.unstable.max,
      step: 0.1,
      show: `${knobs.unstable.toFixed(1)}`,
    };
  }
  if (id === "loopMode") {
    return {
      key: "coupling",
      label: "Coupling",
      value: knobs.coupling,
      min: LIMITS.coupling.min,
      max: LIMITS.coupling.max,
      step: 0.02,
      show: `${knobs.coupling.toFixed(2)}`,
    };
  }
  if (id === "loopSample") {
    return {
      key: "dt",
      label: "Sample",
      value: knobs.dt,
      min: LIMITS.dt.min,
      max: LIMITS.dt.max,
      step: 0.01,
      show: `${knobs.dt.toFixed(2)} s`,
      aside: `delay ${knobs.tau.toFixed(2)} s`,
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

export type ChartMarker = { t: number; label: string };

export function chartMarkers(id: SceneId, knobs: Knobs): ChartMarker[] {
  if (id === "mass") return [{ t: BURN, label: "tank empty" }];
  if (id === "engine") return [{ t: ASK_AT, label: "command" }];
  if (id === "loopOpen") return [{ t: 1, label: "extra thrust" }];
  if (id === "loopError") return [{ t: 2, label: "target step" }];
  if (id === "loopMode") return [{ t: 1.5, label: "target step" }];
  if (id === "loopSample") {
    const markers: ChartMarker[] = [{ t: 1, label: "target step" }];
    for (let t = knobs.dt; t < durationOf(id) - 1e-9; t += knobs.dt) {
      markers.push({ t, label: "sample" });
      if (markers.length >= 9) break;
    }
    return markers;
  }
  return [];
}

function allowedList(id: SceneId): string {
  return chartColumns(id).join(", ");
}

export function chatBrief(id: SceneId, knobs: Knobs): string {
  const scene = sceneById(id);
  if (id === "state") return `${scene.title}. Undamped spring, mass ${SPRING_M} kg, stiffness ${SPRING_K} N/m. Height from rest is cos(t) meters. Rate is -sin(t) meters per second. Allowed chart columns: ${allowedList(id)}.`;
  if (id === "step") return `${scene.title}. True height is cos(t). Stepped height is explicit Euler of the spring force with dt = ${knobs.dt.toFixed(2)} s. Allowed chart columns: ${allowedList(id)}.`;
  if (id === "mass") {
    return `${scene.title}. Thrust input is ${knobs.thrust.toFixed(0)} N until t = ${BURN} s, then zero. Dry mass ${DRY} kg, fuel ${FUEL} kg. Height and rate integrate acceleration. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "engine") {
    return `${scene.title}. Command input steps to ${knobs.ask.toFixed(0)} N at ${ASK_AT} s. Lag is ${knobs.tau.toFixed(2)} s. Thrust is clamped to ${LIMIT} N. Height and rate integrate thrust. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "loopOpen") {
    return `${scene.title}. Hover thrust is ${LOOP_T0.toFixed(1)} N on a ${LOOP_M} kg rocket. Extra thrust ${knobs.extraThrust.toFixed(0)} N after 1 s with no feedback. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "loopError") {
    return `${scene.title}. Target steps to 2 m at 2 s. Gains Kp=${knobs.kp}, Ki=${knobs.ki}, Kd=${knobs.kd} for the three single-mode runs. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "loopEquil") {
    return `${scene.title}. Start offset ${knobs.perturb.toFixed(2)} m with closed-loop hover law. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "loopHover") {
    return `${scene.title}. Open-loop instability gain ${knobs.unstable.toFixed(1)} with closed-loop landing on the same model. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "loopMode") {
    return `${scene.title}. Coupling ${knobs.coupling.toFixed(2)} from hidden mode into height. Allowed chart columns: ${allowedList(id)}.`;
  }
  return `${scene.title}. Sample time ${knobs.dt.toFixed(2)} s and measurement delay ${knobs.tau.toFixed(2)} s versus continuous control. Allowed chart columns: ${allowedList(id)}.`;
}
