// The case the engine already reads. Knobs edit a few fields. The rest stay
// at the nominal landing so a request cannot change the timestep or invent a new rocket.

export type Gains = { kp: number; ki: number; kd: number };

export type FlightCase = {
  inner_pid_vertical_velocity: { gains: Gains };
  outer_pid_vertical_position: { gains: Gains };
  inner_pid_horizontal_velocity: { gains: Gains };
  outer_pid_horizontal_position: { gains: Gains };
  simulation: {
    timestep_inner: number;
    timestep_outer: number;
    timestep_inner_h: number;
    timestep_outer_h: number;
    initial_x_pos: number;
    initial_y_pos: number;
    initial_z_pos: number;
    initial_x_velocity: number;
    initial_y_velocity: number;
    initial_z_velocity: number;
    initial_pad_x_pos: number;
    initial_pad_y_pos: number;
    initial_pad_x_velocity: number;
    initial_pad_y_velocity: number;
    wind_x_velocity: number;
    wind_y_velocity: number;
    gust_magnitude: number;
    gust_frequency: number;
    initial_fuel: number;
    max_ref_accel_vertical: number;
    max_ref_accel_horizontal: number;
    use_rerun: boolean;
    rerun_spawn_viewer: boolean;
    rerun_save_rrd: boolean;
  };
};

export type KnobKey = "height" | "offset" | "descent" | "fuel" | "wind" | "pad" | "gain";

export type KnobSpec = {
  key: KnobKey;
  label: string;
  min: number;
  max: number;
  step: number;
  unit: string;
};

export const KNOBS: KnobSpec[] = [
  { key: "height", label: "Altitude", min: 40, max: 600, step: 1, unit: "m" },
  { key: "offset", label: "Downrange", min: -300, max: 300, step: 1, unit: "m" },
  { key: "descent", label: "Rate", min: -60, max: 10, step: 0.5, unit: "m/s" },
  { key: "fuel", label: "Fuel", min: 50, max: 1200, step: 5, unit: "kg" },
  { key: "wind", label: "Wind", min: -15, max: 15, step: 0.1, unit: "m/s" },
  { key: "pad", label: "Barge", min: -4, max: 4, step: 0.1, unit: "m/s" },
  { key: "gain", label: "Gain", min: 0.02, max: 0.5, step: 0.01, unit: "" },
];

export const NOMINAL_CASE: FlightCase = {
  inner_pid_vertical_velocity: { gains: { kp: 0.5, ki: 0.1, kd: 0.0 } },
  outer_pid_vertical_position: { gains: { kp: 0.1, ki: 0.0, kd: 0.0 } },
  inner_pid_horizontal_velocity: { gains: { kp: 0.1, ki: 0.0, kd: 0.0 } },
  outer_pid_horizontal_position: { gains: { kp: 0.5, ki: 0.0, kd: 0.0 } },
  simulation: {
    timestep_inner: 0.02,
    timestep_outer: 0.1,
    timestep_inner_h: 0.1,
    timestep_outer_h: 0.3,
    initial_x_pos: 120,
    initial_y_pos: -80,
    initial_z_pos: 300,
    initial_x_velocity: 8,
    initial_y_velocity: -5,
    initial_z_velocity: -20,
    initial_pad_x_pos: 0,
    initial_pad_y_pos: 0,
    initial_pad_x_velocity: 0.5,
    initial_pad_y_velocity: -0.3,
    wind_x_velocity: 3.0,
    wind_y_velocity: 1.5,
    gust_magnitude: 2.0,
    gust_frequency: 0.2,
    initial_fuel: 600.0,
    max_ref_accel_vertical: 1.5,
    max_ref_accel_horizontal: 3.0,
    use_rerun: false,
    rerun_spawn_viewer: false,
    rerun_save_rrd: false,
  },
};

function clamp(value: number, min: number, max: number): number {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

function specFor(key: KnobKey): KnobSpec {
  const found = KNOBS.find((item) => item.key === key);
  if (!found) return KNOBS[0];
  return found;
}

export function readKnob(flight: FlightCase, key: KnobKey): number {
  if (key === "height") return flight.simulation.initial_z_pos;
  if (key === "offset") return flight.simulation.initial_x_pos;
  if (key === "descent") return flight.simulation.initial_z_velocity;
  if (key === "fuel") return flight.simulation.initial_fuel;
  if (key === "wind") return flight.simulation.wind_x_velocity;
  if (key === "pad") return flight.simulation.initial_pad_x_velocity;
  return flight.outer_pid_vertical_position.gains.kp;
}

function writeKnob(flight: FlightCase, key: KnobKey, value: number): void {
  if (key === "height") flight.simulation.initial_z_pos = value;
  else if (key === "offset") flight.simulation.initial_x_pos = value;
  else if (key === "descent") flight.simulation.initial_z_velocity = value;
  else if (key === "fuel") flight.simulation.initial_fuel = value;
  else if (key === "wind") flight.simulation.wind_x_velocity = value;
  else if (key === "pad") flight.simulation.initial_pad_x_velocity = value;
  else flight.outer_pid_vertical_position.gains.kp = value;
}

export function knobValue(flight: FlightCase, key: KnobKey): number {
  const spec = specFor(key);
  return clamp(readKnob(flight, key), spec.min, spec.max);
}

// Rebuild from the nominal case. Only the knob fields come from the request,
// and each one is held inside its range.
export function caseFromRequest(body: unknown): FlightCase {
  const next: FlightCase = structuredClone(NOMINAL_CASE);
  if (!body || typeof body !== "object") return next;
  const raw = body as Partial<FlightCase>;
  const sim = raw.simulation;
  const finite = (value: unknown): number | null =>
    typeof value === "number" && Number.isFinite(value) ? value : null;
  if (sim && typeof sim === "object") {
    const height = finite(sim.initial_z_pos);
    const offset = finite(sim.initial_x_pos);
    const descent = finite(sim.initial_z_velocity);
    const fuel = finite(sim.initial_fuel);
    const wind = finite(sim.wind_x_velocity);
    const pad = finite(sim.initial_pad_x_velocity);
    if (height !== null) writeKnob(next, "height", height);
    if (offset !== null) writeKnob(next, "offset", offset);
    if (descent !== null) writeKnob(next, "descent", descent);
    if (fuel !== null) writeKnob(next, "fuel", fuel);
    if (wind !== null) writeKnob(next, "wind", wind);
    if (pad !== null) writeKnob(next, "pad", pad);
  }
  const gain = finite(raw.outer_pid_vertical_position?.gains?.kp);
  if (gain !== null) writeKnob(next, "gain", gain);

  for (const spec of KNOBS) {
    writeKnob(next, spec.key, clamp(readKnob(next, spec.key), spec.min, spec.max));
  }
  return next;
}

export function withKnob(flight: FlightCase, key: KnobKey, value: number): FlightCase {
  const next: FlightCase = structuredClone(flight);
  const spec = specFor(key);
  writeKnob(next, key, clamp(value, spec.min, spec.max));
  return next;
}

export function formatKnob(key: KnobKey, value: number): string {
  const spec = specFor(key);
  const digits = spec.step < 1 ? 2 : spec.step < 5 ? 1 : 0;
  const shown = value.toFixed(digits);
  if (!spec.unit) return shown;
  return `${shown} ${spec.unit}`;
}

export function sameCase(a: FlightCase, b: FlightCase): boolean {
  return JSON.stringify(a) === JSON.stringify(b);
}
