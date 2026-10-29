// Same case the page is allowed to send. Knob fields are copied and clamped.
// Timestep, gust, and the other gains stay at the nominal landing.
// Keep the ranges in step with sim/src/flight/case.ts.

const KNOBS = {
  height: { min: 40, max: 600 },
  offset: { min: -300, max: 300 },
  descent: { min: -60, max: 10 },
  fuel: { min: 50, max: 1200 },
  wind: { min: -15, max: 15 },
  pad: { min: -4, max: 4 },
  gain: { min: 0.02, max: 0.5 },
};

const NOMINAL = {
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

function clamp(value, min, max) {
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

function finite(value) {
  if (typeof value !== "number" || !Number.isFinite(value)) return null;
  return value;
}

export function caseFromRequest(body) {
  const next = structuredClone(NOMINAL);
  const sim = body && typeof body === "object" ? body.simulation : null;
  if (sim && typeof sim === "object") {
    const height = finite(sim.initial_z_pos);
    const offset = finite(sim.initial_x_pos);
    const descent = finite(sim.initial_z_velocity);
    const fuel = finite(sim.initial_fuel);
    const wind = finite(sim.wind_x_velocity);
    const pad = finite(sim.initial_pad_x_velocity);
    if (height !== null) next.simulation.initial_z_pos = height;
    if (offset !== null) next.simulation.initial_x_pos = offset;
    if (descent !== null) next.simulation.initial_z_velocity = descent;
    if (fuel !== null) next.simulation.initial_fuel = fuel;
    if (wind !== null) next.simulation.wind_x_velocity = wind;
    if (pad !== null) next.simulation.initial_pad_x_velocity = pad;
  }
  const gain = finite(body?.outer_pid_vertical_position?.gains?.kp);
  if (gain !== null) next.outer_pid_vertical_position.gains.kp = gain;

  next.simulation.initial_z_pos = clamp(next.simulation.initial_z_pos, KNOBS.height.min, KNOBS.height.max);
  next.simulation.initial_x_pos = clamp(next.simulation.initial_x_pos, KNOBS.offset.min, KNOBS.offset.max);
  next.simulation.initial_z_velocity = clamp(next.simulation.initial_z_velocity, KNOBS.descent.min, KNOBS.descent.max);
  next.simulation.initial_fuel = clamp(next.simulation.initial_fuel, KNOBS.fuel.min, KNOBS.fuel.max);
  next.simulation.wind_x_velocity = clamp(next.simulation.wind_x_velocity, KNOBS.wind.min, KNOBS.wind.max);
  next.simulation.initial_pad_x_velocity = clamp(next.simulation.initial_pad_x_velocity, KNOBS.pad.min, KNOBS.pad.max);
  next.outer_pid_vertical_position.gains.kp = clamp(
    next.outer_pid_vertical_position.gains.kp,
    KNOBS.gain.min,
    KNOBS.gain.max,
  );
  return next;
}
