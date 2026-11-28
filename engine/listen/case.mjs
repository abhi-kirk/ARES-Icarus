// Same case the page is allowed to send. Knob fields are copied and clamped.
// Timestep and the other gains stay at the nominal landing.
// Keep the ranges in step with sim/src/flight/case.ts.

const KNOBS = {
  height: { min: 40, max: 600 },
  offset: { min: -300, max: 300 },
  descent: { min: -60, max: 10 },
  fuel: { min: 50, max: 1200 },
  wind: { min: -15, max: 15 },
  pad: { min: -4, max: 4 },
  gust: { min: 0, max: 20 },
  gain: { min: 0.02, max: 0.5 },
  floor: { min: 0, max: 15000 },
  ceiling: { min: 12000, max: 25000 },
  cone: { min: 5, max: 90 },
  glide: { min: 5, max: 90 },
  lag: { min: 0, max: 1.5 },
  gimbal: { min: 4, max: 25 },
  inertia: { min: 500, max: 5000 },
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
    thrust_min: 0.0,
    thrust_max: 25000.0,
    pointing_cone_deg: 90.0,
    glide_cone_deg: 90.0,
    glide_pull: 0.5,
    thrust_tau: 0.0,
    drag_scale: 1.0,
    gravity: 9.81,
    thrust_slew: 0.0,
    use_6dof: true,
    Ixx: 2000.0,
    Iyy: 2000.0,
    Izz: 400.0,
    gimbal_lever: 8.0,
    gimbal_max_deg: 15.0,
    attitude_kp: 1.5,
    attitude_kd: 2.45,
    controller: "mpc",
    entry: false,
    mpc_horizon: 50,
    mpc_dt: 0.1,
    mpc_w_pos: 40.0,
    mpc_w_vel: 50.0,
    mpc_w_u: 1.0e-9,
    mpc_w_terminal: 120.0,
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
    const gust = finite(sim.gust_magnitude);
    const floor = finite(sim.thrust_min);
    const ceiling = finite(sim.thrust_max);
    const cone = finite(sim.pointing_cone_deg);
    const glide = finite(sim.glide_cone_deg);
    const lag = finite(sim.thrust_tau);
    const drag = finite(sim.drag_scale);
    const gravity = finite(sim.gravity);
    const gimbal = finite(sim.gimbal_max_deg);
    const inertia = finite(sim.Ixx);
    const cross = finite(sim.initial_y_pos);
    const vx = finite(sim.initial_x_velocity);
    const vy = finite(sim.initial_y_velocity);
    const windY = finite(sim.wind_y_velocity);
    const padY = finite(sim.initial_pad_y_velocity);
    if (typeof sim.controller === "string") {
      const mode = sim.controller.toLowerCase();
      if (mode === "pid" || mode === "mpc" || mode === "scvx") next.simulation.controller = mode;
    }
    next.simulation.entry = sim.entry === true;
    if (height !== null) next.simulation.initial_z_pos = height;
    if (offset !== null) next.simulation.initial_x_pos = offset;
    if (descent !== null) next.simulation.initial_z_velocity = descent;
    if (fuel !== null) next.simulation.initial_fuel = fuel;
    if (wind !== null) next.simulation.wind_x_velocity = wind;
    if (pad !== null) next.simulation.initial_pad_x_velocity = pad;
    if (gust !== null) next.simulation.gust_magnitude = gust;
    if (floor !== null) next.simulation.thrust_min = floor;
    if (ceiling !== null) next.simulation.thrust_max = ceiling;
    if (cone !== null) next.simulation.pointing_cone_deg = cone;
    if (glide !== null) next.simulation.glide_cone_deg = glide;
    if (lag !== null) next.simulation.thrust_tau = lag;
    if (drag !== null) next.simulation.drag_scale = drag;
    if (gravity !== null) next.simulation.gravity = gravity;
    if (gimbal !== null) next.simulation.gimbal_max_deg = gimbal;
    if (inertia !== null) {
      next.simulation.Ixx = inertia;
      next.simulation.Iyy = inertia;
    }
    if (cross !== null) next.simulation.initial_y_pos = cross;
    if (vx !== null) next.simulation.initial_x_velocity = vx;
    if (vy !== null) next.simulation.initial_y_velocity = vy;
    if (windY !== null) next.simulation.wind_y_velocity = windY;
    if (padY !== null) next.simulation.initial_pad_y_velocity = padY;
  }
  const gain = finite(body?.outer_pid_vertical_position?.gains?.kp);
  if (gain !== null) next.outer_pid_vertical_position.gains.kp = gain;

  next.simulation.initial_z_pos = clamp(next.simulation.initial_z_pos, KNOBS.height.min, KNOBS.height.max);
  next.simulation.initial_x_pos = clamp(next.simulation.initial_x_pos, KNOBS.offset.min, KNOBS.offset.max);
  next.simulation.initial_y_pos = clamp(next.simulation.initial_y_pos, KNOBS.offset.min, KNOBS.offset.max);
  next.simulation.initial_x_velocity = clamp(next.simulation.initial_x_velocity, -40, 40);
  next.simulation.initial_y_velocity = clamp(next.simulation.initial_y_velocity, -40, 40);
  next.simulation.wind_y_velocity = clamp(next.simulation.wind_y_velocity, KNOBS.wind.min, KNOBS.wind.max);
  next.simulation.initial_pad_y_velocity = clamp(
    next.simulation.initial_pad_y_velocity,
    KNOBS.pad.min,
    KNOBS.pad.max,
  );
  next.simulation.initial_z_velocity = clamp(next.simulation.initial_z_velocity, KNOBS.descent.min, KNOBS.descent.max);
  next.simulation.initial_fuel = clamp(next.simulation.initial_fuel, KNOBS.fuel.min, KNOBS.fuel.max);
  next.simulation.wind_x_velocity = clamp(next.simulation.wind_x_velocity, KNOBS.wind.min, KNOBS.wind.max);
  next.simulation.initial_pad_x_velocity = clamp(next.simulation.initial_pad_x_velocity, KNOBS.pad.min, KNOBS.pad.max);
  next.simulation.gust_magnitude = clamp(next.simulation.gust_magnitude, KNOBS.gust.min, KNOBS.gust.max);
  next.simulation.thrust_min = clamp(next.simulation.thrust_min, 0, 25000);
  next.simulation.thrust_max = clamp(next.simulation.thrust_max, 0, 25000);
  if (next.simulation.thrust_min > next.simulation.thrust_max) {
    next.simulation.thrust_min = next.simulation.thrust_max;
  }
  next.simulation.pointing_cone_deg = clamp(next.simulation.pointing_cone_deg, KNOBS.cone.min, KNOBS.cone.max);
  next.simulation.glide_cone_deg = clamp(next.simulation.glide_cone_deg, KNOBS.glide.min, KNOBS.glide.max);
  next.simulation.thrust_tau = clamp(next.simulation.thrust_tau, KNOBS.lag.min, KNOBS.lag.max);
  next.simulation.drag_scale = clamp(next.simulation.drag_scale, 0, 2);
  next.simulation.gravity = clamp(next.simulation.gravity, 1, 12);
  next.simulation.gimbal_max_deg = clamp(next.simulation.gimbal_max_deg, KNOBS.gimbal.min, KNOBS.gimbal.max);
  next.simulation.Ixx = clamp(next.simulation.Ixx, KNOBS.inertia.min, KNOBS.inertia.max);
  next.simulation.Iyy = next.simulation.Ixx;
  next.outer_pid_vertical_position.gains.kp = clamp(
    next.outer_pid_vertical_position.gains.kp,
    KNOBS.gain.min,
    KNOBS.gain.max,
  );
  return next;
}
