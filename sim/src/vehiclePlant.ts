import type { Knobs } from "./scenes";

/** The vehicle. Ascent is one climb. The later scenes are the nose, the air, the estimate, guidance, and the autopilot. */
export type VehicleId = "vehAscent" | "vehNose" | "vehWind" | "vehEstimate" | "vehGuide" | "vehAuto";

export type VehicleSample = {
  t: number;
  height: number;
  rate: number;
  downrange: number;
  attack: number;
  estimate: number;
  measured: number;
  pitch: number;
  aim: number;
  mass: number;
  accel: number;
  thrust: number;
  tilt: number;
  drift: number;
  flame: number;
};

const STEP = 0.02;
const MASS = 2;
const G = 9.81;
const DEG = 180 / Math.PI;
const NOSE_THRUST = 50;
const CLIMB_THRUST = 45;

export function vehicleDuration(id: VehicleId): number {
  if (id === "vehGuide" || id === "vehAuto") return 8;
  return 6;
}

let cache: { key: string; rows: VehicleSample[] } | null = null;

export function vehicleRows(id: VehicleId, knobs: Knobs): VehicleSample[] {
  const key = `${id}|${knobs.lift}|${knobs.lean}|${knobs.wind}|${knobs.trust}|${knobs.kick}|${knobs.hold}|${knobs.noise}`;
  if (cache && cache.key === key) return cache.rows;

  const end = vehicleDuration(id);
  const rows: VehicleSample[] = [];
  let estimate = 0;
  let rateHat = 0;
  let pitch = 0;
  let pitchRate = 0;
  let speed = 0;
  let height = 0;
  let downrange = 0;
  let sideSpeed = 0;
  let aim = 0;
  let xHat = 0;
  let zHat = 0;

  for (let t = 0; t <= end + 1e-9; t += STEP) {
    rows.push(sample(id, t, knobs, { estimate, pitch, height, downrange, speed, aim: id === "vehGuide" ? aim : 0 }));
    if (id === "vehEstimate") {
      const accel = CLIMB_THRUST / MASS - G;
      const truth = 0.5 * accel * t * t;
      const measured = truth + knobs.noise * Math.sin(2.2 * t);
      const ratePred = rateHat + accel * STEP;
      const pred = estimate + rateHat * STEP + 0.5 * accel * STEP * STEP;
      const gap = measured - pred;
      // Fixed-gain complementary update (not a Kalman gain).
      estimate = pred + knobs.trust * gap * STEP;
      rateHat = ratePred + knobs.trust * gap * STEP * 0.5;
    }
    if (id === "vehGuide") {
      // Closed-loop guidance: aim the nose at a ground target using a filtered state.
      const xMeas = downrange + 0.4 * Math.sin(2.4 * t);
      const zMeas = height + 0.4 * Math.cos(1.7 * t);
      xHat += 1.2 * (xMeas - xHat) * STEP;
      zHat += 1.2 * (zMeas - zHat) * STEP;
      const targetX = knobs.kick;
      // Pitch from vertical toward the target, clipped so the climb still has lift.
      const raw = Math.atan2(targetX - xHat, Math.max(20, zHat)) * DEG;
      aim = Math.max(-5, Math.min(40, raw));
      pitch += ((aim - pitch) / 0.7) * STEP;
      const tilt = pitch / DEG;
      const specific = NOSE_THRUST / MASS;
      const up = specific * Math.cos(tilt) - G;
      const side = specific * Math.sin(tilt);
      speed += up * STEP;
      sideSpeed += side * STEP;
      height += speed * STEP;
      downrange += sideSpeed * STEP;
      if (height < 0) {
        height = 0;
        speed = Math.max(speed, 0);
      }
      estimate = xHat;
    }
    if (id === "vehAuto") {
      const mass = burnMass(t);
      const command = t < 0.4 ? 0 : 12;
      const accel = 40 / mass - G;
      speed += accel * STEP;
      height += speed * STEP;
      const air = 0.0003 * speed * speed;
      const inertia = 0.25 * mass;
      const wn2 = knobs.hold / inertia;
      const wn = Math.sqrt(wn2);
      const error = (command - pitch) / DEG;
      const alpha = wn2 * error - 1.6 * wn * pitchRate - air * (pitch / DEG);
      pitchRate += alpha * STEP;
      pitch += pitchRate * STEP * DEG;
    }
  }

  cache = { key, rows };
  return rows;
}

export function vehicleAt(id: VehicleId, t: number, knobs: Knobs): VehicleSample {
  const rows = vehicleRows(id, knobs);
  let chosen = rows[0];
  for (const row of rows) {
    if (row.t <= t + 1e-9) chosen = row;
    else break;
  }
  return chosen;
}

export function vehicleBrief(id: VehicleId, knobs: Knobs): string {
  if (id === "vehAscent") {
    return `Ascent. Mass ${MASS} kg. Thrust ${knobs.lift.toFixed(0)} N. Height and rate come from thrust over mass, minus gravity. Allowed chart columns: height, rate, accel, thrust.`;
  }
  if (id === "vehNose") {
    return `Thrust along the nose. Thrust ${NOSE_THRUST} N on a ${MASS} kg vehicle, tilted ${knobs.lean.toFixed(0)} degrees from vertical. Height and downrange integrate the two components. Allowed chart columns: height, downrange, rate, pitch.`;
  }
  if (id === "vehWind") {
    return `Wind. The climb uses ${CLIMB_THRUST} N on ${MASS} kg. Wind is ${knobs.wind.toFixed(0)} m/s across the path. Attack is the angle between the nose and the air. Allowed chart columns: rate, attack, height.`;
  }
  if (id === "vehEstimate") {
    return `Sensors and an estimate. Fixed-gain filter (not Kalman). Truth is the ${CLIMB_THRUST} N climb. The sensor adds a ${knobs.noise.toFixed(1)} m wave. Gain ${knobs.trust.toFixed(1)}. Chart live series are errors from truth. Allowed chart columns: height, estimate, measured, rate.`;
  }
  if (id === "vehGuide") {
    return `Guidance. Closed-loop: a filtered estimate of downrange and height aims the nose at a ground target ${knobs.kick.toFixed(0)} m downrange. The body follows that command with τ = 0.7 s. Allowed chart columns: downrange, height, aim, pitch, estimate.`;
  }
  return `Autopilot. The command is 12 degrees after 0.4 s. Gain is ${knobs.hold.toFixed(1)}. Mass falls from 3 kg to 1.4 kg, and the air moment grows with speed squared. Allowed chart columns: pitch, mass, aim, height.`;
}

type Pose = {
  estimate: number;
  pitch: number;
  height: number;
  downrange: number;
  speed: number;
  aim: number;
};

function sample(id: VehicleId, t: number, knobs: Knobs, pose: Pose): VehicleSample {
  const row = empty(t);

  if (id === "vehAscent") {
    const accel = knobs.lift / MASS - G;
    row.accel = accel;
    row.thrust = knobs.lift;
    row.rate = accel * t;
    row.height = 0.5 * accel * t * t;
    row.flame = knobs.lift / 70;
    return row;
  }

  if (id === "vehNose") {
    const tilt = knobs.lean / DEG;
    const specific = NOSE_THRUST / MASS;
    const up = specific * Math.cos(tilt) - G;
    const side = specific * Math.sin(tilt);
    row.accel = up;
    row.thrust = NOSE_THRUST;
    row.rate = up * t;
    row.height = 0.5 * up * t * t;
    row.downrange = 0.5 * side * t * t;
    row.pitch = knobs.lean;
    row.tilt = -tilt;
    row.drift = row.downrange;
    row.flame = 0.85;
    return row;
  }

  if (id === "vehWind") {
    const accel = CLIMB_THRUST / MASS - G;
    row.rate = accel * t;
    row.height = 0.5 * accel * t * t;
    const speed = Math.max(row.rate, 1);
    row.attack = Math.atan(knobs.wind / speed) * DEG;
    row.flame = 0.75;
    return row;
  }

  if (id === "vehEstimate") {
    const accel = CLIMB_THRUST / MASS - G;
    row.rate = accel * t;
    row.height = 0.5 * accel * t * t;
    row.measured = row.height + knobs.noise * Math.sin(2.2 * t);
    row.estimate = pose.estimate;
    row.flame = 0.75;
    return row;
  }

  if (id === "vehGuide") {
    row.aim = pose.aim;
    row.pitch = pose.pitch;
    row.height = pose.height;
    row.downrange = pose.downrange;
    row.rate = pose.speed;
    row.estimate = pose.estimate;
    row.tilt = -pose.pitch / DEG;
    row.drift = pose.downrange;
    row.flame = 0.8;
    return row;
  }

  row.aim = t < 0.4 ? 0 : 12;
  row.pitch = pose.pitch;
  row.mass = burnMass(t);
  row.height = pose.height;
  row.rate = pose.speed;
  row.tilt = -pose.pitch / DEG;
  row.flame = 0.7;
  return row;
}

function burnMass(t: number): number {
  const burned = Math.min(1, t / 8);
  return 3 - 1.6 * burned;
}

function empty(t: number): VehicleSample {
  return {
    t,
    height: 0,
    rate: 0,
    downrange: 0,
    attack: 0,
    estimate: 0,
    measured: 0,
    pitch: 0,
    aim: 0,
    mass: 0,
    accel: 0,
    thrust: 0,
    tilt: 0,
    drift: 0,
    flame: 0,
  };
}
