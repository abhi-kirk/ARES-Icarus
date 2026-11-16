import type { Knobs } from "./scenes";

/** Attitude scenes for The turn. Angles in the chart are degrees. Torques are N·m. */
export type TurnId =
  | "turnFrames"
  | "turnLock"
  | "turnQuat"
  | "turnSpin"
  | "turnInertia"
  | "turnGimbal"
  | "turnAir"
  | "turnSlosh";

export type TurnSample = {
  t: number;
  worldX: number;
  worldZ: number;
  pitch: number;
  axisGap: number;
  quatW: number;
  quatZ: number;
  angle: number;
  spin: number;
  side: number;
  moment: number;
  attack: number;
  fill: number;
  offset: number;
  yaw: number;
  tilt: number;
  gimbal: number;
  slosh: number;
  cp: number;
  flame: number;
};

const STEP = 0.02;
const TORQUE = 2;
const THRUST = 50;
const LEVER = 1.5;
const DEG = 180 / Math.PI;

export function turnDuration(id: TurnId): number {
  if (id === "turnGimbal") return 6;
  if (id === "turnAir" || id === "turnSlosh") return 10;
  return 8;
}

let cache: { key: string; rows: TurnSample[] } | null = null;

export function turnRows(id: TurnId, knobs: Knobs): TurnSample[] {
  const key = `${id}|${knobs.yaw}|${knobs.pitch}|${knobs.tumble}|${knobs.rate}|${knobs.inertia}|${knobs.gimbal}|${knobs.margin}|${knobs.slosh}`;
  if (cache && cache.key === key) return cache.rows;
  const end = turnDuration(id);
  const rows: TurnSample[] = [];
  let attack = 12 / DEG;
  let attackRate = 0;
  // Negative margin runs away; stop at a full tumble so the chart stays readable.
  const attackLimit = Math.PI;
  for (let t = 0; t <= end + 1e-9; t += STEP) {
    rows.push(sample(id, t, knobs, attack));
    if (id === "turnAir") {
      const accel = -knobs.margin * 2.25 * attack;
      attackRate += accel * STEP;
      attack += attackRate * STEP;
      if (Math.abs(attack) > attackLimit) {
        attack = Math.sign(attack) * attackLimit;
        attackRate = 0;
      }
    }
  }
  cache = { key, rows };
  return rows;
}

export function turnAt(id: TurnId, t: number, knobs: Knobs): TurnSample {
  const rows = turnRows(id, knobs);
  let chosen = rows[0];
  for (const row of rows) {
    if (row.t <= t + 1e-9) chosen = row;
    else break;
  }
  return chosen;
}

function sample(id: TurnId, t: number, knobs: Knobs, attack: number): TurnSample {
  const row = empty(t);
  const end = turnDuration(id);

  if (id === "turnFrames") {
    const yaw = (knobs.yaw / DEG) * (t / end);
    row.worldX = Math.cos(yaw);
    row.worldZ = Math.sin(yaw);
    row.yaw = yaw;
    return row;
  }

  if (id === "turnLock") {
    const pitchDeg = knobs.pitch * (t / end);
    row.pitch = pitchDeg;
    row.axisGap = 90 - pitchDeg;
    row.tilt = pitchDeg / DEG - Math.PI / 2;
    return row;
  }

  if (id === "turnQuat") {
    const pitchDeg = knobs.tumble * (t / end);
    const half = pitchDeg / DEG / 2;
    row.pitch = pitchDeg;
    row.quatW = Math.cos(half);
    row.quatZ = Math.sin(half);
    row.tilt = pitchDeg / DEG - Math.PI / 2;
    return row;
  }

  if (id === "turnSpin") {
    row.spin = knobs.rate;
    row.angle = knobs.rate * t;
    row.yaw = row.angle / DEG;
    return row;
  }

  if (id === "turnInertia") {
    const alpha = TORQUE / knobs.inertia;
    const omega = alpha * t;
    row.spin = omega * DEG;
    row.angle = 0.5 * alpha * t * t * DEG;
    row.moment = TORQUE;
    row.yaw = row.angle / DEG;
    return row;
  }

  if (id === "turnGimbal") {
    const deg = knobs.gimbal * Math.min(1, t / 1.5);
    const rad = deg / DEG;
    row.pitch = deg;
    row.side = THRUST * Math.sin(rad);
    row.moment = LEVER * row.side;
    row.gimbal = rad;
    return row;
  }

  if (id === "turnAir") {
    const attackDeg = attack * DEG;
    row.attack = attackDeg;
    row.moment = -8 * knobs.margin * attack;
    row.tilt = -attack;
    row.cp = knobs.margin;
    return row;
  }

  const fuel = 1.5 * Math.max(0, 1 - t / 8);
  const swing = (knobs.slosh / DEG) * Math.sin(2 * Math.PI * 0.8 * t) * Math.exp(-0.12 * t);
  const slug = 0.18 * Math.sin(swing);
  row.fill = fuel;
  row.offset = (fuel / (1.5 + fuel)) * slug;
  row.slosh = swing;
  row.cp = fuel / 1.5;
  return row;
}

function empty(t: number): TurnSample {
  return {
    t,
    worldX: 0,
    worldZ: 0,
    pitch: 0,
    axisGap: 0,
    quatW: 0,
    quatZ: 0,
    angle: 0,
    spin: 0,
    side: 0,
    moment: 0,
    attack: 0,
    fill: 0,
    offset: 0,
    yaw: 0,
    tilt: 0,
    gimbal: 0,
    slosh: 0,
    cp: 0,
    flame: 0,
  };
}

export function turnBrief(id: TurnId, knobs: Knobs): string {
  if (id === "turnFrames") {
    return `Frames. A body side axis of length 1 yaws through ${knobs.yaw.toFixed(0)} degrees. worldX and worldZ are that same axis in the world. Allowed chart columns: worldX, worldZ.`;
  }
  if (id === "turnLock") {
    return `Gimbal lock. Pitch rises from horizontal to ${knobs.pitch.toFixed(0)} degrees. axisGap is the angle between the nose and the vertical yaw axis. They meet at 90 degrees of pitch. Allowed chart columns: pitch, axisGap.`;
  }
  if (id === "turnQuat") {
    return `Quaternions. One pitch about Z, from 0 to ${knobs.tumble.toFixed(0)} degrees. quatW is cos(pitch/2) and quatZ is sin(pitch/2), with pitch in radians. Allowed chart columns: quatW, quatZ, pitch.`;
  }
  if (id === "turnSpin") {
    return `Spin. Constant rate ${knobs.rate.toFixed(0)} degrees per second, so angle is that rate times time. Allowed chart columns: angle, spin.`;
  }
  if (id === "turnInertia") {
    return `Inertia. A ${TORQUE} N·m torque on inertia ${knobs.inertia.toFixed(2)} kg·m². Rate is torque/inertia times time. Allowed chart columns: angle, spin, moment.`;
  }
  if (id === "turnGimbal") {
    return `Gimbal. One nozzle tilts to ${knobs.gimbal.toFixed(1)} degrees over 1.5 s and holds. Thrust is ${THRUST} N and the lever to the center of mass is ${LEVER} m. Side force and moment both come from sin(gimbal). Allowed chart columns: side, moment, pitch.`;
  }
  if (id === "turnAir") {
    const stability =
      knobs.margin < 0
        ? "Negative margin is unstable: attack grows until a ±180° tumble stop."
        : "Positive margin puts the center of pressure behind the center of mass, so the moment opposes angle of attack.";
    return `Air. Static margin ${knobs.margin.toFixed(2)} calibers. ${stability} Allowed chart columns: attack, moment.`;
  }
  return `Slosh. Fuel burns from 1.5 kg toward empty. The slug swings with amplitude ${knobs.slosh.toFixed(0)} degrees, and offset is the lateral shift of the vehicle center of mass. Allowed chart columns: fill, offset.`;
}
