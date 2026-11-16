import type { Knobs } from "./scenes";

/** The best descent. A cost picks a gain, then constraints, convex passes, a replan, and a miss. */
export type DescentId =
  | "desCost"
  | "desPath"
  | "desLoss"
  | "desScvx"
  | "desGusto"
  | "desReplan"
  | "desMiss";

export type DescentSample = {
  t: number;
  height: number;
  rate: number;
  downrange: number;
  thrust: number;
  command: number;
  reference: number;
  tilt: number;
  drift: number;
  flame: number;
};

const STEP = 0.02;
const G = 9.81;
const MASS = 2;
const TMAX = 55;
const VE = 220;

function holdPad(height: number, rate: number): { height: number; rate: number } {
  if (height >= 0) return { height, rate };
  return { height: 0, rate: Math.max(rate, 0) };
}

export function descentDuration(id: DescentId): number {
  if (id === "desCost") return 6;
  return 8;
}

export function costGains(weight: number): { kp: number; kd: number } {
  const kp = Math.sqrt(weight);
  const kd = Math.sqrt(2 * kp);
  return { kp, kd };
}

let cache: { key: string; rows: DescentSample[] } | null = null;

export function descentRows(id: DescentId, knobs: Knobs): DescentSample[] {
  const key = `${id}|${knobs.care}|${knobs.cone}|${knobs.floor}|${knobs.pass}|${knobs.region}|${knobs.gust}|${knobs.fuel}`;
  if (cache && cache.key === key) return cache.rows;
  const rows = integrate(id, knobs);
  cache = { key, rows };
  return rows;
}

export function descentAt(id: DescentId, t: number, knobs: Knobs): DescentSample {
  const rows = descentRows(id, knobs);
  let chosen = rows[0];
  for (const row of rows) {
    if (row.t <= t + 1e-9) chosen = row;
    else break;
  }
  return chosen;
}

export function descentBrief(id: DescentId, knobs: Knobs): string {
  if (id === "desCost") {
    const gains = costGains(knobs.care);
    return `A cost that makes a gain. Height weight q = ${knobs.care.toFixed(2)}, control weight r = 1, so kp = ${gains.kp.toFixed(2)} and kd = ${gains.kd.toFixed(2)}. Mass ${MASS} kg. The ground is not in the cost.`;
  }
  if (id === "desPath") {
    return `A path under constraints. Glide cone ${knobs.cone.toFixed(0)} degrees from the pad. Thrust stays between 6 N and ${TMAX} N.`;
  }
  if (id === "desLoss") {
    return `Lossless convexification. Thrust floor ${knobs.floor.toFixed(0)} N, ceiling ${TMAX} N. The flown magnitude is lifted onto the floor when the command falls inside the hole.`;
  }
  if (id === "desScvx") {
    return `Successive convexification. Pass ${Math.round(knobs.pass)} of 5. Each pass moves the thrust 0.65 of the way toward the guidance solution and flies that profile.`;
  }
  if (id === "desGusto") {
    return `GuSTO. The thrust may move at most ${knobs.region.toFixed(1)} N from the weak first push. The dashed height is that first push.`;
  }
  if (id === "desReplan") {
    return `Replanning. A sideways gust of ${knobs.gust.toFixed(1)} m/s hits at 2.2 s. The dashed downrange is the plan from before the gust.`;
  }
  return `Runs that do not land. Fuel budget ${knobs.fuel.toFixed(2)} kg, exhaust speed ${280} m/s. Thrust stops when the budget is gone.`;
}

function integrate(id: DescentId, knobs: Knobs): DescentSample[] {
  if (id === "desCost") return costRows(knobs.care);
  if (id === "desPath") return pathRows(knobs.cone);
  if (id === "desLoss") return lossRows(knobs.floor);
  if (id === "desScvx") return scvxRows(knobs.pass);
  if (id === "desGusto") return gustoRows(knobs.region);
  if (id === "desReplan") return replanRows(knobs.gust);
  return missRows(knobs.fuel);
}

function costRows(weight: number): DescentSample[] {
  const gains = costGains(weight);
  let height = 18;
  let rate = -6;
  const rows: DescentSample[] = [];
  const end = 6;
  for (let t = 0; t <= end + 1e-9; t += STEP) {
    const excess = -gains.kp * height - gains.kd * rate;
    const thrust = Math.min(TMAX, Math.max(0, MASS * (excess + G)));
    const applied = thrust / MASS - G;
    rows.push(sample(t, height, rate, 0, thrust, thrust, 0, 0, thrust, thrust));
    rate += applied * STEP;
    height += rate * STEP;
    const held = holdPad(height, rate);
    height = held.height;
    rate = held.rate;
  }
  return rows;
}

function pathRows(coneDeg: number): DescentSample[] {
  const wall = Math.tan((coneDeg * Math.PI) / 180);
  let x = 24;
  let z = 36;
  let vx = 1;
  let vz = -2;
  const rows: DescentSample[] = [];
  for (let t = 0; t <= 8 + 1e-9; t += STEP) {
    const limit = Math.max(0, z) * wall;
    const excess = Math.abs(x) - limit;
    let ax = -0.55 * x - 1.2 * vx;
    const az = -0.35 * z - 1.1 * vz;
    if (excess > 0) {
      const outward = x === 0 ? 1 : Math.sign(x);
      ax += -outward * (3 + 2 * excess);
    }
    const command = clampThrust(MASS * ax, MASS * (az + G), 6);
    rows.push(sample(t, z, vz, x, command.mag, command.mag, limit, command.tx, command.tz, command.mag));
    vx += (command.tx / MASS) * STEP;
    vz += (command.tz / MASS - G) * STEP;
    x += vx * STEP;
    z += vz * STEP;
    if (z < 0) {
      z = 0;
      vz = 0;
      vx *= 0.2;
    }
  }
  return rows;
}

function lossRows(floor: number): DescentSample[] {
  let x = 8;
  let z = 40;
  let vx = 0;
  let vz = 0;
  const rows: DescentSample[] = [];
  for (let t = 0; t <= 8 + 1e-9; t += STEP) {
    const left = Math.max(0.35, 8 - t);
    const ax = guidanceAccel(x, vx, left);
    const az = guidanceAccel(z, vz, left);
    const tx = MASS * ax;
    const tz = MASS * (az + G);
    const requested = Math.hypot(tx, tz);
    const command = clampThrust(tx, tz, floor);
    rows.push(sample(t, z, vz, x, command.mag, requested, 0, command.tx, command.tz, command.mag));
    vx += (command.tx / MASS) * STEP;
    vz += (command.tz / MASS - G) * STEP;
    x += vx * STEP;
    z += vz * STEP;
    const held = holdPad(z, vz);
    z = held.height;
    vz = held.rate;
  }
  return rows;
}

function scvxRows(passKnob: number): DescentSample[] {
  const passes = scvxPasses();
  const index = Math.min(passes.length, Math.max(1, Math.round(passKnob))) - 1;
  const current = passes[index];
  const previous = index === 0 ? current : passes[index - 1];
  return current.map((row, i) => {
    const copy = { ...row };
    copy.reference = previous[i].height;
    return copy;
  });
}

function scvxPasses(): DescentSample[][] {
  const count = Math.round(8 / STEP);
  let tx = Array(count).fill(0);
  let tz = Array(count).fill(16);
  const passes: DescentSample[][] = [];
  for (let pass = 0; pass < 5; pass += 1) {
    passes.push(flyProfile(tx, tz, 12, 30, 0, -2));
    const solved = guidanceProfile(12, 30, 0, -2);
    tx = tx.map((value, i) => value + 0.65 * (solved.tx[i] - value));
    tz = tz.map((value, i) => value + 0.65 * (solved.tz[i] - value));
  }
  return passes;
}

function gustoRows(trust: number): DescentSample[] {
  const guessUp = 14;
  const solved = guidanceProfile(12, 30, 0, -2);
  const tx: number[] = [];
  const tz: number[] = [];
  for (let i = 0; i < solved.tx.length; i += 1) {
    const errorX = solved.tx[i];
    const errorZ = solved.tz[i] - guessUp;
    const error = Math.hypot(errorX, errorZ) || 1;
    const scale = Math.min(1, trust / error);
    tx.push(errorX * scale);
    tz.push(guessUp + errorZ * scale);
  }
  const flown = flyProfile(tx, tz, 12, 30, 0, -2);
  const guess = flyProfile(Array(tx.length).fill(0), Array(tz.length).fill(guessUp), 12, 30, 0, -2);
  return flown.map((row, i) => {
    const copy = { ...row };
    copy.reference = guess[i].height;
    return copy;
  });
}

function replanRows(gust: number): DescentSample[] {
  let x = 8;
  let z = 36;
  let vx = 0;
  let vz = -1;
  let oldX = x;
  let oldZ = z;
  let oldVx = vx;
  let oldVz = vz;
  let kicked = false;
  const rows: DescentSample[] = [];
  for (let t = 0; t <= 8 + 1e-9; t += STEP) {
    if (!kicked && t >= 2.2) {
      vx += gust;
      kicked = true;
    }
    const live = guidanceThrust(x, z, vx, vz, t);
    const old = guidanceThrust(oldX, oldZ, oldVx, oldVz, t);
    rows.push(sample(t, z, vz, x, live.mag, live.mag, oldX, live.tx, live.tz, live.mag));
    vx += (live.tx / MASS) * STEP;
    vz += (live.tz / MASS - G) * STEP;
    x += vx * STEP;
    z += vz * STEP;
    const held = holdPad(z, vz);
    z = held.height;
    vz = held.rate;
    oldVx += (old.tx / MASS) * STEP;
    oldVz += (old.tz / MASS - G) * STEP;
    oldX += oldVx * STEP;
    oldZ += oldVz * STEP;
    const heldOld = holdPad(oldZ, oldVz);
    oldZ = heldOld.height;
    oldVz = heldOld.rate;
  }
  return rows;
}

function missRows(fuel: number): DescentSample[] {
  const exhaust = 280;
  let z = 45;
  let vz = -3;
  let left = fuel;
  let hit = false;
  const rows: DescentSample[] = [];
  for (let t = 0; t <= 8 + 1e-9; t += STEP) {
    let thrust = 0;
    if (!hit && left > 0) {
      const leftTime = Math.max(0.4, 8 - t);
      const az = guidanceAccel(z, vz, leftTime);
      thrust = Math.min(TMAX, Math.max(0, MASS * (az + G)));
      const need = (thrust / exhaust) * STEP;
      if (need > left) {
        thrust = 0;
        left = 0;
      } else {
        left -= need;
      }
    }
    rows.push(sample(t, z, vz, 0, thrust, thrust, left, 0, thrust, thrust));
    if (!hit) {
      vz += (thrust / MASS - G) * STEP;
      z += vz * STEP;
      const held = holdPad(z, vz);
      z = held.height;
      vz = held.rate;
      if (z <= 0) hit = true;
    }
  }
  return rows;
}

function guidanceThrust(x: number, z: number, vx: number, vz: number, t: number): { tx: number; tz: number; mag: number } {
  const left = Math.max(0.4, 8 - t);
  const ax = guidanceAccel(x, vx, left);
  const az = guidanceAccel(z, vz, left);
  return clampThrust(MASS * ax, MASS * (az + G), 4);
}

function guidanceAccel(position: number, rate: number, timeLeft: number): number {
  return (-6 * position) / (timeLeft * timeLeft) - (4 * rate) / timeLeft;
}

function guidanceProfile(x0: number, z0: number, vx0: number, vz0: number): { tx: number[]; tz: number[] } {
  const count = Math.round(8 / STEP);
  const tx: number[] = [];
  const tz: number[] = [];
  let x = x0;
  let z = z0;
  let vx = vx0;
  let vz = vz0;
  let mass = MASS;
  for (let i = 0; i < count; i += 1) {
    const left = Math.max(0.45, 8 - i * STEP);
    let dtx = mass * guidanceAccel(x, vx, left);
    let dtz = mass * (guidanceAccel(z, vz, left) + G);
    const magnitude = Math.hypot(dtx, dtz) || 1;
    const used = Math.min(TMAX, Math.max(0, magnitude));
    dtx *= used / magnitude;
    dtz *= used / magnitude;
    tx.push(dtx);
    tz.push(dtz);
    vx += (dtx / mass) * STEP;
    vz += (dtz / mass - G) * STEP;
    x += vx * STEP;
    z += vz * STEP;
    mass = Math.max(1.4, mass - used / VE * STEP);
  }
  return { tx, tz };
}

function flyProfile(tx: number[], tz: number[], x0: number, z0: number, vx0: number, vz0: number): DescentSample[] {
  let x = x0;
  let z = z0;
  let vx = vx0;
  let vz = vz0;
  let mass = MASS;
  const rows: DescentSample[] = [];
  for (let i = 0; i < tx.length; i += 1) {
    const t = i * STEP;
    const magnitude = Math.hypot(tx[i], tz[i]);
    rows.push(sample(t, z, vz, x, magnitude, magnitude, 0, tx[i], tz[i], magnitude));
    vx += (tx[i] / mass) * STEP;
    vz += (tz[i] / mass - G) * STEP;
    x += vx * STEP;
    z += vz * STEP;
    const held = holdPad(z, vz);
    z = held.height;
    vz = held.rate;
    mass = Math.max(1.4, mass - magnitude / VE * STEP);
  }
  const last = rows[rows.length - 1];
  rows.push({ ...last, t: 8 });
  return rows;
}

function clampThrust(tx: number, tz: number, floor: number): { tx: number; tz: number; mag: number } {
  const magnitude = Math.hypot(tx, tz);
  const used = Math.min(TMAX, Math.max(floor, magnitude));
  if (magnitude < 1e-8) {
    return { tx: 0, tz: used, mag: used };
  }
  const scale = used / magnitude;
  return { tx: tx * scale, tz: tz * scale, mag: used };
}

function sample(
  t: number,
  height: number,
  rate: number,
  downrange: number,
  thrust: number,
  command: number,
  reference: number,
  tx: number,
  tz: number,
  flameSource: number,
): DescentSample {
  const tilt = tx === 0 && tz === 0 ? 0 : -Math.atan2(tx, tz);
  return {
    t,
    height,
    rate,
    downrange,
    thrust,
    command,
    reference,
    tilt,
    drift: downrange,
    flame: Math.min(1, Math.max(0, flameSource / TMAX)),
  };
}
