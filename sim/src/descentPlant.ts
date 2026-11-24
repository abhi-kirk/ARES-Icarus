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

export function costGains(weight: number, spend = 1): { kp: number; kd: number } {
  const r = Math.max(0.05, spend);
  const kp = Math.sqrt(weight / r);
  const kd = Math.sqrt(2 * kp);
  return { kp, kd };
}

let cache: { key: string; rows: DescentSample[] } | null = null;

export function descentRows(id: DescentId, knobs: Knobs): DescentSample[] {
  const key = `${id}|${knobs.care}|${knobs.spend}|${knobs.cone}|${knobs.floor}|${knobs.pass}|${knobs.region}|${knobs.gust}|${knobs.fuel}|${knobs.cause}`;
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
    const gains = costGains(knobs.care, knobs.spend);
    return String.raw`A cost that makes a gain. Height weight $q = ${knobs.care.toFixed(2)}$, control weight $r = ${knobs.spend.toFixed(2)}$, so $k_p = ${gains.kp.toFixed(2)}$ and $k_d = ${gains.kd.toFixed(2)}$. Mass ${MASS} kg. Thrust floors at 0 N. The ground is not in the cost.`;
  }
  if (id === "desPath") {
    return String.raw`A path under constraints. Start at downrange ${PATH_X0} m, height ${PATH_Z0} m (inside an 8° cone). Glide cone $\theta = ${knobs.cone.toFixed(0)}°$ measured from vertical through the pad. Wall is $|x| \le z \tan\theta$. Thrust is a vector with magnitude between ${PATH_T_FLOOR} N and ${TMAX} N.`;
  }
  if (id === "desLoss") {
    return String.raw`Lossless convexification. Thrust floor ${knobs.floor.toFixed(0)} N, ceiling ${TMAX} N. $\sigma$ is stored in reference and equals $|T|$ (minimum-fuel puts the thrust on the slack). Command is the uncapped ask.`;
  }
  if (id === "desScvx") {
    const infos = scvxPassInfo();
    const index = Math.min(infos.length, Math.max(1, Math.round(knobs.pass))) - 1;
    const info = infos[index];
    const detail = info
      ? ` End height ${info.error.toFixed(2)} m, trust radius ${info.radius.toFixed(1)} N${info.accepted ? "" : " (step rejected)"}.`
      : "";
    return `Successive convexification. Pass ${Math.round(knobs.pass)} of 5: fly, solve a pad-aiming subproblem, limit the thrust step by a trust region, add virtual vertical slack, then accept or shrink.${detail}`;
  }
  if (id === "desGusto") {
    const info = gustoInfo();
    const detail = info
      ? ` End height ${info.error.toFixed(2)} m after ${info.accepts} accept / ${info.rejects} reject, final $\\Delta = ${info.radius.toFixed(1)}\\,\\mathrm{N}$.`
      : "";
    return `GuSTO. Start from a weak push with trust radius ${knobs.region.toFixed(1)} N. Each iteration linearizes about the flown path, takes a hard trust-limited step, and accepts or shrinks on a merit test (end-height error). No virtual control.${detail}`;
  }
  if (id === "desReplan") {
    return `Replanning (MPC-style). A sideways gust of ${knobs.gust.toFixed(1)} m/s hits at 2.2 s. Live guidance recomputes from the current state each step, with thrust capped at ${REPLAN_TMAX} N and a ${REPLAN_CONE_DEG}° glide cone from vertical. Large gusts can saturate those limits and miss. The dashed downrange is the plan from before the gust.`;
  }
  const outcome = missOutcome(knobs);
  if (knobs.cause >= 0.5) {
    return String.raw`Runs that do not land (INFEASIBLE mode). Thrust ceiling ${MISS_HARD_CEILING} N on ${MASS} kg cannot meet soft landing ($|\dot{z}| \le ${TOUCH_SPEED}\,\mathrm{m/s}$ at the pad) from 45 m / −3 m/s. Status ${outcome.status}, impact ${outcome.impactRate.toFixed(2)} m/s.`;
  }
  return String.raw`Runs that do not land (fuel mode). Fuel budget ${knobs.fuel.toFixed(2)} kg, exhaust 280 m/s. Soft landing needs $|\dot{z}| \le ${TOUCH_SPEED}\,\mathrm{m/s}$ at the pad. Status ${outcome.status}, impact ${outcome.impactRate.toFixed(2)} m/s.`;
}

function integrate(id: DescentId, knobs: Knobs): DescentSample[] {
  if (id === "desCost") return costRows(knobs.care, knobs.spend);
  if (id === "desPath") return pathRows(knobs.cone);
  if (id === "desLoss") return lossRows(knobs.floor);
  if (id === "desScvx") return scvxRows(knobs.pass);
  if (id === "desGusto") return gustoRows(knobs.region);
  if (id === "desReplan") return replanRows(knobs.gust);
  return missRows(knobs.fuel, knobs.cause);
}

/** Soft-landing speed limit at the pad (m/s). Faster is a miss. */
export const TOUCH_SPEED = 0.5;
/** Ceiling that cannot soft-land the miss-scene start state (teaching INFEASIBLE). */
export const MISS_HARD_CEILING = 18;

export type MissStatus = "soft" | "miss" | "infeasible";

export function missOutcome(knobs: Knobs): { status: MissStatus; impactRate: number; touchLimit: number } {
  const rows = descentRows("desMiss", knobs);
  const end = rows[rows.length - 1];
  const impactRate = end.rate;
  const soft = Math.abs(impactRate) <= TOUCH_SPEED;
  if (knobs.cause >= 0.5) {
    return { status: soft ? "soft" : "infeasible", impactRate, touchLimit: TOUCH_SPEED };
  }
  return { status: soft ? "soft" : "miss", impactRate, touchLimit: TOUCH_SPEED };
}

function costRows(weight: number, spend: number): DescentSample[] {
  const gains = costGains(weight, spend);
  let height = 18;
  let rate = -6;
  const rows: DescentSample[] = [];
  const end = 6;
  // No pad clamp here: the lesson is that the cost has no ground, so the path can cross the pad.
  for (let t = 0; t <= end + 1e-9; t += STEP) {
    const excess = -gains.kp * height - gains.kd * rate;
    const raw = MASS * (excess + G);
    const thrust = Math.min(TMAX, Math.max(0, raw));
    const applied = thrust / MASS - G;
    // command holds the uncapped demand so the 0 N floor is visible against it
    rows.push(sample(t, height, rate, 0, thrust, raw, 0, 0, thrust, thrust));
    rate += applied * STEP;
    height += rate * STEP;
  }
  return rows;
}

/** Start inside the tightest cone (8°): |x| ≤ z tan 8° ≈ 5.1 m at z = 36. */
const PATH_X0 = 4;
const PATH_Z0 = 36;
const PATH_T_FLOOR = 6;

function pathRows(coneDeg: number): DescentSample[] {
  const wall = Math.tan((coneDeg * Math.PI) / 180);
  let x = PATH_X0;
  let z = PATH_Z0;
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
    const command = clampThrust(MASS * ax, MASS * (az + G), PATH_T_FLOOR);
    // reference = cone wall half-width so the chart can show |x| against the wall
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
    const flown = clampThrust(tx, tz, floor);
    // reference holds σ = |T| (lossless: thrust sits on the slack)
    rows.push(sample(t, z, vz, x, flown.mag, requested, flown.mag, flown.tx, flown.tz, flown.mag));
    vx += (flown.tx / MASS) * STEP;
    vz += (flown.tz / MASS - G) * STEP;
    x += vx * STEP;
    z += vz * STEP;
    const held = holdPad(z, vz);
    z = held.height;
    vz = held.rate;
  }
  return rows;
}

/** End-height error and trust radius after each SCvx pass (for the knob aside). */
export type ScvxPassInfo = { error: number; radius: number; accepted: boolean };

let scvxInfoCache: ScvxPassInfo[] | null = null;

export function scvxPassInfo(): ScvxPassInfo[] {
  if (!scvxInfoCache) scvxPasses();
  return scvxInfoCache ?? [];
}

function scvxRows(passKnob: number): DescentSample[] {
  const { passes } = scvxPasses();
  const index = Math.min(passes.length, Math.max(1, Math.round(passKnob))) - 1;
  const current = passes[index];
  const previous = index === 0 ? current : passes[index - 1];
  return current.map((row, i) => {
    const copy = { ...row };
    copy.reference = previous[i].height;
    return copy;
  });
}

function scvxPasses(): { passes: DescentSample[][] } {
  const count = Math.round(8 / STEP);
  // Weak first push, well below weight, so pass 1 falls through the pad.
  let tx = Array(count).fill(0);
  let tz = Array(count).fill(8);
  let radius = 4;
  const passes: DescentSample[][] = [];
  const info: ScvxPassInfo[] = [];
  // Pad-aiming subproblem (closed-form stand-in for the convex program).
  const solved = guidanceProfile(12, 30, 0, -2);

  for (let pass = 0; pass < 5; pass += 1) {
    // Fly without a pad clamp so early passes can miss through the ground.
    const flown = flyProfile(tx, tz, 12, 30, 0, -2, false);
    const error = flown[flown.length - 1].height;
    passes.push(flown);

    // Trust region: only a limited step in thrust from the current profile toward the solve.
    const stepped = trustStep(tx, tz, solved.tx, solved.tz, radius);
    const steppedFlight = flyProfile(stepped.tx, stepped.tz, 12, 30, 0, -2, false);
    const leftover = steppedFlight[steppedFlight.length - 1].height;
    // Virtual control: vertical slack once the trust step is already near the pad.
    const virtual = virtualVertical(leftover, count);
    const aimTx = stepped.tx;
    const aimTz = stepped.tz.map((value, i) => value + virtual[i]);
    const trial = flyProfile(aimTx, aimTz, 12, 30, 0, -2, false);
    const trialError = trial[trial.length - 1].height;

    const radiusShown = radius;
    let accepted = false;
    if (Math.abs(trialError) < Math.abs(error) - 0.05) {
      tx = aimTx;
      tz = aimTz;
      radius = Math.min(35, radius * 1.45);
      accepted = true;
    } else if (Math.abs(leftover) < Math.abs(error) - 0.05) {
      // Virtual made it worse; keep the trust-limited step alone and grow more slowly.
      tx = stepped.tx;
      tz = stepped.tz;
      radius = Math.min(35, radius * 1.25);
      accepted = true;
    } else {
      // Reject the step and shrink the trust region.
      radius = Math.max(2.5, radius * 0.65);
    }
    info.push({ error, radius: radiusShown, accepted });
  }

  scvxInfoCache = info;
  return { passes };
}

function trustStep(
  tx: number[],
  tz: number[],
  solTx: number[],
  solTz: number[],
  radius: number,
): { tx: number[]; tz: number[] } {
  const nextTx: number[] = [];
  const nextTz: number[] = [];
  for (let i = 0; i < tx.length; i += 1) {
    let dtx = solTx[i] - tx[i];
    let dtz = solTz[i] - tz[i];
    const step = Math.hypot(dtx, dtz);
    if (step > radius && step > 1e-9) {
      const scale = radius / step;
      dtx *= scale;
      dtz *= scale;
    }
    nextTx.push(tx[i] + dtx);
    nextTz.push(tz[i] + dtz);
  }
  return { tx: nextTx, tz: nextTz };
}

/** Soft vertical slack once a trust step is near the pad. Far leftovers stay at zero. */
function virtualVertical(leftover: number, count: number): number[] {
  if (Math.abs(leftover) >= 25 || Math.abs(leftover) < 0.15) return Array(count).fill(0);
  const slack = (-leftover / 6) * MASS;
  const per = Math.max(-8, Math.min(8, slack));
  return Array(count).fill(per);
}

export type GustoInfo = { error: number; radius: number; accepts: number; rejects: number };

let gustoInfoCache: GustoInfo | null = null;

export function gustoInfo(): GustoInfo | null {
  return gustoInfoCache;
}

function gustoRows(trust: number): DescentSample[] {
  const count = Math.round(8 / STEP);
  const guessUp = 14;
  let tx = Array(count).fill(0);
  let tz = Array(count).fill(guessUp);
  let radius = Math.max(0.75, trust);
  let accepts = 0;
  let rejects = 0;

  // A few GuSTO iterations: linearize about the flown path, hard trust step, merit accept/reject.
  for (let iter = 0; iter < 3; iter += 1) {
    const flown = flyProfile(tx, tz, 12, 30, 0, -2, false);
    const error = flown[flown.length - 1].height;
    // Local pad-aim about the flown states (open-loop). Far from the path this model is wrong.
    const local = retargetAlong(flown);
    const stepped = trustStep(tx, tz, local.tx, local.tz, radius);
    const trial = flyProfile(stepped.tx, stepped.tz, 12, 30, 0, -2, false);
    const trialError = trial[trial.length - 1].height;
    if (Math.abs(trialError) < Math.abs(error) - 0.05) {
      tx = stepped.tx;
      tz = stepped.tz;
      radius = Math.min(10, radius * 1.15);
      accepts += 1;
    } else {
      radius = Math.max(0.75, radius * 0.55);
      rejects += 1;
    }
  }

  const flown = flyProfile(tx, tz, 12, 30, 0, -2, false);
  const guess = flyProfile(Array(count).fill(0), Array(count).fill(guessUp), 12, 30, 0, -2, false);
  gustoInfoCache = {
    error: flown[flown.length - 1].height,
    radius,
    accepts,
    rejects,
  };
  return flown.map((row, i) => {
    const copy = { ...row };
    copy.reference = guess[i].height;
    return copy;
  });
}

/** Open-loop pad-aim using ZEM/ZEV at each flown sample (local model about that path). */
function retargetAlong(flown: DescentSample[]): { tx: number[]; tz: number[] } {
  const tx: number[] = [];
  const tz: number[] = [];
  const n = flown.length - 1;
  for (let i = 0; i < n; i += 1) {
    const row = flown[i];
    const prev = i > 0 ? flown[i - 1] : row;
    const vx = i > 0 ? (row.downrange - prev.downrange) / STEP : 0;
    const left = Math.max(0.45, 8 - row.t);
    let dtx = MASS * guidanceAccel(row.downrange, vx, left);
    let dtz = MASS * (guidanceAccel(row.height, row.rate, left) + G);
    const magnitude = Math.hypot(dtx, dtz) || 1;
    const used = Math.min(TMAX, Math.max(0, magnitude));
    dtx *= used / magnitude;
    dtz *= used / magnitude;
    tx.push(dtx);
    tz.push(dtz);
  }
  return { tx, tz };
}

const REPLAN_TMAX = 40;
const REPLAN_CONE_DEG = 12;
const REPLAN_GUST_AT = 2.2;

function replanRows(gust: number): DescentSample[] {
  // Start inside the 12° cone: wall at 36 m is about 7.7 m.
  let x = 6;
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
    if (!kicked && t >= REPLAN_GUST_AT) {
      vx += gust;
      kicked = true;
    }
    // Live plan: recompute from the state you have now (MPC-style), inside thrust and cone limits.
    const live = replanThrust(x, z, vx, vz, t);
    // Old plan: same limits, but the state never felt the gust.
    const old = replanThrust(oldX, oldZ, oldVx, oldVz, t);
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

/** Pad-aim with a hard thrust ceiling and a hard tilt cone from vertical. */
function replanThrust(
  x: number,
  z: number,
  vx: number,
  vz: number,
  t: number,
): { tx: number; tz: number; mag: number } {
  const left = Math.max(0.4, 8 - t);
  const ax = guidanceAccel(x, vx, left);
  const az = guidanceAccel(z, vz, left);
  let thr = clampThrustCap(MASS * ax, MASS * (az + G), 4, REPLAN_TMAX);
  const cone = (REPLAN_CONE_DEG * Math.PI) / 180;
  const tilt = Math.atan2(Math.abs(thr.tx), Math.max(1e-6, Math.abs(thr.tz)));
  if (tilt > cone && thr.mag > 1e-8) {
    const sign = thr.tx >= 0 ? 1 : -1;
    thr = {
      tx: sign * thr.mag * Math.sin(cone),
      tz: Math.sign(thr.tz || 1) * thr.mag * Math.cos(cone),
      mag: thr.mag,
    };
  }
  return thr;
}

function clampThrustCap(
  tx: number,
  tz: number,
  floor: number,
  ceiling: number,
): { tx: number; tz: number; mag: number } {
  const magnitude = Math.hypot(tx, tz);
  const used = Math.min(ceiling, Math.max(floor, magnitude));
  if (magnitude < 1e-8) {
    return { tx: 0, tz: used, mag: used };
  }
  const scale = used / magnitude;
  return { tx: tx * scale, tz: tz * scale, mag: used };
}

function missRows(fuel: number, cause: number): DescentSample[] {
  const exhaust = 280;
  const infeasible = cause >= 0.5;
  const ceiling = infeasible ? MISS_HARD_CEILING : TMAX;
  let z = 45;
  let vz = -3;
  // Infeasible mode: plenty of fuel; the ceiling alone forbids a soft landing.
  let left = infeasible ? 2 : fuel;
  let hit = false;
  const rows: DescentSample[] = [];
  for (let t = 0; t <= 8 + 1e-9; t += STEP) {
    let thrust = 0;
    if (!hit && left > 0) {
      const leftTime = Math.max(0.4, 8 - t);
      const az = guidanceAccel(z, vz, leftTime);
      thrust = Math.min(ceiling, Math.max(0, MASS * (az + G)));
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
      if (z <= 0) {
        // Freeze height and keep the impact rate so soft vs miss is readable.
        z = 0;
        hit = true;
      }
    }
  }
  return rows;
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

function flyProfile(
  tx: number[],
  tz: number[],
  x0: number,
  z0: number,
  vx0: number,
  vz0: number,
  clampGround = true,
): DescentSample[] {
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
    if (clampGround) {
      const held = holdPad(z, vz);
      z = held.height;
      vz = held.rate;
    }
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
