// The propellant. Four scenes, one exhaust speed.
// Specific impulse is a burn time: ve = Isp * g0, with g0 the standard gravity.
// The burn in The state stays the short version. This chapter is the longer one.

export type PropellantId = "propEq" | "propNozzle" | "propStage" | "propLoss";

export type PropKnobs = {
  ratio: number;
  isp: number;
  expand: number;
  split: number;
  structure: number;
  throttle: number;
};

export type PropSample = {
  t: number;
  // Distance the pictured vehicle has traveled, or altitude on the pad scenes.
  height: number;
  // Staged speed, or the speed of the single vehicle.
  rate: number;
  accel: number;
  mass: number;
  thrust: number;
  // Ideal free-space speed, or the single-stage speed on the staging scene.
  ideal: number;
  isp: number;
  // (pe - pa) * Ae. Negative when the bell is too wide for the air around it.
  push: number;
  flame: number;
  // 0 while a booster is attached, then 0 to 1 as it peels away.
  drift: number;
  // Speed the dropped booster had when it left. The picture parks it there.
  drop: number;
  // Distance of the vehicle that keeps its empty structure.
  companion: number;
  // Bell width relative to the default expansion.
  flare: number;
  // True once the ideal nozzle term is past the separation rule of thumb.
  separated: number;
};

// Standard gravity in the definition of specific impulse. Not the local g.
const G0 = 9.80665;
// Local gravity on the vertical climb. The design uses 9.81.
const G = 9.81;

const EQ_MASS = 4000;
const EQ_BURN = 10;

const CHAMBER_P = 9.7e6;
const CHAMBER_T = 3600;
const GAS_R = 380;
const GAMMA = 1.22;
const MDOT = 25;
const SEA_P = 101325;
const SCALE_H = 8500;
const CLIMB = 60000;
const NOZZLE_TIME = 12;
// Summerfield's rule of thumb: the plume separates when exit pressure
// falls below about 0.4 of the air outside. The trace stays on the ideal term.
const SEPARATION = 0.4;

const PAYLOAD = 1000;
const PROPELLANT = 8000;
const STAGE_TIME = 12;
// The chapter's shared exhaust speed: 300 s * g0.
const SHARED_ISP = 300;

const LOSS_TIME = 240;
const FULL_THRUST = 80000;

export function propDuration(id: PropellantId): number {
  if (id === "propEq") return EQ_BURN;
  if (id === "propNozzle") return NOZZLE_TIME;
  if (id === "propStage") return STAGE_TIME;
  return LOSS_TIME;
}

export function sharedExhaust(): number {
  return SHARED_ISP * G0;
}

function exhaust(isp: number): number {
  return isp * G0;
}

// --- rocket equation -------------------------------------------------------

function eqMass(ratio: number): { mf: number; mdot: number } {
  const mf = EQ_MASS / ratio;
  const mdot = (EQ_MASS - mf) / EQ_BURN;
  return { mf, mdot };
}

// Closed form with gravity off: x = (ve / mdot) * (m0 - m - m ln(m0/m)).
function eqDistance(ve: number, mdot: number, mass: number): number {
  const ratioNow = EQ_MASS / mass;
  return (ve / mdot) * (EQ_MASS - mass - mass * Math.log(ratioNow));
}

export function eqPoint(t: number, knobs: PropKnobs): PropSample {
  const ve = exhaust(knobs.isp);
  const burned = Math.min(Math.max(t, 0), EQ_BURN);
  const { mdot } = eqMass(knobs.ratio);
  const mass = EQ_MASS - mdot * burned;
  const speed = ve * Math.log(EQ_MASS / mass);
  const accel = (ve * mdot) / mass;
  const height = eqDistance(ve, mdot, mass);
  return sample(burned, {
    height,
    rate: speed,
    accel,
    mass,
    thrust: ve * mdot,
    ideal: speed,
    flame: burned < EQ_BURN ? 1 : 0,
  });
}

export function eqBurnout(knobs: PropKnobs): { speed: number; accel0: number; accel1: number; mf: number; ve: number } {
  const ve = exhaust(knobs.isp);
  const { mf, mdot } = eqMass(knobs.ratio);
  return {
    speed: ve * Math.log(knobs.ratio),
    accel0: (ve * mdot) / EQ_MASS,
    accel1: (ve * mdot) / mf,
    mf,
    ve,
  };
}

// --- nozzle ----------------------------------------------------------------

function areaRatio(mach: number): number {
  const gp1 = GAMMA + 1;
  const gm1 = GAMMA - 1;
  const inside = (2 / gp1) * (1 + 0.5 * gm1 * mach * mach);
  const power = gp1 / (2 * gm1);
  return (1 / mach) * Math.pow(inside, power);
}

function machForArea(epsilon: number): number {
  let lo = 1.01;
  let hi = 2;
  while (areaRatio(hi) < epsilon && hi < 30) hi *= 1.4;
  for (let step = 0; step < 60; step += 1) {
    const mid = 0.5 * (lo + hi);
    if (areaRatio(mid) < epsilon) lo = mid;
    else hi = mid;
  }
  return 0.5 * (lo + hi);
}

function starVelocity(): number {
  const power = (GAMMA + 1) / (2 * (GAMMA - 1));
  return Math.sqrt((GAS_R * CHAMBER_T) / GAMMA) * Math.pow((GAMMA + 1) / 2, power);
}

export type NozzleFlow = {
  mach: number;
  pe: number;
  ve: number;
  area: number;
  seaIsp: number;
  vacIsp: number;
  matchKm: number;
  separatesAtSea: boolean;
};

let flowCache: { epsilon: number; flow: NozzleFlow } | null = null;

export function nozzleFlow(epsilon: number): NozzleFlow {
  if (flowCache && flowCache.epsilon === epsilon) return flowCache.flow;
  const mach = machForArea(epsilon);
  const gm1 = GAMMA - 1;
  const pressureRatio = Math.pow(1 + 0.5 * gm1 * mach * mach, -GAMMA / gm1);
  const pe = CHAMBER_P * pressureRatio;
  const ve = Math.sqrt(((2 * GAMMA) / gm1) * GAS_R * CHAMBER_T * (1 - Math.pow(pressureRatio, gm1 / GAMMA)));
  const throat = (MDOT * starVelocity()) / CHAMBER_P;
  const area = epsilon * throat;
  const seaPush = (pe - SEA_P) * area;
  const vacPush = pe * area;
  const seaIsp = (MDOT * ve + seaPush) / (MDOT * G0);
  const vacIsp = (MDOT * ve + vacPush) / (MDOT * G0);
  // pe = pa at the matched altitude. Above the air column, there is no match.
  const matchKm = pe < SEA_P ? (-SCALE_H * Math.log(pe / SEA_P)) / 1000 : 0;
  const flow = {
    mach,
    pe,
    ve,
    area,
    seaIsp,
    vacIsp,
    matchKm,
    separatesAtSea: pe < SEPARATION * SEA_P,
  };
  flowCache = { epsilon, flow };
  return flow;
}

function nozzlePoint(t: number, knobs: PropKnobs): PropSample {
  const burned = Math.min(Math.max(t, 0), NOZZLE_TIME);
  const height = CLIMB * (burned / NOZZLE_TIME);
  const air = SEA_P * Math.exp(-height / SCALE_H);
  const flow = nozzleFlow(knobs.expand);
  const push = (flow.pe - air) * flow.area;
  const thrust = MDOT * flow.ve + push;
  const isp = thrust / (MDOT * G0);
  const separated = flow.pe < SEPARATION * air ? 1 : 0;
  return sample(burned, {
    height,
    rate: isp,
    accel: 0,
    mass: MDOT,
    thrust,
    ideal: flow.vacIsp,
    isp,
    push,
    flame: 0.7,
    flare: Math.sqrt(knobs.expand / 16),
    separated,
  });
}

// --- two stages ------------------------------------------------------------

type StageBuild = {
  prop1: number;
  prop2: number;
  dry1: number;
  dry2: number;
  m0: number;
  afterFirst: number;
  upper0: number;
  upperF: number;
  single0: number;
  singleF: number;
};

function stageBuild(split: number, structure: number): StageBuild {
  const prop1 = split * PROPELLANT;
  const prop2 = PROPELLANT - prop1;
  const dry1 = structure * prop1;
  const dry2 = structure * prop2;
  const singleDry = structure * PROPELLANT;
  return {
    prop1,
    prop2,
    dry1,
    dry2,
    m0: PAYLOAD + PROPELLANT + dry1 + dry2,
    afterFirst: PAYLOAD + prop2 + dry1 + dry2,
    upper0: PAYLOAD + prop2 + dry2,
    upperF: PAYLOAD + dry2,
    single0: PAYLOAD + PROPELLANT + singleDry,
    singleF: PAYLOAD + singleDry,
  };
}

function stageSpeeds(build: StageBuild, ve: number): { first: number; second: number; staged: number; single: number } {
  const first = build.prop1 > 1 ? ve * Math.log(build.m0 / build.afterFirst) : 0;
  const second = build.prop2 > 1 ? ve * Math.log(build.upper0 / build.upperF) : 0;
  const single = ve * Math.log(build.single0 / build.singleF);
  return { first, second, staged: first + second, single };
}

export function stageScore(knobs: PropKnobs): {
  first: number;
  second: number;
  staged: number;
  single: number;
  best: number;
  flat: boolean;
  stageAt: number;
} {
  const ve = sharedExhaust();
  const build = stageBuild(knobs.split, knobs.structure);
  const now = stageSpeeds(build, ve);
  let bestSplit = knobs.split;
  let bestSpeed = now.staged;
  let lowSpeed = now.staged;
  for (let split = 0.15; split <= 0.9 + 1e-9; split += 0.01) {
    const speed = stageSpeeds(stageBuild(split, knobs.structure), ve).staged;
    if (speed > bestSpeed) {
      bestSpeed = speed;
      bestSplit = split;
    }
    if (speed < lowSpeed) lowSpeed = speed;
  }
  // With no structure to drop, every split ends at the same speed.
  if (bestSpeed - lowSpeed < 1) bestSplit = knobs.split;
  return {
    first: now.first,
    second: now.second,
    staged: now.staged,
    single: now.single,
    best: bestSplit,
    flat: bestSpeed - lowSpeed < 1,
    stageAt: STAGE_TIME * knobs.split,
  };
}

function stagePoint(t: number, knobs: PropKnobs): PropSample {
  const ve = sharedExhaust();
  const build = stageBuild(knobs.split, knobs.structure);
  const clock = Math.min(Math.max(t, 0), STAGE_TIME);
  const stageAt = STAGE_TIME * knobs.split;
  const speeds = stageSpeeds(build, ve);

  let staged = 0;
  let single = 0;
  let mass = build.m0;
  let flame = 1;
  if (clock <= stageAt || stageAt <= 1e-6) {
    const frac = stageAt <= 1e-6 ? 1 : clock / stageAt;
    const burned = build.prop1 * Math.min(frac, 1);
    mass = build.m0 - burned;
    staged = build.prop1 > 1 ? ve * Math.log(build.m0 / mass) : 0;
  } else {
    const frac = (clock - stageAt) / Math.max(STAGE_TIME - stageAt, 1e-6);
    const burned = build.prop2 * Math.min(frac, 1);
    mass = build.upper0 - burned;
    const second = build.prop2 > 1 ? ve * Math.log(build.upper0 / Math.max(mass, build.upperF)) : 0;
    staged = speeds.first + second;
  }
  if (clock >= STAGE_TIME - 1e-6) {
    staged = speeds.staged;
    flame = 0;
  }

  const singleFrac = clock / STAGE_TIME;
  const singleMass = build.single0 - (build.single0 - build.singleF) * singleFrac;
  single = ve * Math.log(build.single0 / singleMass);

  // Distance is the integral of the speed so far. Speed grows as the log, so
  // a short step here is enough for the picture.
  const steps = 40;
  let stagedDistance = 0;
  let singleDistance = 0;
  for (let i = 1; i <= steps; i += 1) {
    const earlier = stagePointSpeeds(clock * ((i - 0.5) / steps), knobs, build, ve, speeds.first);
    stagedDistance += earlier.staged * (clock / steps);
    singleDistance += earlier.single * (clock / steps);
  }

  const peel = clock <= stageAt ? 0 : Math.min(1, (clock - stageAt) / 1.5);
  const dropSpeed = clock <= stageAt ? staged : speeds.first;
  return sample(clock, {
    height: stagedDistance,
    rate: staged,
    accel: 0,
    mass,
    thrust: flame > 0 ? ve * (PROPELLANT / STAGE_TIME) : 0,
    ideal: single,
    flame,
    drift: peel,
    drop: dropSpeed,
    companion: singleDistance,
  });
}

function stagePointSpeeds(
  time: number,
  knobs: PropKnobs,
  build: StageBuild,
  ve: number,
  firstSpeed: number,
): { staged: number; single: number; time: number } {
  const stageAt = STAGE_TIME * knobs.split;
  const clock = Math.min(Math.max(time, 0), STAGE_TIME);
  let staged = 0;
  if (clock <= stageAt || stageAt <= 1e-6) {
    const frac = stageAt <= 1e-6 ? 1 : clock / stageAt;
    const mass = build.m0 - build.prop1 * Math.min(frac, 1);
    staged = build.prop1 > 1 ? ve * Math.log(build.m0 / mass) : 0;
  } else {
    const frac = (clock - stageAt) / Math.max(STAGE_TIME - stageAt, 1e-6);
    const mass = build.upper0 - build.prop2 * Math.min(frac, 1);
    const second = build.prop2 > 1 ? ve * Math.log(build.upper0 / Math.max(mass, build.upperF)) : 0;
    staged = firstSpeed + second;
  }
  const singleMass = build.single0 - (build.single0 - build.singleF) * (clock / STAGE_TIME);
  const single = ve * Math.log(build.single0 / singleMass);
  return { staged, single, time: clock };
}

// --- gravity loss ----------------------------------------------------------

export function lossScore(knobs: PropKnobs): {
  speed: number;
  ideal: number;
  loss: number;
  burnout: number;
  liftoff: number;
  onPad: boolean;
} {
  const ve = sharedExhaust();
  const thrust = knobs.throttle * FULL_THRUST;
  const mdot = thrust / ve;
  const mf = EQ_MASS / 4;
  const burnout = (EQ_MASS - mf) / mdot;
  const ideal = ve * Math.log(EQ_MASS / mf);
  const liftoffMass = thrust / G;
  // Heavier than liftoffMass, the vehicle stays on the pad and the mass falls.
  const liftoff = liftoffMass >= EQ_MASS ? 0 : (EQ_MASS - liftoffMass) / mdot;
  const stillDown = liftoff >= LOSS_TIME || liftoff >= burnout;
  if (stillDown) {
    return { speed: 0, ideal, loss: 0, burnout, liftoff, onPad: true };
  }
  const massAtLiftoff = liftoff <= 0 ? EQ_MASS : EQ_MASS - mdot * liftoff;
  const flew = burnout - liftoff;
  const gained = ve * Math.log(massAtLiftoff / mf);
  const finish = gained - G * flew;
  return {
    speed: finish,
    ideal,
    loss: ideal - finish,
    burnout,
    liftoff,
    onPad: false,
  };
}

function lossPoint(t: number, knobs: PropKnobs): PropSample {
  const ve = sharedExhaust();
  const thrustNow = knobs.throttle * FULL_THRUST;
  const mdot = thrustNow / ve;
  const fuel = EQ_MASS * 0.75;
  const burnout = fuel / mdot;
  const clock = Math.min(Math.max(t, 0), LOSS_TIME);
  const burning = clock < burnout;
  const burned = mdot * Math.min(clock, burnout);
  const mass = EQ_MASS - burned;
  const ideal = ve * Math.log(EQ_MASS / mass);
  const liftoffMass = thrustNow / G;
  const liftoff = liftoffMass >= EQ_MASS ? 0 : (EQ_MASS - liftoffMass) / mdot;

  let rate = 0;
  let height = 0;
  let onPad = true;
  if (clock > liftoff && liftoff < burnout) {
    onPad = false;
    const fly = Math.min(clock, burnout) - liftoff;
    const massThen = EQ_MASS - mdot * Math.min(clock, burnout);
    const massLift = EQ_MASS - mdot * liftoff;
    rate = ve * Math.log(massLift / massThen) - G * fly;
    // Coast after the tank is empty: the burn is over, gravity remains.
    if (clock > burnout) rate -= G * (clock - burnout);
    height = lossHeight(clock, liftoff, burnout, ve, mdot, massLift);
    if (height <= 0 && rate < 0) {
      height = 0;
      rate = 0;
      onPad = true;
    }
  }

  return sample(clock, {
    height,
    rate: onPad ? 0 : rate,
    accel: onPad ? 0 : (burning ? thrustNow / mass : 0) - G,
    mass,
    thrust: burning ? thrustNow : 0,
    ideal,
    flame: burning ? (onPad ? 0.35 : knobs.throttle) : 0,
  });
}

// Altitude from the exact speed, stepped. The speed itself stays closed form.
function lossHeight(clock: number, liftoff: number, burnout: number, ve: number, mdot: number, massLift: number): number {
  const end = Math.min(clock, LOSS_TIME);
  if (end <= liftoff) return 0;
  const step = 0.25;
  let height = 0;
  let left = liftoff;
  while (left < end - 1e-9) {
    const right = Math.min(end, left + step);
    const mid = 0.5 * (left + right);
    let speed: number;
    if (mid <= burnout) {
      const massThen = EQ_MASS - mdot * mid;
      speed = ve * Math.log(massLift / massThen) - G * (mid - liftoff);
    } else {
      const massEnd = EQ_MASS - mdot * burnout;
      const atBurn = ve * Math.log(massLift / massEnd) - G * (burnout - liftoff);
      speed = atBurn - G * (mid - burnout);
    }
    height += speed * (right - left);
    left = right;
  }
  return height;
}

function sample(t: number, patch: Partial<PropSample>): PropSample {
  return {
    t,
    height: 0,
    rate: 0,
    accel: 0,
    mass: 0,
    thrust: 0,
    ideal: 0,
    isp: 0,
    push: 0,
    flame: 0,
    drift: 0,
    drop: 0,
    companion: 0,
    flare: 1,
    separated: 0,
    ...patch,
  };
}

let rowCache: { key: string; rows: PropSample[] } | null = null;

function cacheKey(id: PropellantId, knobs: PropKnobs): string {
  return `${id}|${knobs.ratio}|${knobs.isp}|${knobs.expand}|${knobs.split}|${knobs.structure}|${knobs.throttle}`;
}

export function propRows(id: PropellantId, knobs: PropKnobs): PropSample[] {
  const key = cacheKey(id, knobs);
  if (rowCache && rowCache.key === key) return rowCache.rows;
  const rows: PropSample[] = [];
  const end = propDuration(id);
  const step = id === "propLoss" ? 0.25 : 0.05;
  for (let t = 0; t <= end + 1e-9; t += step) {
    rows.push(propPoint(id, t, knobs));
  }
  if (id === "propLoss") {
    const score = lossScore(knobs);
    const marks = [score.liftoff, score.burnout];
    for (const mark of marks) {
      if (mark <= 0 || mark >= end) continue;
      const already = rows.some((row) => Math.abs(row.t - mark) < 1e-6);
      if (!already) rows.push(propPoint(id, mark, knobs));
    }
    rows.sort((a, b) => a.t - b.t);
  }
  rowCache = { key, rows };
  return rows;
}

function propPoint(id: PropellantId, t: number, knobs: PropKnobs): PropSample {
  if (id === "propEq") return eqPoint(t, knobs);
  if (id === "propNozzle") return nozzlePoint(t, knobs);
  if (id === "propStage") return stagePoint(t, knobs);
  return lossPoint(t, knobs);
}

export function propAt(id: PropellantId, t: number, knobs: PropKnobs): PropSample {
  if (id === "propStage") {
    const rows = propRows(id, knobs);
    let chosen = rows[0];
    for (const row of rows) {
      if (row.t <= t + 1e-9) chosen = row;
      else break;
    }
    return chosen;
  }
  return propPoint(id, t, knobs);
}

export function propMarkers(id: PropellantId, knobs: PropKnobs): { t: number; label: string }[] {
  if (id === "propEq") return [{ t: EQ_BURN, label: "burnout" }];
  if (id === "propNozzle") {
    const flow = nozzleFlow(knobs.expand);
    if (flow.matchKm <= 0 || flow.matchKm >= CLIMB / 1000) return [];
    return [{ t: NOZZLE_TIME * ((flow.matchKm * 1000) / CLIMB), label: "matched" }];
  }
  if (id === "propStage") return [{ t: STAGE_TIME * knobs.split, label: "stage" }];
  const score = lossScore(knobs);
  const markers: { t: number; label: string }[] = [];
  if (score.liftoff > 0.5 && score.liftoff < LOSS_TIME) markers.push({ t: score.liftoff, label: "liftoff" });
  if (score.burnout < LOSS_TIME) markers.push({ t: score.burnout, label: "burnout" });
  return markers;
}

export function propBrief(id: PropellantId, knobs: PropKnobs): string {
  if (id === "propEq") {
    const end = eqBurnout(knobs);
    return String.raw`The rocket equation. Gravity off, no air. Mass ${EQ_MASS} kg to ${end.mf.toFixed(0)} kg in ${EQ_BURN} s. $I_{sp} = ${knobs.isp.toFixed(0)}\,\mathrm{s}$, $v_e = ${end.ve.toFixed(0)}\,\mathrm{m/s}$, mass ratio ${knobs.ratio.toFixed(2)}. Final speed ${end.speed.toFixed(0)} m/s. Acceleration ${end.accel0.toFixed(0)} m/s² at ignition and ${end.accel1.toFixed(0)} m/s² at burnout. Final speed does not depend on the burn time while gravity is off.`;
  }
  if (id === "propNozzle") {
    const flow = nozzleFlow(knobs.expand);
    return String.raw`The nozzle. Thrust is $\dot{m} v_e + (p_e - p_a) A_e$. Chamber ${CHAMBER_P / 1e6} MPa, ${CHAMBER_T} K, $\gamma = ${GAMMA}$, $\dot{m} = ${MDOT}\,\mathrm{kg/s}$. Expansion $\varepsilon = ${knobs.expand.toFixed(0)}$. Sea-level $I_{sp} = ${flow.seaIsp.toFixed(0)}\,\mathrm{s}$, vacuum $I_{sp} = ${flow.vacIsp.toFixed(0)}\,\mathrm{s}$, matched altitude ${flow.matchKm.toFixed(1)} km. The clock is a prescribed climb from 0 to ${CLIMB / 1000} km in ${NOZZLE_TIME} s. The ideal term stops applying once $p_e < ${SEPARATION}\, p_a$; the trace stays on the ideal term.`;
  }
  if (id === "propStage") {
    const score = stageScore(knobs);
    const finish = score.flat
      ? `With this structure factor every split ends at ${score.staged.toFixed(0)} m/s, the same speed as the single stage.`
      : `The split that finishes fastest near this structure is ${score.best.toFixed(2)}.`;
    return String.raw`Two stages. Payload ${PAYLOAD} kg, propellant ${PROPELLANT} kg, $v_e = ${sharedExhaust().toFixed(0)}\,\mathrm{m/s}$, structure ${knobs.structure.toFixed(2)} kg dry per kg propellant. Split ${knobs.split.toFixed(2)} puts the first burn at ${score.first.toFixed(0)} m/s and the second at ${score.second.toFixed(0)} m/s, total ${score.staged.toFixed(0)} m/s. One stage that keeps the structure ends at ${score.single.toFixed(0)} m/s. ${finish}`;
  }
  const score = lossScore(knobs);
  const where = score.onPad
    ? `still on the pad at ${LOSS_TIME} s because thrust is under the weight`
    : `burnout speed ${score.speed.toFixed(0)} m/s after ${score.burnout.toFixed(0)} s, gravity loss ${score.loss.toFixed(0)} m/s`;
  return String.raw`Gravity loss. Same 4000 kg to 1000 kg tank, ideal ${score.ideal.toFixed(0)} m/s. Vertical, no drag, $g = ${G}\,\mathrm{m/s^2}$. Throttle ${knobs.throttle.toFixed(2)} of ${FULL_THRUST} N. ${where}. The ideal line is $v_e \ln(m_0/m)$ and keeps climbing while fuel burns on the pad. The $gt$ subtraction applies only while flying.`;
}
