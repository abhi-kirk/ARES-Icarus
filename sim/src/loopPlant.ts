import type { Knobs } from "./scenes";

/** Fixed-mass hover rocket for The loop chapter. Height is meters above the pad; thrust in newtons. */
export const LOOP_M = 2.5;
export const LOOP_G = 9.81;
export const LOOP_T0 = LOOP_M * LOOP_G;
export const LOOP_LIMIT = 80;

export type LoopRow = {
  t: number;
  height: number;
  rate: number;
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
  actual: number;
  command: number;
};

function blank(t: number): LoopRow {
  return {
    t,
    height: 0,
    rate: 0,
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
    thrust: LOOP_T0,
    actual: LOOP_T0,
    command: LOOP_T0,
  };
}

function clampThrust(value: number): number {
  return Math.min(LOOP_LIMIT, Math.max(0, value));
}

function integrateOpen(thrustExtra: number, end: number, step: number): LoopRow[] {
  const rows: LoopRow[] = [];
  let h = 0;
  let v = 0;
  for (let t = 0; t <= end + 1e-9; t += step) {
    const ref = 0;
    const thrust = t >= 1 ? LOOP_T0 + thrustExtra : LOOP_T0;
    const accel = thrust / LOOP_M - LOOP_G;
    const row = blank(t);
    row.reference = ref;
    row.height = h;
    row.rate = v;
    row.thrust = thrust;
    row.actual = thrust;
    row.command = thrust;
    rows.push(row);
    v += accel * step;
    h += v * step;
  }
  return rows;
}

function refStep(t: number, at: number, size: number): number {
  return t >= at ? size : 0;
}

function simulatePid(
  kind: "p" | "i" | "d",
  end: number,
  step: number,
  gains: { kp: number; ki: number; kd: number },
): LoopRow[] {
  const rows: LoopRow[] = [];
  let h = 0;
  let v = 0;
  let integral = 0;
  for (let t = 0; t <= end + 1e-9; t += step) {
    const ref = refStep(t, 2, 2);
    const error = ref - h;
    integral += error * step;
    let thrust = LOOP_T0;
    if (kind === "p") thrust += gains.kp * error;
    if (kind === "i") thrust += gains.ki * integral;
    if (kind === "d") thrust += -gains.kd * v;
    thrust = clampThrust(thrust);
    const accel = thrust / LOOP_M - LOOP_G;
    const row = blank(t);
    row.reference = ref;
    row.height = h;
    row.rate = v;
    row.thrust = thrust;
    rows.push(row);
    v += accel * step;
    h += v * step;
  }
  return rows;
}

function mergeErrorRows(p: LoopRow[], i: LoopRow[], d: LoopRow[]): LoopRow[] {
  return p.map((row, index) => {
    const merged = { ...row };
    merged.pHeight = p[index].height;
    merged.iHeight = i[index].height;
    merged.dHeight = d[index].height;
    merged.reference = row.reference;
    return merged;
  });
}

function simulateEquilibrium(perturb: number, end: number, step: number): LoopRow[] {
  const rows: LoopRow[] = [];
  let hN = perturb;
  let vN = 0;
  let hL = perturb;
  let vL = 0;
  const kp = 14;
  const linearAccel = -kp / LOOP_M;
  for (let t = 0; t <= end + 1e-9; t += step) {
    const ref = 0;
    const errorN = ref - hN;
    const thrustN = clampThrust(LOOP_T0 + kp * errorN);
    const accelN = thrustN / LOOP_M - LOOP_G;
    const accelL = linearAccel * (ref - hL);

    const row = blank(t);
    row.nonlinear = hN;
    row.linear = hL;
    row.height = hN;
    row.rate = vN;
    rows.push(row);

    vN += accelN * step;
    hN += vN * step;
    vL += accelL * step;
    hL += vL * step;
  }
  return rows;
}

function simulateHover(unstableGain: number, end: number, step: number): LoopRow[] {
  const rows: LoopRow[] = [];
  let hOpen = 0.05;
  let vOpen = 0;
  let hClosed = 0.05;
  let vClosed = 0;
  const kp = 22;
  for (let t = 0; t <= end + 1e-9; t += step) {
    const ref = t < 6 ? 0.8 : 0;
    const openAccel = unstableGain * hOpen;
    const error = ref - hClosed;
    const closedAccel = unstableGain * hClosed + (kp * error) / LOOP_M;

    const row = blank(t);
    row.reference = ref;
    row.openHeight = hOpen;
    row.closedHeight = hClosed;
    row.height = hClosed;
    row.rate = vClosed;
    rows.push(row);

    vOpen += openAccel * step;
    hOpen += vOpen * step;
    vClosed += closedAccel * step;
    hClosed += vClosed * step;
  }
  return rows;
}

function simulateHidden(coupling: number, end: number, step: number): LoopRow[] {
  const rows: LoopRow[] = [];
  let h = 0;
  let v = 0;
  let q = 0.02;
  let w = 0;
  const lambda = 2.2;
  const kp = 16;
  for (let t = 0; t <= end + 1e-9; t += step) {
    const ref = refStep(t, 1.5, 1.2);
    const error = ref - h;
    const thrust = clampThrust(LOOP_T0 + kp * error);
    const accel = thrust / LOOP_M - LOOP_G + coupling * q;
    const qAccel = lambda * lambda * q;

    const row = blank(t);
    row.reference = ref;
    row.measured = h;
    row.hiddenMode = q;
    row.height = h;
    row.rate = v;
    rows.push(row);

    v += accel * step;
    h += v * step;
    w += qAccel * step;
    q += w * step;
  }
  return rows;
}

function simulateSample(sampleDt: number, delay: number, end: number, step: number): LoopRow[] {
  const rows: LoopRow[] = [];
  let hC = 0;
  let vC = 0;
  let hD = 0;
  let vD = 0;
  let integralC = 0;
  let integralD = 0;
  let heldThrust = LOOP_T0;
  let nextSample = 0;
  const history: { t: number; h: number }[] = [];
  const kp = 12;
  const ki = 4;

  for (let t = 0; t <= end + 1e-9; t += step) {
    const ref = refStep(t, 1, 1.5);
    history.push({ t, h: hD });

    const errorC = ref - hC;
    integralC += errorC * step;
    const thrustC = clampThrust(LOOP_T0 + kp * errorC + ki * integralC);
    const accelC = thrustC / LOOP_M - LOOP_G;

    let measured = hD;
    const delayedTime = t - delay;
    for (let i = history.length - 1; i >= 0; i -= 1) {
      if (history[i].t <= delayedTime + 1e-9) {
        measured = history[i].h;
        break;
      }
    }

    if (t + 1e-9 >= nextSample) {
      const errorD = ref - measured;
      integralD += errorD * sampleDt;
      heldThrust = clampThrust(LOOP_T0 + kp * errorD + ki * integralD);
      nextSample += sampleDt;
    }
    const accelD = heldThrust / LOOP_M - LOOP_G;

    const row = blank(t);
    row.reference = ref;
    row.continuous = hC;
    row.discrete = hD;
    row.height = hD;
    row.rate = vD;
    row.command = heldThrust;
    row.actual = thrustC;
    rows.push(row);

    vC += accelC * step;
    hC += vC * step;
    vD += accelD * step;
    hD += vD * step;
  }
  return rows;
}

export function loopRowsFor(
  id: "loopOpen" | "loopError" | "loopEquil" | "loopHover" | "loopMode" | "loopSample",
  knobs: Knobs,
): LoopRow[] {
  const step = 0.02;
  if (id === "loopOpen") return integrateOpen(knobs.extraThrust, 10, step);
  if (id === "loopError") {
    const gains = { kp: knobs.kp, ki: knobs.ki, kd: knobs.kd };
    const p = simulatePid("p", 14, step, gains);
    const i = simulatePid("i", 14, step, gains);
    const d = simulatePid("d", 14, step, gains);
    return mergeErrorRows(p, i, d);
  }
  if (id === "loopEquil") return simulateEquilibrium(knobs.perturb, 10, step);
  if (id === "loopHover") return simulateHover(knobs.unstable, 14, step);
  if (id === "loopMode") return simulateHidden(knobs.coupling, 12, step);
  return simulateSample(knobs.dt, knobs.tau, 12, step);
}

export function loopRowAt(rows: LoopRow[], t: number): LoopRow {
  let chosen = rows[0];
  for (const row of rows) {
    if (row.t <= t + 1e-9) chosen = row;
    else break;
  }
  return chosen;
}
