/**
 * Frequency-domain hover. Same rocket as The loop's unstable hover:
 * mass 2.5 kg, open-loop growth μ = 1.5 /s, so μ² = 2.25.
 *
 * Deviation height x is meters from the hover point. The picture adds
 * FREQ_PAD so that point sits above the ground.
 *
 * Open-loop gain is L(s) = C(s) G(s), with
 *   G(s) = 1 / (m (s² − μ²))
 *   C(s) = Kp + Kd s
 * A pure delay multiplies by e^{−sτ}. A zero-order hold multiplies by
 * (1 − e^{−sT}) / (sT). The notch scene multiplies G by a light bend
 * and C by a notch tuned to FREQ_NOTCH_W.
 */

export type FreqId = "freqMargin" | "freqDelay" | "freqNotch";

export type FreqKnobs = {
  gain: number;
  damp: number;
  lag: number;
  sample: number;
  bend: number;
  notch: number;
};

export const FREQ_M = 2.5;
export const FREQ_MU = 1.5;
export const FREQ_MU2 = FREQ_MU * FREQ_MU;
/** Kp below this and the low-frequency loop gain sits under 0 dB. */
export const FREQ_FLOOR = FREQ_M * FREQ_MU2;
export const FREQ_NOTCH_W = 8;
export const FREQ_BEND_ZETA = 0.045;
export const FREQ_NOTCH_ZETA = 0.55;
export const FREQ_TIME = 12;
export const FREQ_RELEASE = 0.35;
export const FREQ_PAD = 1.2;

const W_MIN = 0.25;
const W_MAX = 40;
const W_COUNT = 80;

type Cplx = { re: number; im: number };

function c(re: number, im = 0): Cplx {
  return { re, im };
}

function add(a: Cplx, b: Cplx): Cplx {
  return { re: a.re + b.re, im: a.im + b.im };
}

function scale(a: Cplx, k: number): Cplx {
  return { re: a.re * k, im: a.im * k };
}

function mul(a: Cplx, b: Cplx): Cplx {
  return {
    re: a.re * b.re - a.im * b.im,
    im: a.re * b.im + a.im * b.re,
  };
}

function div(a: Cplx, b: Cplx): Cplx {
  const den = b.re * b.re + b.im * b.im;
  if (den < 1e-18) return c(0, 0);
  return {
    re: (a.re * b.re + a.im * b.im) / den,
    im: (a.im * b.re - a.re * b.im) / den,
  };
}

function hypot(z: Cplx): number {
  return Math.hypot(z.re, z.im);
}

export type BodePoint = {
  w: number;
  magDb: number;
  phaseDeg: number;
};

export type FreqMargins = {
  /** Gain-crossover frequency, rad/s. Null when |L| never crosses 0 dB. */
  wc: number | null;
  /** Phase margin in degrees at the worst gain crossover. */
  pm: number | null;
  /** How far Kp sits above m μ², in dB. Negative means the hover runs away. */
  gmDb: number;
  /** Phase the pure delay subtracts at wc, degrees. */
  delayDeg: number;
  /** Phase the zero-order hold subtracts at wc, degrees. */
  holdDeg: number;
  /** Magnitude at the bend frequency, dB. Scene 3. */
  peakDb: number;
  /** True when the notch is on and the bend sits on the notch frequency. */
  covered: boolean;
};

export type FreqSample = {
  t: number;
  height: number;
  rate: number;
  thrust: number;
  away: number;
};

type LoopOpts = {
  gain: number;
  damp: number;
  lag: number;
  sample: number;
  bend: number;
  notchOn: boolean;
  flex: boolean;
};

function optsFor(id: FreqId, knobs: FreqKnobs): LoopOpts {
  const notchOn = knobs.notch >= 0.5;
  if (id === "freqMargin") {
    return {
      gain: knobs.gain,
      damp: knobs.damp,
      lag: knobs.lag,
      sample: 0,
      bend: knobs.bend,
      notchOn: false,
      flex: false,
    };
  }
  if (id === "freqDelay") {
    return {
      gain: knobs.gain,
      damp: knobs.damp,
      lag: knobs.lag,
      sample: knobs.sample,
      bend: knobs.bend,
      notchOn: false,
      flex: false,
    };
  }
  return {
    gain: knobs.gain,
    damp: knobs.damp,
    lag: 0,
    sample: 0,
    bend: knobs.bend,
    notchOn,
    flex: true,
  };
}

function quadratic(w: number, omega: number, zeta: number): Cplx {
  // s² + 2 ζ ω s + ω² at s = j w.
  const s = c(0, w);
  const s2 = mul(s, s);
  return add(add(s2, scale(s, 2 * zeta * omega)), c(omega * omega));
}

/** L(jw) for the open loop this scene draws. */
function loopAt(w: number, opts: LoopOpts): Cplx {
  const s = c(0, w);
  const s2 = mul(s, s);
  const rigidDen = scale(add(s2, c(-FREQ_MU2)), FREQ_M);
  let plant = div(c(1), rigidDen);

  if (opts.flex) {
    const bendDen = quadratic(w, opts.bend, FREQ_BEND_ZETA);
    plant = mul(plant, div(c(opts.bend * opts.bend), bendDen));
  }

  let control = c(opts.gain, opts.damp * w);
  if (opts.notchOn) {
    const zeros = quadratic(w, FREQ_NOTCH_W, FREQ_BEND_ZETA);
    const poles = quadratic(w, FREQ_NOTCH_W, FREQ_NOTCH_ZETA);
    control = mul(control, div(zeros, poles));
  }

  let loop = mul(control, plant);

  if (opts.lag > 1e-6) {
    const phase = -w * opts.lag;
    loop = mul(loop, c(Math.cos(phase), Math.sin(phase)));
  }

  if (opts.sample > 1e-4) {
    const half = (w * opts.sample) / 2;
    const sinc = Math.abs(half) < 1e-6 ? 1 : Math.sin(half) / half;
    const phase = -half;
    loop = mul(loop, scale(c(Math.cos(phase), Math.sin(phase)), sinc));
  }

  return loop;
}

function frequencies(): number[] {
  const out: number[] = [];
  for (let i = 0; i < W_COUNT; i += 1) {
    const u = i / (W_COUNT - 1);
    out.push(W_MIN * Math.pow(W_MAX / W_MIN, u));
  }
  return out;
}

function unwrap(degrees: number[]): number[] {
  const out = degrees.slice();
  for (let i = 1; i < out.length; i += 1) {
    let jump = out[i] - out[i - 1];
    while (jump > 180) {
      out[i] -= 360;
      jump -= 360;
    }
    while (jump < -180) {
      out[i] += 360;
      jump += 360;
    }
  }
  return out;
}

export type BodeCurve = {
  points: BodePoint[];
  margins: FreqMargins;
};

function analyze(opts: LoopOpts): BodeCurve {
  const ws = frequencies();
  const rawPhase: number[] = [];
  const mags: number[] = [];
  for (const w of ws) {
    const z = loopAt(w, opts);
    const mag = hypot(z);
    mags.push(mag);
    rawPhase.push((Math.atan2(z.im, z.re) * 180) / Math.PI);
  }
  // atan2 reports the negative-real axis as +180°. This loop sits on that
  // axis at low frequency, and delay carries it below −180°. Anchor there
  // so the curve does not flip to the positive side.
  let phase = unwrap(rawPhase);
  if (phase[0] > 0) phase = phase.map((value) => value - 360);
  const points: BodePoint[] = ws.map((w, i) => ({
    w,
    magDb: 20 * Math.log10(Math.max(mags[i], 1e-8)),
    phaseDeg: phase[i],
  }));

  // Worst gain crossover: the one whose phase sits closest to, or past, −180°.
  let wc: number | null = null;
  let pm: number | null = null;
  for (let i = 1; i < points.length; i += 1) {
    const a = points[i - 1];
    const b = points[i];
    const cross = (a.magDb >= 0 && b.magDb < 0) || (a.magDb < 0 && b.magDb >= 0);
    if (!cross) continue;
    const span = a.magDb - b.magDb;
    const t = Math.abs(span) < 1e-9 ? 0 : a.magDb / span;
    const w = a.w * Math.pow(b.w / a.w, t);
    const phaseDeg = a.phaseDeg + (b.phaseDeg - a.phaseDeg) * t;
    let thisPm = phaseDeg + 180;
    if (thisPm > 180) thisPm -= 360;
    if (thisPm < -180) thisPm += 360;
    if (pm === null || thisPm < pm) {
      pm = thisPm;
      wc = w;
    }
  }

  const gmDb = 20 * Math.log10(Math.max(opts.gain, 1e-6) / FREQ_FLOOR);
  let delayDeg = 0;
  let holdDeg = 0;
  if (wc !== null) {
    delayDeg = ((wc * opts.lag) * 180) / Math.PI;
    holdDeg = (((wc * opts.sample) / 2) * 180) / Math.PI;
  }

  const zBend = loopAt(Math.max(opts.bend, W_MIN), opts);
  const peakDb = 20 * Math.log10(Math.max(hypot(zBend), 1e-8));
  const covered = opts.notchOn && Math.abs(opts.bend - FREQ_NOTCH_W) < 0.35;

  return {
    points,
    margins: {
      wc,
      pm,
      gmDb,
      delayDeg,
      holdDeg,
      peakDb,
      covered,
    },
  };
}

const SIM_DT = 0.002;
const ROW_DT = 0.02;
const CAP = 8;

type Hist = { t: number; x: number; v: number };

function delayed(history: Hist[], t: number, lag: number, x: number, v: number): { x: number; v: number } {
  if (lag <= 1e-6 || history.length === 0) return { x, v };
  const want = t - lag;
  let found = history[0];
  for (let i = history.length - 1; i >= 0; i -= 1) {
    if (history[i].t <= want + 1e-9) {
      found = history[i];
      break;
    }
  }
  return { x: found.x, v: found.v };
}

function simulate(opts: LoopOpts): FreqSample[] {
  const rows: FreqSample[] = [];
  let x = FREQ_RELEASE;
  let v = 0;
  let w = 0;
  let wDot = 0;
  let q = 0;
  let qDot = 0;
  let away = 0;

  const history: Hist[] = [];
  const preload = Math.max(opts.lag, SIM_DT);
  for (let t = -preload; t < -1e-12; t += SIM_DT) {
    history.push({ t, x, v });
  }

  let ePrev = -opts.gain * x;
  let held = ePrev;
  let nextSample = 0;
  const useHold = opts.sample > 1e-4;

  const wn = FREQ_NOTCH_W;
  const b = 2 * FREQ_BEND_ZETA * wn;
  const d = 2 * FREQ_NOTCH_ZETA * wn;
  const c0 = wn * wn;

  const steps = Math.round(FREQ_TIME / SIM_DT);
  let nextRow = 0;

  for (let n = 0; n <= steps; n += 1) {
    const t = n * SIM_DT;
    if (t + 1e-9 >= nextRow) {
      rows.push({
        t: nextRow,
        height: x + FREQ_PAD,
        rate: v,
        thrust: held,
        away,
      });
      nextRow += ROW_DT;
    }

    const sense = delayed(history, t, opts.lag, x, v);
    const command = -opts.gain * sense.x - opts.damp * sense.v;
    let u = command;

    if (useHold) {
      if (t + 1e-9 >= nextSample) {
        u = command;
        if (opts.notchOn) u = notchOf(command);
        held = u;
        nextSample += opts.sample;
      } else {
        u = held;
      }
    } else if (opts.notchOn) {
      u = notchOf(command);
      held = u;
    } else {
      held = command;
      u = command;
    }

    let force = u;
    if (opts.flex) {
      const wb = opts.bend;
      const wAcc = -wb * wb * w - 2 * FREQ_BEND_ZETA * wb * wDot + wb * wb * u;
      wDot += wAcc * SIM_DT;
      w += wDot * SIM_DT;
      force = w;
    }

    const accel = FREQ_MU2 * x + force / FREQ_M;
    v += accel * SIM_DT;
    x += v * SIM_DT;

    if (x > CAP) {
      x = CAP;
      v = 0;
      away = 1;
    }
    if (x < -FREQ_PAD) {
      x = -FREQ_PAD;
      v = Math.max(v, 0);
    }

    history.push({ t, x, v });
    const keep = t - opts.lag - 0.05;
    while (history.length > 2 && history[0].t < keep) history.shift();
  }

  return rows;

  function notchOf(command: number): number {
    const edot = (command - ePrev) / SIM_DT;
    ePrev = command;
    const qAcc = (b - d) * edot - d * qDot - c0 * q;
    qDot += qAcc * SIM_DT;
    q += qDot * SIM_DT;
    return q + command;
  }
}

type Bundle = { rows: FreqSample[]; bode: BodeCurve };

let cache: { key: string; bundle: Bundle } | null = null;

function bundleKey(id: FreqId, knobs: FreqKnobs): string {
  return `${id}|${knobs.gain}|${knobs.damp}|${knobs.lag}|${knobs.sample}|${knobs.bend}|${knobs.notch}`;
}

function bundle(id: FreqId, knobs: FreqKnobs): Bundle {
  const key = bundleKey(id, knobs);
  if (cache && cache.key === key) return cache.bundle;
  const opts = optsFor(id, knobs);
  const next = { rows: simulate(opts), bode: analyze(opts) };
  cache = { key, bundle: next };
  return next;
}

export function freqRows(id: FreqId, knobs: FreqKnobs): FreqSample[] {
  return bundle(id, knobs).rows;
}

export function freqBode(id: FreqId, knobs: FreqKnobs): BodeCurve {
  return bundle(id, knobs).bode;
}

export function freqSampleAt(id: FreqId, t: number, knobs: FreqKnobs): FreqSample {
  const rows = freqRows(id, knobs);
  let chosen = rows[0];
  for (const row of rows) {
    if (row.t <= t + 1e-9) chosen = row;
    else break;
  }
  return chosen;
}

export function freqBrief(id: FreqId, knobs: FreqKnobs): string {
  const { margins: m } = freqBode(id, knobs);
  const pm = m.pm === null ? "no crossover" : `${m.pm.toFixed(0)}°`;
  const wc = m.wc === null ? "none" : `${m.wc.toFixed(2)} rad/s`;
  const common = `Hover mass ${FREQ_M} kg, μ = ${FREQ_MU} /s, so m μ² = ${FREQ_FLOOR.toFixed(3)} N/m. Kp = ${knobs.gain.toFixed(1)}, Kd = ${knobs.damp.toFixed(1)}. Gain margin ${m.gmDb.toFixed(1)} dB is how far Kp sits above m μ². Phase margin ${pm} at ${wc}.`;
  if (id === "freqMargin") {
    return `${common} Delay ${knobs.lag.toFixed(2)} s subtracts ${m.delayDeg.toFixed(0)}° at the crossover. The live chart is the Bode plot of L(jω), not a time series. Released from ${FREQ_RELEASE} m above the hover.`;
  }
  if (id === "freqDelay") {
    return `${common} Pure delay ${knobs.lag.toFixed(2)} s subtracts ${m.delayDeg.toFixed(0)}°. Sample time ${knobs.sample.toFixed(2)} s holds thrust and subtracts about ${m.holdDeg.toFixed(0)}°. The live chart is the Bode plot.`;
  }
  const peak = m.covered ? "The notch covers the bend, so the peak is gone." : `Bend at ${knobs.bend.toFixed(1)} rad/s, magnitude ${m.peakDb.toFixed(0)} dB there. Notch is ${knobs.notch >= 0.5 ? "on, tuned to 8 rad/s" : "off"}.`;
  return `${common} ${peak} The bend is a resonance, ω² / (s² + 2 ζ ω s + ω²) with ζ = ${FREQ_BEND_ZETA}. It is not the unstable hidden mode from The loop. The live chart is the Bode plot.`;
}
