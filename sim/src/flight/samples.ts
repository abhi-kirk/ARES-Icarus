export type Sample = {
  t: number;
  x: number;
  y: number;
  z: number;
  vx: number;
  vy: number;
  vz: number;
  fuel: number;
  /** Air density at this altitude, kg/m³. Absent on recordings from before the field. */
  rho?: number;
  fx: number;
  fy: number;
  fz: number;
  padX: number;
  padY: number;
  roll: number;
  pitch: number;
  yaw: number;
  /** Body-to-inertial quaternion, scalar first. Present on the coast recording. */
  qw?: number;
  qx?: number;
  qy?: number;
  qz?: number;
  gimbalX: number;
  gimbalY: number;
  /** MPC predicted position at this sample, when the QP flew. */
  planX?: number;
  planY?: number;
  planZ?: number;
  /** Remaining predicted path as flat [x,y,z, x,y,z, ...]. */
  planTrail?: number[];
};

function num(value: unknown): number | null {
  if (typeof value !== "number" || !Number.isFinite(value)) return null;
  return value;
}

// Matches engine/src/dynamics/air.h. Older recordings omit rho, so the
// page fills Earth density from altitude. A Moon sample from the plant
// carries rho 0 and this fallback is not used.
const SEA_LEVEL_DENSITY = 1.225;
const AIR_SCALE_HEIGHT_M = 8500;

export function earthAirDensity(altitude: number): number {
  const height = altitude > 0 ? altitude : 0;
  return SEA_LEVEL_DENSITY * Math.exp(-height / AIR_SCALE_HEIGHT_M);
}

function optionalNum(value: unknown): number {
  const parsed = num(value);
  if (parsed === null) return 0;
  return parsed;
}

export function parseSample(line: string): Sample | null {
  const trimmed = line.trim();
  if (!trimmed) return null;
  let raw: Record<string, unknown>;
  try {
    raw = JSON.parse(trimmed) as Record<string, unknown>;
  } catch {
    return null;
  }
  const t = num(raw.t);
  const x = num(raw.x);
  const y = num(raw.y);
  const z = num(raw.z);
  const vx = num(raw.vx);
  const vy = num(raw.vy);
  const vz = num(raw.vz);
  const fuel = num(raw.fuel);
  const fx = num(raw.fx);
  const fy = num(raw.fy);
  const fz = num(raw.fz);
  const padX = num(raw.padX);
  const padY = num(raw.padY);
  if (
    t === null ||
    x === null ||
    y === null ||
    z === null ||
    vx === null ||
    vy === null ||
    vz === null ||
    fuel === null ||
    fx === null ||
    fy === null ||
    fz === null ||
    padX === null ||
    padY === null
  ) {
    return null;
  }
  const planX = num(raw.planX);
  const planY = num(raw.planY);
  const planZ = num(raw.planZ);
  let planTrail: number[] | undefined;
  if (Array.isArray(raw.planTrail)) {
    const trail: number[] = [];
    for (const value of raw.planTrail) {
      const parsed = num(value);
      if (parsed === null) {
        trail.length = 0;
        break;
      }
      trail.push(parsed);
    }
    if (trail.length >= 3 && trail.length % 3 === 0) planTrail = trail;
  }
  return {
    t,
    x,
    y,
    z,
    vx,
    vy,
    vz,
    fuel,
    rho: num(raw.rho) ?? undefined,
    fx,
    fy,
    fz,
    padX,
    padY,
    roll: optionalNum(raw.roll),
    pitch: optionalNum(raw.pitch),
    yaw: optionalNum(raw.yaw),
    qw: num(raw.qw) ?? undefined,
    qx: num(raw.qx) ?? undefined,
    qy: num(raw.qy) ?? undefined,
    qz: num(raw.qz) ?? undefined,
    gimbalX: optionalNum(raw.gimbalX),
    gimbalY: optionalNum(raw.gimbalY),
    planX: planX ?? undefined,
    planY: planY ?? undefined,
    planZ: planZ ?? undefined,
    planTrail,
  };
}

export function parseLines(text: string): Sample[] {
  const samples: Sample[] = [];
  for (const line of text.split("\n")) {
    const sample = parseSample(line);
    if (sample) samples.push(sample);
  }
  return samples;
}

function mix(a: number, b: number, u: number): number {
  return a + (b - a) * u;
}

// Shortest-path blend. The entry flip swings the nose through a large angle,
// and a raw component blend the other way would twist the ship.
function mixAttitude(
  left: Sample,
  right: Sample,
  u: number,
): { qw: number; qx: number; qy: number; qz: number } | undefined {
  if (
    left.qw === undefined ||
    left.qx === undefined ||
    left.qy === undefined ||
    left.qz === undefined ||
    right.qw === undefined ||
    right.qx === undefined ||
    right.qy === undefined ||
    right.qz === undefined
  ) {
    return undefined;
  }
  let rw = right.qw;
  let rx = right.qx;
  let ry = right.qy;
  let rz = right.qz;
  const dot = left.qw * rw + left.qx * rx + left.qy * ry + left.qz * rz;
  if (dot < 0) {
    rw = -rw;
    rx = -rx;
    ry = -ry;
    rz = -rz;
  }
  return {
    qw: mix(left.qw, rw, u),
    qx: mix(left.qx, rx, u),
    qy: mix(left.qy, ry, u),
    qz: mix(left.qz, rz, u),
  };
}

export function sampleAt(samples: Sample[], time: number): Sample | null {
  if (samples.length === 0) return null;
  if (time <= samples[0].t) return samples[0];
  const last = samples[samples.length - 1];
  if (time >= last.t) return last;

  let lo = 0;
  let hi = samples.length - 1;
  while (hi - lo > 1) {
    const mid = Math.floor((lo + hi) / 2);
    if (samples[mid].t <= time) lo = mid;
    else hi = mid;
  }

  const left = samples[lo];
  const right = samples[hi];
  const span = right.t - left.t;
  const u = span > 0 ? (time - left.t) / span : 0;
  const attitude = mixAttitude(left, right, u);
  return {
    t: time,
    x: mix(left.x, right.x, u),
    y: mix(left.y, right.y, u),
    z: mix(left.z, right.z, u),
    vx: mix(left.vx, right.vx, u),
    vy: mix(left.vy, right.vy, u),
    vz: mix(left.vz, right.vz, u),
    fuel: mix(left.fuel, right.fuel, u),
    rho: left.rho === undefined || right.rho === undefined
      ? left.rho ?? right.rho
      : mix(left.rho, right.rho, u),
    fx: mix(left.fx, right.fx, u),
    fy: mix(left.fy, right.fy, u),
    fz: mix(left.fz, right.fz, u),
    padX: mix(left.padX, right.padX, u),
    padY: mix(left.padY, right.padY, u),
    roll: mix(left.roll, right.roll, u),
    pitch: mix(left.pitch, right.pitch, u),
    yaw: mix(left.yaw, right.yaw, u),
    qw: attitude?.qw,
    qx: attitude?.qx,
    qy: attitude?.qy,
    qz: attitude?.qz,
    gimbalX: mix(left.gimbalX, right.gimbalX, u),
    gimbalY: mix(left.gimbalY, right.gimbalY, u),
  };
}

export function endTime(samples: Sample[]): number {
  if (samples.length === 0) return 0;
  return samples[samples.length - 1].t;
}
