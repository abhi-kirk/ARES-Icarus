import type { Sample } from "./samples";

/** Same gates as the plant: farther than this is a miss, faster than this is a hard hit. */
export const MISS_M = 25;
export const HARD_VZ = 2;

export type TouchKind = "place" | "hard";

/** The first moment the ship is on the ground, when that landing is not soft. */
export type Touch = {
  kind: TouchKind;
  t: number;
  x: number;
  y: number;
  vx: number;
  vy: number;
  vz: number;
  miss: number;
};

export function touchOf(samples: Sample[]): Touch | null {
  let hit: Sample | null = null;
  for (const sample of samples) {
    if (sample.z <= 0.5) {
      hit = sample;
      break;
    }
  }
  if (!hit) return null;

  const miss = Math.hypot(hit.x - hit.padX, hit.y - hit.padY);
  const speed = Math.abs(hit.vz);
  if (speed <= HARD_VZ && miss <= MISS_M) return null;

  const kind: TouchKind = speed > HARD_VZ ? "hard" : "place";
  return {
    kind,
    t: hit.t,
    x: hit.x,
    y: hit.y,
    vx: hit.vx,
    vy: hit.vy,
    vz: hit.vz,
    miss,
  };
}
