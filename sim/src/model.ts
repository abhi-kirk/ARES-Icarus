// One idea: height is the state, its slope is the rate.
// h = 4 sin(t), so the rate is 4 cos(t). The bead turns around
// when the rate crosses zero.

export const T_END = 2 * Math.PI;
export const SAMPLE_DT = 0.05;

export type Sample = { t: number; height: number; rate: number };
export type Column = "height" | "rate";

export function height(t: number): number {
  return 4 * Math.sin(t);
}

export function rate(t: number): number {
  return 4 * Math.cos(t);
}

export function samples(): Sample[] {
  const rows: Sample[] = [];
  for (let t = 0; t <= T_END + 1e-9; t += SAMPLE_DT) {
    rows.push({ t, height: height(t), rate: rate(t) });
  }
  return rows;
}

export const BUFFER = samples();
