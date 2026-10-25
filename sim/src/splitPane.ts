const KEY = "icarus-reading-split";
const MIN = 300;
const MAX_RATIO = 0.62;

export function clampSplit(px: number, frameWidth: number): number {
  const max = Math.max(MIN, frameWidth * MAX_RATIO);
  return Math.min(Math.max(px, MIN), max);
}

export function defaultSplit(frameWidth: number): number {
  return clampSplit(Math.min(640, frameWidth * 0.4), frameWidth);
}

export function loadSplit(): number | null {
  try {
    const raw = sessionStorage.getItem(KEY);
    if (!raw) return null;
    const value = Number(raw);
    if (!Number.isFinite(value)) return null;
    return value;
  } catch {
    return null;
  }
}

export function saveSplit(px: number): void {
  try {
    sessionStorage.setItem(KEY, String(Math.round(px)));
  } catch {
    // ignore
  }
}
