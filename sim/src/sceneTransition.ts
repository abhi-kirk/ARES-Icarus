export const SCENE_VEIL_IN_MS = 150;
export const SCENE_VEIL_OUT_MS = 200;

export function sceneMotionOk(): boolean {
  if (typeof window === "undefined") return true;
  return !window.matchMedia("(prefers-reduced-motion: reduce)").matches;
}
