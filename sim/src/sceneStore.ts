import { useSyncExternalStore } from "react";
import { T_END } from "./model";

let time = 0.4;
const listeners = new Set<() => void>();

export function getTime(): number {
  return time;
}

export function setTime(next: number): void {
  const wrapped = ((next % T_END) + T_END) % T_END;
  if (Math.abs(wrapped - time) < 1e-6) return;
  time = wrapped;
  listeners.forEach((listener) => listener());
}

export function useTime(): number {
  return useSyncExternalStore(
    (listener) => {
      listeners.add(listener);
      return () => listeners.delete(listener);
    },
    getTime,
  );
}
