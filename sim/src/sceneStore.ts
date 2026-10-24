import { useSyncExternalStore } from "react";
import { T_END } from "./model";

// One clock. The bead, the slider, the chart, and play all use it.
// Play runs at real time and stops at the end, so the trace stays complete.

let time = 0.4;
let playing = false;
let frame = 0;
let lastStamp = 0;

const listeners = new Set<() => void>();
const playListeners = new Set<() => void>();

function publishTime(): void {
  listeners.forEach((listener) => listener());
}

function publishPlay(): void {
  playListeners.forEach((listener) => listener());
}

// The end of the trace is a real time. Wrapping it back to zero would erase the line.
function place(next: number): number {
  if (next >= T_END - 1e-9 && next <= T_END + 1e-6) return T_END;
  const wrapped = ((next % T_END) + T_END) % T_END;
  return wrapped;
}

export function getTime(): number {
  return time;
}

export function isPlaying(): boolean {
  return playing;
}

export function setTime(next: number): void {
  stop();
  const placed = place(next);
  if (Math.abs(placed - time) < 1e-6) return;
  time = placed;
  publishTime();
}

function stop(): void {
  if (!playing) return;
  playing = false;
  cancelAnimationFrame(frame);
  publishPlay();
}

function tick(now: number): void {
  if (!playing) return;

  const elapsed = (now - lastStamp) / 1000;
  lastStamp = now;
  const step = Math.min(0.05, elapsed);
  const next = time + step;

  if (next >= T_END) {
    time = T_END;
    publishTime();
    stop();
    return;
  }

  time = next;
  publishTime();
  frame = requestAnimationFrame(tick);
}

export function play(): void {
  if (playing) return;

  if (time >= T_END - 1e-4) {
    time = 0;
    publishTime();
  }

  playing = true;
  lastStamp = performance.now();
  publishPlay();
  frame = requestAnimationFrame(tick);
}

export function togglePlay(): void {
  if (playing) stop();
  else play();
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

export function usePlaying(): boolean {
  return useSyncExternalStore(
    (listener) => {
      playListeners.add(listener);
      return () => playListeners.delete(listener);
    },
    isPlaying,
  );
}
