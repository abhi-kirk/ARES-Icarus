import { useSyncExternalStore } from "react";
import { durationOf, KNOBS, type Knobs, type SceneId } from "./scenes";

// One clock for the page. Dragging the slider, the body, or a chat time pauses it.
// Play runs at real time: one second on the picture is one second on the wall.

let time = 0.4;
let playing = false;
let scene: SceneId = "state";
let knobs: Knobs = { ...KNOBS };
let lastStamp = 0;
let frame = 0;

const listeners = new Set<() => void>();
const playListeners = new Set<() => void>();

function publish(): void {
  listeners.forEach((listener) => listener());
}

function publishPlay(): void {
  playListeners.forEach((listener) => listener());
}

export function getTime(): number {
  return time;
}

export function getScene(): SceneId {
  return scene;
}

export function getKnobs(): Knobs {
  return knobs;
}

export function isPlaying(): boolean {
  return playing;
}

function endOf(): number {
  return durationOf(scene);
}

function place(next: number): number {
  const end = endOf();
  if (next >= end) return end;
  if (next < 0) return 0;
  return next;
}

export function setTime(next: number): void {
  stop();
  const placed = place(next);
  if (Math.abs(placed - time) < 1e-6) return;
  time = placed;
  publish();
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
  const end = endOf();

  if (next >= end) {
    time = end;
    publish();
    stop();
    return;
  }

  time = next;
  publish();
  frame = requestAnimationFrame(tick);
}

export function play(): void {
  if (playing) return;

  const end = endOf();
  if (time >= end - 1e-4) {
    time = 0;
    publish();
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

export function setScene(next: SceneId): void {
  if (next === scene) return;
  stop();
  scene = next;
  time = next === "state" ? 0.4 : 0;
  publish();
}

export function setKnob(key: keyof Knobs, value: number): void {
  stop();
  knobs = { ...knobs, [key]: value };
  publish();
}

export function useTime(): number {
  return useSyncExternalStore(subscribe, getTime);
}

export function useScene(): SceneId {
  return useSyncExternalStore(subscribe, getScene);
}

export function useKnobs(): Knobs {
  return useSyncExternalStore(subscribe, getKnobs);
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

function subscribe(listener: () => void): () => void {
  listeners.add(listener);
  return () => listeners.delete(listener);
}
