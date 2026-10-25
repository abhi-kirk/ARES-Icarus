import { bodyAt, springRate, steppedRate } from "./scenes";
import { getKnobs, getScene, getTime, isPlaying } from "./sceneStore";

// One ear on the same clock as the picture. Pause silences it.
// The spring is quiet when the mass stops, and loudest as it passes the rest point.
// The rocket is an engine. You hear the thrust it is making, not the thrust you asked for.

let ctx: AudioContext | null = null;
let airGain: GainNode | null = null;
let rumbleGain: GainNode | null = null;
let rumbleFilter: BiquadFilterNode | null = null;
let tone: OscillatorNode | null = null;
let toneGain: GainNode | null = null;
let frame = 0;
let enabled = false;

export function soundOn(): boolean {
  return enabled;
}

function noiseBuffer(context: AudioContext, brown: boolean): AudioBuffer {
  const length = context.sampleRate * 2;
  const buffer = context.createBuffer(1, length, context.sampleRate);
  const data = buffer.getChannelData(0);
  let last = 0;
  for (let i = 0; i < length; i++) {
    const white = Math.random() * 2 - 1;
    if (brown) {
      last = (last + 0.02 * white) / 1.02;
      data[i] = last * 3.2;
    } else {
      data[i] = white * 0.4;
    }
  }
  return buffer;
}

function build(context: AudioContext): void {
  const master = context.createGain();
  master.gain.value = 0.85;
  const compressor = context.createDynamicsCompressor();
  compressor.threshold.value = -18;
  compressor.knee.value = 10;
  compressor.ratio.value = 3;
  compressor.attack.value = 0.01;
  compressor.release.value = 0.18;
  master.connect(compressor);
  compressor.connect(context.destination);

  const air = context.createBufferSource();
  air.buffer = noiseBuffer(context, false);
  air.loop = true;
  const airFilter = context.createBiquadFilter();
  airFilter.type = "bandpass";
  airFilter.frequency.value = 900;
  airFilter.Q.value = 0.6;
  airGain = context.createGain();
  airGain.gain.value = 0;
  air.connect(airFilter);
  airFilter.connect(airGain);
  airGain.connect(master);
  air.start();

  const rumble = context.createBufferSource();
  rumble.buffer = noiseBuffer(context, true);
  rumble.loop = true;
  rumbleFilter = context.createBiquadFilter();
  rumbleFilter.type = "lowpass";
  rumbleFilter.frequency.value = 140;
  rumbleFilter.Q.value = 0.7;
  rumbleGain = context.createGain();
  rumbleGain.gain.value = 0;
  rumble.connect(rumbleFilter);
  rumbleFilter.connect(rumbleGain);
  rumbleGain.connect(master);
  rumble.start();

  tone = context.createOscillator();
  tone.type = "sine";
  tone.frequency.value = 48;
  toneGain = context.createGain();
  toneGain.gain.value = 0;
  tone.connect(toneGain);
  toneGain.connect(master);
  tone.start();
}

function aim(param: AudioParam, value: number, seconds = 0.05): void {
  if (!ctx) return;
  param.setTargetAtTime(value, ctx.currentTime, seconds);
}

function hush(): void {
  if (!airGain || !rumbleGain || !toneGain) return;
  aim(airGain.gain, 0, 0.02);
  aim(rumbleGain.gain, 0, 0.02);
  aim(toneGain.gain, 0, 0.02);
}

function follow(): void {
  if (!ctx || !enabled || !airGain || !rumbleGain || !rumbleFilter || !tone || !toneGain) return;
  if (ctx.state !== "running") return;
  if (!isPlaying()) {
    hush();
    return;
  }

  const scene = getScene();
  const time = getTime();
  const knobs = getKnobs();
  const spring = scene === "state" || scene === "step";

  if (spring) {
    const speed = scene === "state" ? Math.abs(springRate(time)) : Math.abs(steppedRate(time, knobs.dt));
    const heard = Math.min(1.4, speed);
    aim(airGain.gain, heard * heard * 0.2);
    aim(rumbleGain.gain, 0);
    aim(toneGain.gain, 0);
    return;
  }

  const flame = bodyAt(scene, time, knobs).flame;
  const thrust = Math.max(0, Math.min(1, flame));
  aim(airGain.gain, 0);
  aim(rumbleGain.gain, thrust * 0.32);
  aim(rumbleFilter.frequency, 90 + thrust * 200);
  aim(tone.frequency, 40 + thrust * 42);
  aim(toneGain.gain, thrust * 0.045);
}

function loop(): void {
  follow();
  frame = requestAnimationFrame(loop);
}

export async function unlock(): Promise<void> {
  if (!enabled) return;
  if (!ctx) {
    ctx = new AudioContext();
    build(ctx);
  }
  if (ctx.state === "suspended") await ctx.resume();
  cancelAnimationFrame(frame);
  frame = requestAnimationFrame(loop);
}

export function toggleSound(): boolean {
  enabled = !enabled;
  if (!enabled) {
    hush();
    cancelAnimationFrame(frame);
    return false;
  }
  void unlock();
  return true;
}
