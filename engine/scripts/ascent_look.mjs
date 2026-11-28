#!/usr/bin/env node
// The insertion picture. The pitch program runs from booster separation
// and the samples are rewritten into east, north, and up at the ship,
// with z as altitude, which is the frame the page already draws.
// Usage: node scripts/ascent_look.mjs [path/to/IcarusStream]

import { spawn } from "node:child_process";
import { createWriteStream, readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const binary = process.argv[2] || join(root, "build", "IcarusStream");
const base = JSON.parse(readFileSync(join(root, "config.json"), "utf8"));
const outPath = join(root, "..", "sim", "public", "flights", "ascent-look.ndjson");
const EARTH_RADIUS_M = 6371000;
const START_ALTITUDE_M = 100000;
const START_SPEED = 2000;
const START_GAMMA_DEG = 23.5;

function shipCase() {
  const gamma = (START_GAMMA_DEG * Math.PI) / 180;
  const next = structuredClone(base);
  Object.assign(next.simulation, {
    use_rerun: false,
    rerun_spawn_viewer: false,
    rerun_save_rrd: false,
    ascent: true,
    planet_centered: true,
    entry: false,
    controller: "pid",
    initial_x_pos: 0,
    initial_y_pos: 0,
    initial_z_pos: EARTH_RADIUS_M + START_ALTITUDE_M,
    initial_x_velocity: START_SPEED * Math.cos(gamma),
    initial_y_velocity: 0,
    initial_z_velocity: START_SPEED * Math.sin(gamma),
    initial_fuel: 5000,
    initial_pad_x_pos: 0,
    initial_pad_y_pos: 0,
    initial_pad_x_velocity: 0,
    initial_pad_y_velocity: 0,
    wind_x_velocity: 0,
    wind_y_velocity: 0,
    gust_magnitude: 0,
    thrust_min: 0,
    thrust_max: 100000,
    exhaust_speed: 5000,
    pointing_cone_deg: 90,
    glide_cone_deg: 90,
    drag_scale: 1,
    aero: true,
    gravity: 9.81,
  });
  return next;
}

function runCase(flight) {
  return new Promise((resolve) => {
    const child = spawn(binary, [], { cwd: root, stdio: ["pipe", "pipe", "pipe"] });
    let stderr = "";
    const lines = [];
    let buf = "";
    child.stdout.setEncoding("utf8");
    child.stderr.setEncoding("utf8");
    child.stdout.on("data", (chunk) => {
      buf += chunk;
      const parts = buf.split("\n");
      buf = parts.pop();
      for (const line of parts) {
        if (line.length > 0) lines.push(line);
      }
    });
    child.stderr.on("data", (chunk) => {
      stderr += chunk;
    });
    child.on("close", () => resolve({ lines, stderr }));
    child.stdin.write(JSON.stringify(flight));
    child.stdin.end();
  });
}

function axesAt(x, y, z) {
  const radius = Math.hypot(x, y, z);
  const upX = x / radius;
  const upY = y / radius;
  const upZ = z / radius;
  let eastX = -upY;
  let eastY = upX;
  let eastZ = 0;
  const eastLen = Math.hypot(eastX, eastY, eastZ);
  if (eastLen < 1e-8) {
    eastX = 1;
    eastY = 0;
    eastZ = 0;
  } else {
    eastX /= eastLen;
    eastY /= eastLen;
    eastZ /= eastLen;
  }
  const northX = upY * eastZ - upZ * eastY;
  const northY = upZ * eastX - upX * eastZ;
  const northZ = upX * eastY - upY * eastX;
  return { eastX, eastY, eastZ, northX, northY, northZ, upX, upY, upZ, radius };
}

function project(v, axes) {
  return {
    x: v.x * axes.eastX + v.y * axes.eastY + v.z * axes.eastZ,
    y: v.x * axes.northX + v.y * axes.northY + v.z * axes.northZ,
    z: v.x * axes.upX + v.y * axes.upY + v.z * axes.upZ,
  };
}

function noseOf(sample) {
  const w = sample.qw;
  const x = sample.qx;
  const y = sample.qy;
  const z = sample.qz;
  return {
    x: 2 * (x * z + w * y),
    y: 2 * (y * z - w * x),
    z: 1 - 2 * (x * x + y * y),
  };
}

function quatOnNose(east, north, up) {
  const dot = up;
  if (dot > 0.999999) return { w: 1, x: 0, y: 0, z: 0 };
  if (dot < -0.999999) return { w: 0, x: 1, y: 0, z: 0 };
  const scale = Math.sqrt((1 + dot) * 2);
  const inv = 1 / scale;
  return { w: scale * 0.5, x: -north * inv, y: east * inv, z: 0 };
}

function keep(sample, previous) {
  if (!previous) return true;
  if (sample.t < 20) return sample.t - previous.t >= 0.1;
  return sample.t - previous.t >= 0.25;
}

const flight = await runCase(shipCase());
if (flight.lines.length === 0) {
  console.log("no samples");
  console.log(flight.stderr.slice(0, 600));
  process.exit(1);
}

const sink = createWriteStream(outPath);
let written = 0;
let previous = null;
let prevQuat = null;
let peak = null;
for (const line of flight.lines) {
  const sample = JSON.parse(line);
  if (!keep(sample, previous)) continue;
  const radius = Math.hypot(sample.x, sample.y, sample.z);
  const axes = axesAt(sample.x, sample.y, sample.z);
  const velocity = project({ x: sample.vx, y: sample.vy, z: sample.vz }, axes);
  const force = project({ x: sample.fx, y: sample.fy, z: sample.fz }, axes);
  const nose = project(noseOf(sample), axes);
  const noseLen = Math.hypot(nose.x, nose.y, nose.z) || 1;
  let quat = quatOnNose(nose.x / noseLen, nose.y / noseLen, nose.z / noseLen);
  if (prevQuat) {
    const same = prevQuat.w * quat.w + prevQuat.x * quat.x + prevQuat.y * quat.y + prevQuat.z * quat.z;
    if (same < 0) {
      quat.w = -quat.w;
      quat.x = -quat.x;
      quat.y = -quat.y;
      quat.z = -quat.z;
    }
  }
  prevQuat = quat;
  sample.x = 0;
  sample.y = 0;
  sample.z = radius - EARTH_RADIUS_M;
  sample.vx = velocity.x;
  sample.vy = velocity.y;
  sample.vz = velocity.z;
  sample.fx = force.x;
  sample.fy = force.y;
  sample.fz = force.z;
  sample.qw = quat.w;
  sample.qx = quat.x;
  sample.qy = quat.y;
  sample.qz = quat.z;
  const horizontal = Math.hypot(velocity.x, velocity.y);
  if (!peak || horizontal > peak.horizontal) {
    peak = { t: sample.t, altitudeKm: sample.z / 1000, horizontal, fuel: sample.fuel };
  }
  sink.write(`${JSON.stringify(sample)}\n`);
  written += 1;
  previous = sample;
}
await new Promise((resolve) => sink.end(resolve));
const last = previous;
console.log(
  "wrote",
  written,
  "t", last.t.toFixed(1),
  "z", last.z.toFixed(0),
  "peak_vh", peak.horizontal.toFixed(0),
  "peak_alt_km", peak.altitudeKm.toFixed(1),
  "fuel", last.fuel.toFixed(0)
);
if (flight.stderr.trim().length > 0) {
  console.log(flight.stderr.trim().slice(0, 400));
}
