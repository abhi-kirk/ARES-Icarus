#!/usr/bin/env node
// One flight for the return picture: 7.6 km/s, side to the wind, then the
// landing. Samples are rewritten into east, north, and up at the ship, and
// z is altitude, which is the frame the page already draws.
// Usage: node scripts/entry_look.mjs [path/to/IcarusStream]

import { spawn } from "node:child_process";
import { createWriteStream, readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const binary = process.argv[2] || join(root, "build", "IcarusStream");
const base = JSON.parse(readFileSync(join(root, "config.json"), "utf8"));
const outPath = join(root, "..", "sim", "public", "flights", "entry-look.ndjson");
const EARTH_RADIUS_M = 6371000;
const SPEED = 7600;
const GAMMA = (-1 * Math.PI) / 180;

function rotation(from, to) {
  const fromLen = Math.hypot(from.x, from.y, from.z);
  const toLen = Math.hypot(to.x, to.y, to.z);
  const ax = from.x / fromLen;
  const ay = from.y / fromLen;
  const az = from.z / fromLen;
  const bx = to.x / toLen;
  const by = to.y / toLen;
  const bz = to.z / toLen;
  const cosAngle = ax * bx + ay * by + az * bz;
  const crossX = ay * bz - az * by;
  const crossY = az * bx - ax * bz;
  const crossZ = ax * by - ay * bx;
  const sinAngle = Math.hypot(crossX, crossY, crossZ);
  if (sinAngle < 1e-12) return (v) => ({ x: v.x, y: v.y, z: v.z });
  const axisX = crossX / sinAngle;
  const axisY = crossY / sinAngle;
  const axisZ = crossZ / sinAngle;
  return (v) => {
    const along = axisX * v.x + axisY * v.y + axisZ * v.z;
    const turnX = axisY * v.z - axisZ * v.y;
    const turnY = axisZ * v.x - axisX * v.z;
    const turnZ = axisX * v.y - axisY * v.x;
    return {
      x: v.x * cosAngle + turnX * sinAngle + axisX * along * (1 - cosAngle),
      y: v.y * cosAngle + turnY * sinAngle + axisY * along * (1 - cosAngle),
      z: v.z * cosAngle + turnZ * sinAngle + axisZ * along * (1 - cosAngle),
    };
  };
}

function entryCase(pos, vel) {
  const next = structuredClone(base);
  Object.assign(next.simulation, {
    use_rerun: false,
    rerun_spawn_viewer: false,
    rerun_save_rrd: false,
    entry: true,
    planet_centered: true,
    aero: false,
    controller: "pid",
    initial_x_pos: pos.x,
    initial_y_pos: pos.y,
    initial_z_pos: pos.z,
    initial_x_velocity: vel.x,
    initial_y_velocity: vel.y,
    initial_z_velocity: vel.z,
    initial_fuel: 600,
    initial_pad_x_pos: 0,
    initial_pad_y_pos: 0,
    initial_pad_x_velocity: 0,
    initial_pad_y_velocity: 0,
    wind_x_velocity: 0,
    wind_y_velocity: 0,
    gust_magnitude: 0,
    thrust_min: 0,
    thrust_max: 25000,
    pointing_cone_deg: 90,
    glide_cone_deg: 90,
    drag_scale: 1,
    gravity: 9.81,
    shield_capacity: 1.0e12,
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

// Quaternion that lays body +z on the given east, north, up direction.
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
  if (sample.t < 14) return sample.t - previous.t >= 0.05;
  if (sample.t > 415) return true;
  return sample.t - previous.t >= 0.2;
}

const startPos = { x: 0, y: 0, z: EARTH_RADIUS_M + 120000 };
const startVel = { x: SPEED * Math.cos(GAMMA), y: 0, z: SPEED * Math.sin(GAMMA) };
const open = await runCase(entryCase(startPos, startVel));
const impact = JSON.parse(open.lines[open.lines.length - 1]);
let pos = rotation({ x: impact.x, y: impact.y, z: impact.z }, { x: 0, y: 0, z: 1 })(startPos);
let vel = rotation({ x: impact.x, y: impact.y, z: impact.z }, { x: 0, y: 0, z: 1 })(startVel);
const aimed = await runCase(entryCase(pos, vel));
const hand = aimed.stderr.match(/entry handoff t=\S+ z=(\S+) vz=\S+ vh=\S+ x=(\S+) y=(\S+)/);
if (!hand || aimed.stderr.includes("heat shield")) {
  console.log(aimed.stderr.trim().slice(0, 400));
  process.exit(1);
}
const fix = rotation(
  { x: Number(hand[2]), y: Number(hand[3]), z: EARTH_RADIUS_M + Number(hand[1]) },
  { x: 0, y: 0, z: 1 }
);
pos = fix(pos);
vel = fix(vel);

const flight = await runCase(entryCase(pos, vel));
if (flight.stderr.includes("heat shield")) {
  console.log("shield burned");
  process.exit(1);
}

const sink = createWriteStream(outPath);
let written = 0;
let previous = null;
let prevQuat = null;
for (const line of flight.lines) {
  const sample = JSON.parse(line);
  if (!keep(sample, previous)) continue;
  const radius = Math.hypot(sample.x, sample.y, sample.z);
  const planet = radius > 1000000;
  let quat;
  if (planet) {
    const axes = axesAt(sample.x, sample.y, sample.z);
    const velocity = project({ x: sample.vx, y: sample.vy, z: sample.vz }, axes);
    const force = project({ x: sample.fx, y: sample.fy, z: sample.fz }, axes);
    const nose = project(noseOf(sample), axes);
    const noseLen = Math.hypot(nose.x, nose.y, nose.z);
    quat = quatOnNose(nose.x / noseLen, nose.y / noseLen, nose.z / noseLen);
    sample.x = 0;
    sample.y = 0;
    sample.z = axes.radius - EARTH_RADIUS_M;
    sample.vx = velocity.x;
    sample.vy = velocity.y;
    sample.vz = velocity.z;
    sample.fx = force.x;
    sample.fy = force.y;
    sample.fz = force.z;
  } else {
    quat = { w: sample.qw, x: sample.qx, y: sample.qy, z: sample.qz };
  }
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
  sample.qw = quat.w;
  sample.qx = quat.x;
  sample.qy = quat.y;
  sample.qz = quat.z;
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
  "z", last.z.toFixed(1),
  "vz", last.vz.toFixed(2)
);
