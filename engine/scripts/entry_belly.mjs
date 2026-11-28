#!/usr/bin/env node
// One run from just above the air to the pad.
// The nose stays across the ground track while the ship is faster than
// 2 km/s, so the side slows a 7.6 km/s entry. Then the existing flip and
// brake hand the hop to the landing law. The shield is 1e12. A nose-first
// pass at this speed still exceeds that.
// Usage: node scripts/entry_belly.mjs [path/to/IcarusStream]

import { spawn } from "node:child_process";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const binary = process.argv[2] || join(root, "build", "IcarusStream");
const base = JSON.parse(readFileSync(join(root, "config.json"), "utf8"));
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
  if (sinAngle < 1e-12) {
    return (v) => ({ x: v.x, y: v.y, z: v.z });
  }
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

function entryCase(controller, pos, vel) {
  const next = structuredClone(base);
  Object.assign(next.simulation, {
    use_rerun: false,
    rerun_spawn_viewer: false,
    rerun_save_rrd: false,
    entry: true,
    planet_centered: true,
    aero: false,
    controller,
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
    let lastLine = "";
    let buf = "";
    child.stdout.setEncoding("utf8");
    child.stderr.setEncoding("utf8");
    child.stdout.on("data", (chunk) => {
      buf += chunk;
      const lines = buf.split("\n");
      buf = lines.pop();
      for (const line of lines) {
        if (line.length > 0) lastLine = line;
      }
    });
    child.stderr.on("data", (chunk) => {
      stderr += chunk;
    });
    const timer = setTimeout(() => child.kill("SIGTERM"), 240000);
    child.on("close", (code) => {
      clearTimeout(timer);
      resolve({ code, lastLine, stderr });
    });
    child.stdin.write(JSON.stringify(flight));
    child.stdin.end();
  });
}

function parseHandoff(stderr) {
  const match = stderr.match(
    /entry handoff t=(\S+) z=(\S+) vz=(\S+) vh=(\S+) x=(\S+) y=(\S+) fuel=(\S+)/
  );
  if (!match) return null;
  return {
    t: Number(match[1]),
    z: Number(match[2]),
    vz: Number(match[3]),
    vh: Number(match[4]),
    x: Number(match[5]),
    y: Number(match[6]),
    fuel: Number(match[7]),
  };
}

const startPos = { x: 0, y: 0, z: EARTH_RADIUS_M + 120000 };
const startVel = {
  x: SPEED * Math.cos(GAMMA),
  y: 0,
  z: SPEED * Math.sin(GAMMA),
};

const open = await runCase(entryCase("pid", startPos, startVel));
if (open.stderr.includes("heat shield") || open.lastLine.length === 0) {
  console.log("open entry burned the shield or produced no samples");
  console.log(open.stderr.trim().slice(0, 500));
  process.exit(1);
}
const impact = JSON.parse(open.lastLine);
let pos = startPos;
let vel = startVel;
const ontoPad = rotation(
  { x: impact.x, y: impact.y, z: impact.z },
  { x: 0, y: 0, z: 1 }
);
pos = ontoPad(pos);
vel = ontoPad(vel);

const aimed = await runCase(entryCase("pid", pos, vel));
const aimHand = parseHandoff(aimed.stderr);
if (!aimHand || aimed.stderr.includes("heat shield")) {
  console.log("aimed entry did not hand off");
  console.log(aimed.stderr.trim().slice(0, 800));
  process.exit(1);
}
const fix = rotation(
  { x: aimHand.x, y: aimHand.y, z: EARTH_RADIUS_M + aimHand.z },
  { x: 0, y: 0, z: 1 }
);
pos = fix(pos);
vel = fix(vel);
console.log(
  "aim",
  "x0", pos.x.toFixed(0),
  "y0", pos.y.toFixed(0),
  "z0", pos.z.toFixed(0)
);

let failed = 0;
for (const controller of ["pid", "mpc", "scvx"]) {
  const result = await runCase(entryCase(controller, pos, vel));
  const burned = result.stderr.includes("heat shield");
  const hand = parseHandoff(result.stderr);
  if (result.lastLine.length === 0 || !hand || burned) {
    console.log(controller, burned ? "HEAT" : "NO HANDOFF");
    console.log(result.stderr.trim().slice(0, 500));
    failed += 1;
    continue;
  }
  const last = JSON.parse(result.lastLine);
  const miss = Math.hypot(last.x - last.padX, last.y - last.padY);
  const landed = last.z <= 0.5 && Math.abs(last.vz) <= 2 && miss <= 25;
  console.log(
    controller,
    landed ? "LAND" : "MISS",
    "z", last.z.toFixed(2),
    "vz", last.vz.toFixed(2),
    "miss", miss.toFixed(1),
    "t", last.t.toFixed(1),
    "fuel", last.fuel.toFixed(0),
    "handoff z", hand.z.toFixed(0),
    "vz", hand.vz.toFixed(1),
    "fuel", hand.fuel.toFixed(0)
  );
  if (!landed) failed += 1;
}

process.exit(failed === 0 ? 0 : 1);
