#!/usr/bin/env node
// Ship just after booster separation. The pitch program should arrive near
// 200 km, near 7.8 km/s sideways, with the flight path nearly level, and
// with fuel still above the landing reserve.
// Usage: node scripts/ship_ascent.mjs [path/to/IcarusStream]

import { spawn } from "node:child_process";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const binary = process.argv[2] || join(root, "build", "IcarusStream");
const base = JSON.parse(readFileSync(join(root, "config.json"), "utf8"));

// Matches EARTH_RADIUS_M in environment/gravity.h.
const EARTH_RADIUS_M = 6371000;
const RESERVE_KG = 400;
const START_ALTITUDE_M = 100000;
const START_SPEED = 2000;
const START_GAMMA_DEG = 23.5;

function shipCase() {
  const gamma = START_GAMMA_DEG * Math.PI / 180;
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
    gravity: 9.81,
  });
  return next;
}

function flightOf(sample) {
  const radius = Math.hypot(sample.x, sample.y, sample.z);
  const upX = sample.x / radius;
  const upY = sample.y / radius;
  const upZ = sample.z / radius;
  const radial = sample.vx * upX + sample.vy * upY + sample.vz * upZ;
  const horizontal = Math.hypot(
    sample.vx - radial * upX,
    sample.vy - radial * upY,
    sample.vz - radial * upZ
  );
  const gammaDeg = Math.atan2(radial, horizontal) * 180 / Math.PI;
  return {
    t: sample.t,
    altitudeKm: (radius - EARTH_RADIUS_M) / 1000,
    horizontal,
    gammaDeg,
    fuel: sample.fuel,
  };
}

function onOrbit(point) {
  const altitude = point.altitudeKm >= 180 && point.altitudeKm <= 220;
  const speed = point.horizontal >= 7600 && point.horizontal <= 8000;
  const flat = Math.abs(point.gammaDeg) <= 3;
  const reserve = point.fuel > RESERVE_KG;
  return altitude && speed && flat && reserve;
}

function runCase(flight) {
  return new Promise((resolve) => {
    const child = spawn(binary, [], { cwd: root, stdio: ["pipe", "pipe", "pipe"] });
    let stdout = "";
    let stderr = "";
    child.stdout.setEncoding("utf8");
    child.stderr.setEncoding("utf8");
    child.stdout.on("data", (chunk) => {
      stdout += chunk;
    });
    child.stderr.on("data", (chunk) => {
      stderr += chunk;
    });
    const timer = setTimeout(() => child.kill("SIGTERM"), 120000);
    child.on("close", () => {
      clearTimeout(timer);
      const lines = stdout.trim().split("\n").filter((line) => line.length > 0);
      resolve({ lines, stderr });
    });
    child.stdin.write(JSON.stringify(flight));
    child.stdin.end();
  });
}

const result = await runCase(shipCase());
if (result.lines.length === 0) {
  console.log("no samples");
  console.log(result.stderr.slice(0, 600));
  process.exit(1);
}

let hit = null;
let hitScore = Infinity;
let closest = null;
let closestScore = Infinity;
for (const line of result.lines) {
  const point = flightOf(JSON.parse(line));
  const score = Math.abs(point.altitudeKm - 200)
    + Math.abs(point.horizontal - 7784) / 40
    + Math.abs(point.gammaDeg) * 4;
  if (onOrbit(point) && score < hitScore) {
    hit = point;
    hitScore = score;
  }
  if (point.altitudeKm > 140 && score < closestScore) {
    closestScore = score;
    closest = point;
  }
}

const shown = hit || closest;
if (shown) {
  console.log(
    hit ? "ORBIT" : "MISS",
    "t", shown.t.toFixed(1),
    "alt_km", shown.altitudeKm.toFixed(1),
    "vh", shown.horizontal.toFixed(0),
    "gamma_deg", shown.gammaDeg.toFixed(2),
    "fuel", shown.fuel.toFixed(0)
  );
}
if (result.stderr.trim().length > 0) {
  console.log(result.stderr.trim().slice(0, 400));
}
process.exit(hit ? 0 : 1);
