#!/usr/bin/env node
// Heat on the way down. The shield integrates sqrt(rho) * v^3.
// 7.8 km/s in thick air burns through it. The 1 km/s coast does not.
// A ship that stays inside the shield reaches the landing handoff,
// in the pad frame the existing laws already fly.
// Usage: node scripts/ship_heat.mjs [path/to/IcarusStream]

import { spawn } from "node:child_process";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const binary = process.argv[2] || join(root, "build", "IcarusStream");
const base = JSON.parse(readFileSync(join(root, "config.json"), "utf8"));

const EARTH_RADIUS_M = 6371000;
const SHIELD = 1.0e11;

function withSim(fields) {
  const next = structuredClone(base);
  Object.assign(next.simulation, {
    use_rerun: false,
    rerun_spawn_viewer: false,
    rerun_save_rrd: false,
    controller: "pid",
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
    shield_capacity: SHIELD,
    ...fields,
  });
  return next;
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
      resolve({ stdout, stderr });
    });
    child.stdin.write(JSON.stringify(flight));
    child.stdin.end();
  });
}

function handoff(stderr) {
  const match = stderr.match(/entry handoff t=(\S+) z=(\S+) vz=(\S+)/);
  if (!match) return null;
  return { t: Number(match[1]), z: Number(match[2]), vz: Number(match[3]) };
}

let failed = 0;

const hot = await runCase(withSim({
  entry: true,
  initial_x_pos: 0,
  initial_y_pos: 0,
  initial_z_pos: 40000,
  initial_x_velocity: 7800,
  initial_y_velocity: 0,
  initial_z_velocity: 0,
  initial_fuel: 600,
}));
const hotLine = hot.stderr.includes("heat shield");
console.log(hotLine ? "HOT heat shield" : "HOT missed the shield", hot.stderr.trim().split("\n")[0]);
if (!hotLine) failed += 1;

const coast = await runCase(withSim({
  entry: true,
  initial_x_pos: 0,
  initial_y_pos: 0,
  initial_z_pos: 100000,
  initial_x_velocity: 1000,
  initial_y_velocity: 0,
  initial_z_velocity: 0,
  initial_fuel: 600,
}));
const coastHand = handoff(coast.stderr);
const coastBurned = coast.stderr.includes("heat shield");
console.log(
  !coastBurned && coastHand ? "COAST held" : "COAST failed",
  coastHand ? `handoff t ${coastHand.t.toFixed(1)} z ${coastHand.z.toFixed(0)}` : "no handoff",
  coastBurned ? "heat shield" : "shield ok"
);
if (coastBurned || !coastHand) failed += 1;

const ship = await runCase(withSim({
  entry: true,
  planet_centered: true,
  aero: false,
  // On the way down, over the pad, where the planet frame and the pad
  // frame still agree. This is the state the 1 km/s coast flips from:
  // 4 km up, 141 m/s sideways, 309 m/s down. The brake hands the landing
  // laws that hop in the pad frame. Drag stays the coast law. A tilted
  // nose at this dynamic pressure adds more force than the brake can spare.
  initial_x_pos: 0,
  initial_y_pos: 0,
  initial_z_pos: EARTH_RADIUS_M + 4127,
  initial_x_velocity: 141,
  initial_y_velocity: 0,
  initial_z_velocity: -309,
  initial_fuel: 600,
}));
const shipHand = handoff(ship.stderr);
const shipBurned = ship.stderr.includes("heat shield");
console.log(
  !shipBurned && shipHand ? "SHIP handoff" : "SHIP failed",
  shipHand ? `t ${shipHand.t.toFixed(1)} z ${shipHand.z.toFixed(0)} vz ${shipHand.vz.toFixed(1)}` : "no handoff",
  shipBurned ? "heat shield" : "shield ok"
);
if (!shipHand || shipBurned) {
  failed += 1;
  console.log(ship.stderr.trim().slice(0, 500));
}

process.exit(failed === 0 ? 0 : 1);
