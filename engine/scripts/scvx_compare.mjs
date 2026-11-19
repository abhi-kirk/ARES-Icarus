#!/usr/bin/env node
// One Moon hop, flown by the tracking planner and by the fuel planner.
// Usage: node scripts/scvx_compare.mjs

import { spawn } from "node:child_process";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const binary = join(root, "build", "IcarusStream");
const base = JSON.parse(readFileSync(join(root, "config.json"), "utf8"));

const divert = {
  glide_cone_deg: 20,
  pointing_cone_deg: 25,
  thrust_max: 18000,
  thrust_min: 7200,
  initial_z_pos: 280,
  initial_z_velocity: -23.5,
  initial_x_velocity: 0,
  initial_y_velocity: 0,
  initial_x_pos: 70,
  initial_y_pos: -40,
  wind_x_velocity: 0,
  wind_y_velocity: 0,
  gust_magnitude: 0,
  drag_scale: 0,
  initial_fuel: 80,
  initial_pad_x_velocity: 0,
  initial_pad_y_velocity: 0,
};

function run(controller) {
  const next = structuredClone(base);
  next.simulation.use_rerun = false;
  next.simulation.rerun_spawn_viewer = false;
  next.simulation.rerun_save_rrd = false;
  Object.assign(next.simulation, divert);
  next.simulation.controller = controller;

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
    const timer = setTimeout(() => child.kill("SIGTERM"), 240_000);
    child.on("close", (code) => {
      clearTimeout(timer);
      const lines = stdout.trim().split("\n").filter((line) => line.length > 0);
      let last = null;
      try {
        last = JSON.parse(lines[lines.length - 1]);
      } catch (err) {
        resolve({ controller, ok: false, detail: String(err), stderr });
        return;
      }
      const miss = Math.hypot(last.x - last.padX, last.y - last.padY);
      const fuelUsed = 80 - last.fuel;
      resolve({
        controller,
        code,
        z: last.z,
        vz: last.vz,
        miss,
        fuelLeft: last.fuel,
        fuelUsed,
        t: last.t,
        stderr,
      });
    });
    child.stdin.write(JSON.stringify(next));
    child.stdin.end();
  });
}

const only = process.argv[2];
const mpc = only === "scvx" ? null : await run("mpc");
const scvx = only === "mpc" ? null : await run("scvx");

function line(row) {
  if (row.z === undefined) return `${row.controller} failed ${row.detail}`;
  return [
    row.controller,
    `z=${row.z.toFixed(2)}`,
    `vz=${row.vz.toFixed(2)}`,
    `miss=${row.miss.toFixed(1)}`,
    `fuel_used=${row.fuelUsed.toFixed(1)}`,
    `fuel_left=${row.fuelLeft.toFixed(1)}`,
    `t=${row.t.toFixed(1)}`,
  ].join("  ");
}

if (mpc) console.log(line(mpc));
if (scvx) console.log(line(scvx));
if (scvx && scvx.stderr) {
  const notes = scvx.stderr.trim().split("\n").filter((s) => s.startsWith("scvx ") || s.startsWith("qp "));
  for (const note of notes) console.log(note);
}
