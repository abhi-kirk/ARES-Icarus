#!/usr/bin/env node
// 100 km Earth coast, then the existing landing law.
// The engine stays off until a brake can still reach a hop those laws
// already fly. Success is the touchdown gate. The 80 s hop clock does not apply.
// Usage: node scripts/entry_coast.mjs [path/to/IcarusStream]

import { spawn } from "node:child_process";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const binary = process.argv[2] || join(root, "build", "IcarusStream");
const base = JSON.parse(readFileSync(join(root, "config.json"), "utf8"));

function entryCase(controller, x, y) {
  const next = structuredClone(base);
  Object.assign(next.simulation, {
    use_rerun: false,
    rerun_spawn_viewer: false,
    rerun_save_rrd: false,
    entry: true,
    controller,
    initial_x_pos: x,
    initial_y_pos: y,
    initial_z_pos: 100000,
    initial_x_velocity: 1000,
    initial_y_velocity: 0,
    initial_z_velocity: 0,
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
    const timer = setTimeout(() => child.kill("SIGTERM"), 240000);
    child.on("close", (code) => {
      clearTimeout(timer);
      const lines = stdout.trim().split("\n").filter((line) => line.length > 0);
      resolve({ code, lines, stderr });
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

function report(name, result) {
  if (result.lines.length === 0) {
    console.log(name, "NO SAMPLES", result.stderr.slice(0, 400));
    return false;
  }
  const first = JSON.parse(result.lines[0]);
  const last = JSON.parse(result.lines[result.lines.length - 1]);
  const miss = Math.hypot(last.x - last.padX, last.y - last.padY);
  const landed = last.z <= 0.5 && Math.abs(last.vz) <= 2 && miss <= 25;
  const thrust0 = Math.hypot(first.fx, first.fy, first.fz);
  console.log(
    name,
    landed ? "LAND" : "MISS",
    "z", last.z.toFixed(2),
    "vz", last.vz.toFixed(2),
    "miss", miss.toFixed(1),
    "t", last.t.toFixed(1),
    "fuel", last.fuel.toFixed(0),
    "startPitch", first.pitch.toFixed(3),
    "startThrust", thrust0.toFixed(1)
  );
  const hand = parseHandoff(result.stderr);
  if (hand) {
    console.log(
      "  handoff",
      "t", hand.t.toFixed(1),
      "z", hand.z.toFixed(0),
      "vz", hand.vz.toFixed(1),
      "vh", hand.vh.toFixed(1),
      "fuel", hand.fuel.toFixed(0),
      "x", hand.x.toFixed(0)
    );
  } else {
    console.log("  no handoff");
    console.log(result.stderr.trim());
  }
  return landed;
}

const calibration = await runCase(entryCase("pid", 0, 0));
const handoff = parseHandoff(calibration.stderr);
if (!handoff) {
  console.log("calibration did not reach a hop");
  console.log(calibration.stderr.trim());
  if (calibration.lines.length > 0) {
    const last = JSON.parse(calibration.lines[calibration.lines.length - 1]);
    console.log("last z", last.z, "vz", last.vz, "t", last.t);
  }
  process.exit(1);
}

console.log(
  "aim",
  "x0", (-handoff.x).toFixed(1),
  "y0", (-handoff.y).toFixed(1),
  "from handoff x", handoff.x.toFixed(1)
);

let failed = 0;
for (const controller of ["pid", "mpc", "scvx"]) {
  const result = await runCase(entryCase(controller, -handoff.x, -handoff.y));
  const ok = report(controller, result);
  if (!ok) failed += 1;
}

process.exit(failed === 0 ? 0 : 1);
