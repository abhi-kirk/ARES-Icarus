#!/usr/bin/env node
// Fixed cases the plant is built to fly, one row per environment, all three laws.
// A start the thrust ceiling cannot stop is marked infeasible and is not a
// controller failure. Hop gate: z <= 0.5, |vz| <= 2, miss <= 25, t <= 80.
// Usage: node scripts/controller_matrix.mjs [path/to/IcarusStream]

import { spawn } from "node:child_process";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const binary = process.argv[2] || join(root, "build", "IcarusStream");
const base = JSON.parse(readFileSync(join(root, "config.json"), "utf8"));

const G_EARTH = 9.81;
const G_MOON = 1.62;
const MOON_SCALE = G_MOON / G_EARTH;
const BURN_MAX = 18000;
const BURN_MIN = 7200;
const MASS_DRY = 1000;

const QUIET = {
  use_rerun: false,
  rerun_spawn_viewer: false,
  rerun_save_rrd: false,
};

// The page's Earth and Moon worlds, drop over the pad and the 70 m by 40 m divert.
const PAGE_EARTH = {
  gravity: G_EARTH,
  drag_scale: 1,
  thrust_max: BURN_MAX,
  thrust_min: BURN_MIN,
  glide_cone_deg: 20,
  pointing_cone_deg: 25,
  initial_z_pos: 280,
  initial_z_velocity: -23.5,
  initial_x_velocity: 0,
  initial_y_velocity: 0,
  wind_x_velocity: 3,
  wind_y_velocity: 1.5,
  gust_magnitude: 2,
  initial_pad_x_velocity: 0.5,
  initial_pad_y_velocity: -0.3,
  initial_fuel: 600,
};

const PAGE_MOON = {
  gravity: G_MOON,
  drag_scale: 0,
  thrust_max: BURN_MAX * MOON_SCALE,
  thrust_min: BURN_MIN * MOON_SCALE,
  glide_cone_deg: 20,
  pointing_cone_deg: 25,
  initial_z_pos: 280,
  initial_z_velocity: -23.5,
  initial_x_velocity: 0,
  initial_y_velocity: 0,
  wind_x_velocity: 0,
  wind_y_velocity: 0,
  gust_magnitude: 0,
  initial_pad_x_velocity: 0,
  initial_pad_y_velocity: 0,
  initial_fuel: 80,
};

const CASES = [
  { name: "earth_open", overrides: {} },
  {
    name: "earth_glide35",
    overrides: { glide_cone_deg: 35, initial_x_pos: 80, initial_y_pos: -40 },
  },
  {
    name: "earth_low_fuel",
    overrides: { initial_fuel: 250, initial_z_pos: 220, initial_z_velocity: -15 },
  },
  {
    name: "earth_drop",
    overrides: { ...PAGE_EARTH, initial_x_pos: 0, initial_y_pos: 0 },
  },
  {
    name: "earth_divert",
    overrides: { ...PAGE_EARTH, initial_x_pos: 70, initial_y_pos: -40 },
  },
  {
    name: "moon_drop",
    overrides: { ...PAGE_MOON, initial_x_pos: 0, initial_y_pos: 0 },
  },
  {
    name: "moon_divert",
    overrides: { ...PAGE_MOON, initial_x_pos: 70, initial_y_pos: -40 },
  },
];

function feasible(sim) {
  const z = Math.max(1, sim.initial_z_pos);
  const vz = sim.initial_z_velocity;
  const range = Math.hypot(sim.initial_x_pos, sim.initial_y_pos);
  const glide = sim.glide_cone_deg ?? 90;
  const tanGlide = glide >= 89.9 ? 1e6 : Math.tan((glide * Math.PI) / 180);
  const allow = z * tanGlide + 5;
  const mass = MASS_DRY + Math.max(0, sim.initial_fuel);
  const gravity = sim.gravity ?? G_EARTH;
  const hover = mass * gravity;
  const aBrake = sim.thrust_max / mass - gravity;
  const reasons = [];
  if (range > allow) reasons.push("outside_glide");
  if (sim.thrust_max < hover) reasons.push("cannot_hover");
  if ((sim.thrust_min ?? 0) > hover) reasons.push("floor_above_hover");
  if (aBrake <= 0) reasons.push("cannot_brake");
  else if (vz < 0 && vz * vz > 2 * aBrake * z) reasons.push("insufficient_brake");
  return { ok: reasons.length === 0, reasons };
}

function flightFor(overrides, controller) {
  const next = structuredClone(base);
  Object.assign(next.simulation, QUIET, overrides, { controller });
  return next;
}

function runFlight(flight) {
  return new Promise((resolve) => {
    const child = spawn(binary, [], { cwd: root, stdio: ["pipe", "pipe", "pipe"] });
    let carry = "";
    let lastLine = "";
    child.stdout.setEncoding("utf8");
    child.stdout.on("data", (chunk) => {
      carry += chunk;
      let nl;
      while ((nl = carry.indexOf("\n")) >= 0) {
        const line = carry.slice(0, nl).trim();
        carry = carry.slice(nl + 1);
        if (line.length > 0) lastLine = line;
      }
    });
    const timer = setTimeout(() => child.kill("SIGTERM"), 180000);
    child.on("close", () => {
      clearTimeout(timer);
      if (!lastLine) {
        resolve(null);
        return;
      }
      try {
        resolve(JSON.parse(lastLine));
      } catch {
        resolve(null);
      }
    });
    child.stdin.write(JSON.stringify(flight));
    child.stdin.end();
  });
}

function score(last) {
  if (!last) return { ok: false, text: "no sample" };
  const miss = Math.hypot(last.x - last.padX, last.y - last.padY);
  const vz = last.vz;
  let why = "LAND";
  if (last.z > 0.5) why = "no land";
  else if (Math.abs(vz) > 2) why = "hard";
  else if (miss > 25) why = "miss";
  else if (last.t > 80) why = "slow";
  const ok = why === "LAND";
  const text = `${why} z=${last.z.toFixed(2)} vz=${vz.toFixed(2)} miss=${miss.toFixed(1)} t=${last.t.toFixed(1)} fuel=${last.fuel.toFixed(0)}`;
  return { ok, text, vz, miss, t: last.t, fuel: last.fuel, z: last.z };
}

const jobs = [];
for (const row of CASES) {
  const sim = { ...base.simulation, ...row.overrides };
  const gate = feasible(sim);
  for (const controller of ["pid", "mpc", "scvx"]) {
    jobs.push({ name: row.name, controller, gate, flight: flightFor(row.overrides, controller) });
  }
}

const results = new Map();
let cursor = 0;
async function worker() {
  while (cursor < jobs.length) {
    const job = jobs[cursor];
    cursor += 1;
    const last = await runFlight(job.flight);
    const scored = score(last);
    results.set(`${job.name}:${job.controller}`, scored);
    const mark = job.gate.ok ? (scored.ok ? "ok" : "FAIL") : "envelope";
    console.log(`${mark.padEnd(8)} ${job.name.padEnd(16)} ${job.controller.padEnd(5)} ${scored.text}`);
  }
}

const pool = Math.min(4, jobs.length);
await Promise.all(Array.from({ length: pool }, () => worker()));

let failedFeasible = 0;
console.log("\ncase             feasible  pid   mpc   scvx");
for (const row of CASES) {
  const sim = { ...base.simulation, ...row.overrides };
  const gate = feasible(sim);
  const cells = ["pid", "mpc", "scvx"].map((c) => {
    const scored = results.get(`${row.name}:${c}`);
    if (!gate.ok) return scored.ok ? "land" : "out";
    if (!scored.ok) failedFeasible += 1;
    return scored.ok ? "LAND" : "FAIL";
  });
  const why = gate.ok ? "yes" : gate.reasons.join(",");
  console.log(
    `${row.name.padEnd(16)} ${why.padEnd(18)} ${cells[0].padEnd(5)} ${cells[1].padEnd(5)} ${cells[2]}`
  );
}
process.exit(failedFeasible === 0 ? 0 : 1);
