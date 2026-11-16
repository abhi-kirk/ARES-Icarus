#!/usr/bin/env node
// Headless LTV-MPC landing regressions against IcarusStream.
// Usage: node scripts/mpc_regress.mjs [path/to/IcarusStream]

import { spawn } from "node:child_process";
import { readFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const binary = process.argv[2] || join(root, "build", "IcarusStream");
const base = JSON.parse(readFileSync(join(root, "config.json"), "utf8"));

function cloneCase(overrides) {
  const next = structuredClone(base);
  next.simulation.use_rerun = false;
  next.simulation.rerun_spawn_viewer = false;
  next.simulation.rerun_save_rrd = false;
  next.simulation.controller = "mpc";
  Object.assign(next.simulation, overrides);
  return next;
}

function runCase(name, overrides, check) {
  return new Promise((resolve) => {
    const caseJson = JSON.stringify(cloneCase(overrides));
    const child = spawn(binary, [], {
      cwd: root,
      stdio: ["pipe", "pipe", "pipe"],
    });

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

    const timer = setTimeout(() => {
      child.kill("SIGTERM");
    }, 90_000);

    child.on("close", (code) => {
      clearTimeout(timer);
      const lines = stdout
        .trim()
        .split("\n")
        .filter((line) => line.length > 0);
      if (lines.length === 0) {
        resolve({
          name,
          ok: false,
          detail: `no samples (exit ${code}): ${stderr.slice(0, 200)}`,
        });
        return;
      }

      let last = null;
      try {
        last = JSON.parse(lines[lines.length - 1]);
      } catch (err) {
        resolve({ name, ok: false, detail: `bad last line: ${err.message}` });
        return;
      }

      const result = check(last, lines);
      resolve({ name, ok: result.ok, detail: result.detail, last });
    });

    child.stdin.write(caseJson);
    child.stdin.end();
  });
}

function touchdown(last, opts = {}) {
  const maxZ = opts.maxZ ?? 0.5;
  const maxSpeed = opts.maxSpeed ?? 2.0;
  const maxMiss = opts.maxMiss ?? 25.0;
  const maxT = opts.maxT ?? 80.0;
  const dx = last.x - last.padX;
  const dy = last.y - last.padY;
  const miss = Math.hypot(dx, dy);
  const speed = Math.hypot(last.vx, last.vy, last.vz);
  const parts = [
    `z=${last.z.toFixed(2)}`,
    `vz=${last.vz.toFixed(2)}`,
    `miss=${miss.toFixed(1)}`,
    `speed=${speed.toFixed(2)}`,
    `t=${last.t.toFixed(1)}`,
  ];
  if (last.z > maxZ) {
    return { ok: false, detail: `did not land (${parts.join(", ")})` };
  }
  if (Math.abs(last.vz) > maxSpeed) {
    return { ok: false, detail: `hard landing (${parts.join(", ")})` };
  }
  if (miss > maxMiss) {
    return { ok: false, detail: `missed pad (${parts.join(", ")})` };
  }
  if (last.t > maxT) {
    return { ok: false, detail: `too slow (${parts.join(", ")})` };
  }
  return { ok: true, detail: parts.join(", ") };
}

const cases = [
  {
    name: "nominal",
    overrides: {},
    check: (last) => touchdown(last, { maxSpeed: 1.5, maxMiss: 20 }),
  },
  {
    name: "tight_glide",
    overrides: {
      glide_cone_deg: 35.0,
      initial_x_pos: 80,
      initial_y_pos: -40,
    },
    check: (last) => touchdown(last, { maxSpeed: 2.0, maxMiss: 25 }),
  },
  {
    name: "low_fuel",
    overrides: {
      initial_fuel: 250.0,
      initial_z_pos: 220,
      initial_z_velocity: -15,
    },
    check: (last) => touchdown(last, { maxSpeed: 2.5, maxMiss: 30 }),
  },
];

const results = [];
for (const c of cases) {
  process.stdout.write(`running ${c.name}... `);
  const result = await runCase(c.name, c.overrides, c.check);
  results.push(result);
  console.log(result.ok ? `ok (${result.detail})` : `FAIL (${result.detail})`);
}

const failed = results.filter((r) => !r.ok);
if (failed.length > 0) {
  console.error(`\n${failed.length}/${results.length} failed`);
  process.exit(1);
}
console.log(`\n${results.length}/${results.length} passed`);
