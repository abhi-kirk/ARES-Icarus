#!/usr/bin/env node
// Local Monte Carlo: dispersed landings through IcarusStream, PID vs MPC.
// Lab tool only — runs on this machine, not on Fly.
//
// Usage:
//   node scripts/monte_carlo.mjs
//   node scripts/monte_carlo.mjs --n 100 --seed 2 --jobs 8
//   node scripts/monte_carlo.mjs [path/to/IcarusStream]

import { spawn } from "node:child_process";
import { cpus } from "node:os";
import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const base = JSON.parse(readFileSync(join(root, "config.json"), "utf8"));

const SUCCESS = {
  maxZ: 0.5,
  maxVz: 2.0,
  maxMiss: 25.0,
  maxT: 80.0,
};

function parseArgs(argv) {
  let n = 50;
  let seed = 1;
  let jobs = Math.max(1, cpus().length);
  let binary = join(root, "build", "IcarusStream");

  for (let i = 0; i < argv.length; ++i) {
    const a = argv[i];
    if (a === "--n" && argv[i + 1] != null) {
      n = Math.max(1, parseInt(argv[++i], 10) || n);
    } else if (a === "--seed" && argv[i + 1] != null) {
      seed = parseInt(argv[++i], 10) || seed;
    } else if (a === "--jobs" && argv[i + 1] != null) {
      jobs = Math.max(1, parseInt(argv[++i], 10) || jobs);
    } else if (!a.startsWith("-")) {
      binary = a;
    }
  }
  return { n, seed, jobs, binary };
}

// Mulberry32 — deterministic uint32 stream from a seed.
function mulberry32(seed) {
  let t = seed >>> 0;
  return function next() {
    t = (t + 0x6d2b79f5) >>> 0;
    let r = Math.imul(t ^ (t >>> 15), 1 | t);
    r ^= r + Math.imul(r ^ (r >>> 7), 61 | r);
    return ((r ^ (r >>> 14)) >>> 0) / 4294967296;
  };
}

function randn(rng) {
  // Box-Muller
  const u = Math.max(1e-12, rng());
  const v = rng();
  return Math.sqrt(-2.0 * Math.log(u)) * Math.cos(2.0 * Math.PI * v);
}

function clamp(x, lo, hi) {
  return Math.min(hi, Math.max(lo, x));
}

function mean(xs) {
  if (xs.length === 0) return 0;
  let s = 0;
  for (const x of xs) s += x;
  return s / xs.length;
}

function percentile(xs, p) {
  if (xs.length === 0) return 0;
  const sorted = [...xs].sort((a, b) => a - b);
  const idx = Math.min(sorted.length - 1, Math.floor((p / 100) * sorted.length));
  return sorted[idx];
}

function makeDraw(rng, sim) {
  const windX = clamp(sim.wind_x_velocity + 5.0 * randn(rng), -20, 20);
  const windY = clamp(sim.wind_y_velocity + 5.0 * randn(rng), -20, 20);
  const gust = clamp(sim.gust_magnitude + 2.0 * randn(rng), 0, 15);
  // Thrust shortfall: scale in [0.7, 1.0]
  const thrustScale = clamp(1.0 - 0.15 * Math.abs(randn(rng)), 0.7, 1.0);
  const x0 = sim.initial_x_pos + 20.0 * randn(rng);
  const y0 = sim.initial_y_pos + 20.0 * randn(rng);
  const vz0 = clamp(sim.initial_z_velocity + 3.0 * randn(rng), -40, -5);

  return {
    wind_x_velocity: windX,
    wind_y_velocity: windY,
    gust_magnitude: gust,
    thrust_max: sim.thrust_max * thrustScale,
    thrust_scale: thrustScale,
    initial_x_pos: x0,
    initial_y_pos: y0,
    initial_z_velocity: vz0,
  };
}

function buildCase(draw, controller) {
  const next = structuredClone(base);
  next.simulation.use_rerun = false;
  next.simulation.rerun_spawn_viewer = false;
  next.simulation.rerun_save_rrd = false;
  next.simulation.controller = controller;
  Object.assign(next.simulation, {
    wind_x_velocity: draw.wind_x_velocity,
    wind_y_velocity: draw.wind_y_velocity,
    gust_magnitude: draw.gust_magnitude,
    thrust_max: draw.thrust_max,
    initial_x_pos: draw.initial_x_pos,
    initial_y_pos: draw.initial_y_pos,
    initial_z_velocity: draw.initial_z_velocity,
  });
  return next;
}

function scoreLast(last) {
  if (last == null) {
    return {
      ok: false,
      miss: null,
      vz: null,
      fuel: null,
      t: null,
      z: null,
      reason: "no sample",
    };
  }
  const miss = Math.hypot(last.x - last.padX, last.y - last.padY);
  const vz = last.vz;
  let reason = "ok";
  let ok = true;
  if (last.z > SUCCESS.maxZ) {
    ok = false;
    reason = "did not land";
  } else if (Math.abs(vz) > SUCCESS.maxVz) {
    ok = false;
    reason = "hard landing";
  } else if (miss > SUCCESS.maxMiss) {
    ok = false;
    reason = "missed pad";
  } else if (last.t > SUCCESS.maxT) {
    ok = false;
    reason = "too slow";
  }
  return {
    ok,
    miss,
    vz,
    fuel: last.fuel,
    t: last.t,
    z: last.z,
    reason,
  };
}

function runOne(binary, caseObj) {
  return new Promise((resolve) => {
    const child = spawn(binary, [], {
      cwd: root,
      stdio: ["pipe", "pipe", "pipe"],
    });

    let lastLine = "";
    let carry = "";
    let stderr = "";

    child.stdout.setEncoding("utf8");
    child.stderr.setEncoding("utf8");
    child.stdout.on("data", (chunk) => {
      carry += chunk;
      let nl;
      while ((nl = carry.indexOf("\n")) >= 0) {
        const line = carry.slice(0, nl).trim();
        carry = carry.slice(nl + 1);
        if (line.length > 0) lastLine = line;
      }
    });
    child.stderr.on("data", (chunk) => {
      stderr += chunk;
    });

    const timer = setTimeout(() => {
      child.kill("SIGTERM");
    }, 90_000);

    child.on("close", () => {
      clearTimeout(timer);
      if (carry.trim().length > 0) lastLine = carry.trim();
      if (lastLine.length === 0) {
        resolve({
          last: null,
          score: scoreLast(null),
          detail: stderr.slice(0, 200),
        });
        return;
      }
      try {
        const last = JSON.parse(lastLine);
        resolve({ last, score: scoreLast(last), detail: "" });
      } catch (err) {
        resolve({
          last: null,
          score: { ...scoreLast(null), reason: `bad json: ${err.message}` },
          detail: lastLine.slice(0, 120),
        });
      }
    });

    child.stdin.write(JSON.stringify(caseObj));
    child.stdin.end();
  });
}

async function mapPool(items, jobs, worker) {
  const results = new Array(items.length);
  let next = 0;

  async function pump() {
    while (true) {
      const i = next++;
      if (i >= items.length) return;
      results[i] = await worker(items[i], i);
    }
  }

  const runners = [];
  for (let j = 0; j < Math.min(jobs, items.length); ++j) {
    runners.push(pump());
  }
  await Promise.all(runners);
  return results;
}

function summarize(label, rows) {
  const okRows = rows.filter((r) => r.score.ok);
  const misses = okRows.map((r) => r.score.miss).filter((x) => x != null);
  const vzAbs = okRows.map((r) => Math.abs(r.score.vz)).filter((x) => x != null);
  const fuels = okRows.map((r) => r.score.fuel).filter((x) => x != null);
  const rate = rows.length === 0 ? 0 : okRows.length / rows.length;

  return {
    controller: label,
    n: rows.length,
    successes: okRows.length,
    success_rate: rate,
    mean_miss: mean(misses),
    p90_miss: percentile(misses, 90),
    mean_abs_vz: mean(vzAbs),
    p90_abs_vz: percentile(vzAbs, 90),
    mean_fuel: mean(fuels),
  };
}

function printSummary(s) {
  const pct = (100 * s.success_rate).toFixed(1);
  console.log(
    `${s.controller.padEnd(4)}  ${s.successes}/${s.n} (${pct}%)  ` +
      `miss mean=${s.mean_miss.toFixed(1)} p90=${s.p90_miss.toFixed(1)}  ` +
      `|vz| mean=${s.mean_abs_vz.toFixed(2)} p90=${s.p90_abs_vz.toFixed(2)}  ` +
      `fuel mean=${s.mean_fuel.toFixed(1)}`
  );
}

const { n, seed, jobs, binary } = parseArgs(process.argv.slice(2));
const rng = mulberry32(seed);
const sim = base.simulation;

const draws = [];
for (let i = 0; i < n; ++i) {
  draws.push({ id: i, ...makeDraw(rng, sim) });
}

const controllers = ["pid", "mpc"];
const jobs_list = [];
for (const draw of draws) {
  for (const controller of controllers) {
    jobs_list.push({ draw, controller });
  }
}

console.log(
  `Monte Carlo: n=${n} seed=${seed} jobs=${jobs} controllers=${controllers.join(",")} binary=${binary}`
);

let done = 0;
const total = jobs_list.length;
const t0 = Date.now();

const runResults = await mapPool(jobs_list, jobs, async (job) => {
  const caseObj = buildCase(job.draw, job.controller);
  const out = await runOne(binary, caseObj);
  done += 1;
  if (done % Math.max(1, Math.floor(total / 10)) === 0 || done === total) {
    const elapsed = ((Date.now() - t0) / 1000).toFixed(1);
    process.stdout.write(`  ${done}/${total} (${elapsed}s)\n`);
  }
  return {
    id: job.draw.id,
    controller: job.controller,
    draw: {
      wind_x_velocity: job.draw.wind_x_velocity,
      wind_y_velocity: job.draw.wind_y_velocity,
      gust_magnitude: job.draw.gust_magnitude,
      thrust_max: job.draw.thrust_max,
      thrust_scale: job.draw.thrust_scale,
      initial_x_pos: job.draw.initial_x_pos,
      initial_y_pos: job.draw.initial_y_pos,
      initial_z_velocity: job.draw.initial_z_velocity,
    },
    score: out.score,
  };
});

const byController = {};
for (const c of controllers) {
  byController[c] = runResults.filter((r) => r.controller === c);
}

console.log("");
const summaries = controllers.map((c) => summarize(c, byController[c]));
for (const s of summaries) printSummary(s);

const outDir = join(root, "src", "results", "mc");
mkdirSync(outDir, { recursive: true });
const outPath = join(outDir, `mc_${seed}_${n}.json`);
const payload = {
  seed,
  n,
  jobs,
  success_gate: SUCCESS,
  binary,
  elapsed_s: (Date.now() - t0) / 1000,
  summaries,
  runs: runResults,
};
writeFileSync(outPath, JSON.stringify(payload, null, 2));
console.log(`\nwrote ${outPath}`);
