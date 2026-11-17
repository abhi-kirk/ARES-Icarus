#!/usr/bin/env node
// Local Monte Carlo: dispersed landings through IcarusStream, PID vs MPC.
// Lab tool only — runs on this machine, not on Fly.
//
// Usage:
//   node scripts/monte_carlo.mjs
//   node scripts/monte_carlo.mjs --scenario constrained --n 40 --seed 1
//   node scripts/monte_carlo.mjs --scenario storm --n 40 --seed 1 --jobs 8

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

// Scenarios pick the fixed case envelope. Draws still jitter wind/state/thrust.
// Throttle band is engine-like: floor ~40% of ceiling (cannot command zero
// during the burn), ceiling near soft-land demand for this plant.
const THROTTLE_MAX = 18000.0;
const THROTTLE_MIN = 0.4 * THROTTLE_MAX; // 7200 N

const SCENARIOS = {
  // Open cones, comfortable thrust — baseline from make mc.
  default: {
    label: "default",
    fixed: {},
    thrust_scale_min: 0.7,
    wind_sigma: 5.0,
    gust_sigma: 2.0,
  },
  // Real throttle band + tighter keep-out + pointing limit.
  constrained: {
    label: "constrained",
    fixed: {
      glide_cone_deg: 20.0,
      pointing_cone_deg: 25.0,
      thrust_max: THROTTLE_MAX,
      thrust_min: THROTTLE_MIN,
      initial_x_pos: 100,
      initial_y_pos: -60,
      initial_z_pos: 280,
      initial_z_velocity: -22,
    },
    thrust_scale_min: 0.9,
    wind_sigma: 4.0,
    gust_sigma: 2.0,
  },
  // Same throttle/cones, stronger unmodeled wind/gust.
  storm: {
    label: "storm",
    fixed: {
      glide_cone_deg: 20.0,
      pointing_cone_deg: 25.0,
      thrust_max: THROTTLE_MAX,
      thrust_min: THROTTLE_MIN,
      initial_x_pos: 100,
      initial_y_pos: -60,
      initial_z_pos: 280,
      initial_z_velocity: -22,
      wind_x_velocity: 8.0,
      wind_y_velocity: 4.0,
      gust_magnitude: 5.0,
    },
    thrust_scale_min: 0.9,
    wind_sigma: 6.0,
    gust_sigma: 3.0,
  },
};

function parseArgs(argv) {
  let n = 50;
  let seed = 1;
  let jobs = Math.max(1, cpus().length);
  let scenario = "default";
  let binary = join(root, "build", "IcarusStream");

  for (let i = 0; i < argv.length; ++i) {
    const a = argv[i];
    if (a === "--n" && argv[i + 1] != null) {
      n = Math.max(1, parseInt(argv[++i], 10) || n);
    } else if (a === "--seed" && argv[i + 1] != null) {
      seed = parseInt(argv[++i], 10) || seed;
    } else if (a === "--jobs" && argv[i + 1] != null) {
      jobs = Math.max(1, parseInt(argv[++i], 10) || jobs);
    } else if (a === "--scenario" && argv[i + 1] != null) {
      scenario = argv[++i];
    } else if (!a.startsWith("-")) {
      binary = a;
    }
  }
  if (!SCENARIOS[scenario]) {
    console.error(
      `unknown scenario "${scenario}"; use ${Object.keys(SCENARIOS).join("|")}`
    );
    process.exit(2);
  }
  return { n, seed, jobs, binary, scenario };
}

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

const MASS_DRY = 1000.0;
const GRAVITY = 9.81;
const GLIDE_PAD_RADIUS = 5.0;

function baseSim(scenario) {
  const sim = { ...base.simulation, ...SCENARIOS[scenario].fixed };
  return sim;
}

// Open-loop IC check: is there any chance a constrained soft land exists?
// Outside the glide cone or a dive that max throttle cannot brake are
// case failures, not controller failures (SCvx would be infeasible too).
function classifyDraw(draw, fuel) {
  const z = Math.max(1.0, draw.initial_z_pos);
  const vz = draw.initial_z_velocity;
  const range = Math.hypot(draw.initial_x_pos, draw.initial_y_pos);
  const glide = draw.glide_cone_deg ?? 90.0;
  const tanGlide = glide >= 89.9 ? 1.0e6 : Math.tan((glide * Math.PI) / 180.0);
  const allow = z * tanGlide + GLIDE_PAD_RADIUS;
  const inside_glide = range <= allow;

  const m = MASS_DRY + Math.max(0.0, fuel);
  const hover = m * GRAVITY;
  const thrustMax = draw.thrust_max;
  const thrustMin = draw.thrust_min ?? 0.0;
  const aBrakeMax = thrustMax / m - GRAVITY;

  const reasons = [];
  if (!inside_glide) reasons.push("outside_glide");
  if (thrustMax < hover) reasons.push("cannot_hover");
  if (thrustMin > hover) reasons.push("floor_above_hover");
  if (aBrakeMax <= 0.0) {
    reasons.push("cannot_brake");
  } else if (vz < 0.0 && vz * vz > 2.0 * aBrakeMax * z) {
    // Constant max-brake from here still hits the ground with leftover rate.
    reasons.push("insufficient_brake");
  }

  return {
    feasible: reasons.length === 0,
    reasons,
    inside_glide,
    range,
    allow,
    hover,
    a_brake_max: aBrakeMax,
  };
}

function makeDraw(rng, sim, scenarioCfg) {
  const windX = clamp(
    sim.wind_x_velocity + scenarioCfg.wind_sigma * randn(rng),
    -25,
    25
  );
  const windY = clamp(
    sim.wind_y_velocity + scenarioCfg.wind_sigma * randn(rng),
    -25,
    25
  );
  const gust = clamp(
    sim.gust_magnitude + scenarioCfg.gust_sigma * randn(rng),
    0,
    18
  );
  const thrustScale = clamp(
    1.0 - 0.5 * (1.0 - scenarioCfg.thrust_scale_min) * Math.abs(randn(rng)),
    scenarioCfg.thrust_scale_min,
    1.0
  );
  const thrustMin = sim.thrust_min ?? 0.0;
  // Ceiling shortfall cannot collapse the throttle band; keep a 5% gap.
  let thrustMax = sim.thrust_max * thrustScale;
  if (thrustMin > 0.0) {
    thrustMax = Math.max(thrustMax, thrustMin * 1.05);
  }
  const x0 = sim.initial_x_pos + 15.0 * randn(rng);
  const y0 = sim.initial_y_pos + 15.0 * randn(rng);
  const vz0 = clamp(sim.initial_z_velocity + 3.0 * randn(rng), -40, -5);

  return {
    wind_x_velocity: windX,
    wind_y_velocity: windY,
    gust_magnitude: gust,
    thrust_max: thrustMax,
    thrust_scale: thrustScale,
    thrust_min: thrustMin,
    pointing_cone_deg: sim.pointing_cone_deg,
    glide_cone_deg: sim.glide_cone_deg,
    initial_x_pos: x0,
    initial_y_pos: y0,
    initial_z_pos: sim.initial_z_pos,
    initial_z_velocity: vz0,
  };
}

function buildCase(draw, controller, scenarioCfg) {
  const next = structuredClone(base);
  next.simulation.use_rerun = false;
  next.simulation.rerun_spawn_viewer = false;
  next.simulation.rerun_save_rrd = false;
  next.simulation.controller = controller;
  Object.assign(next.simulation, scenarioCfg.fixed, {
    wind_x_velocity: draw.wind_x_velocity,
    wind_y_velocity: draw.wind_y_velocity,
    gust_magnitude: draw.gust_magnitude,
    thrust_max: draw.thrust_max,
    thrust_min: draw.thrust_min,
    pointing_cone_deg: draw.pointing_cone_deg,
    glide_cone_deg: draw.glide_cone_deg,
    initial_x_pos: draw.initial_x_pos,
    initial_y_pos: draw.initial_y_pos,
    initial_z_pos: draw.initial_z_pos,
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

    // Horizon-50 OSQP under a full job pool can push a single landing past
    // 90 s wall; killing mid-flight falsely scores "did not land".
    const timer = setTimeout(() => {
      child.kill("SIGTERM");
    }, 180_000);

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
  const reasons = {};
  for (const r of rows) {
    if (r.score.ok) continue;
    reasons[r.score.reason] = (reasons[r.score.reason] || 0) + 1;
  }

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
    fail_reasons: reasons,
  };
}

function printSummary(s) {
  const pct = (100 * s.success_rate).toFixed(1);
  const fails =
    Object.keys(s.fail_reasons).length === 0
      ? ""
      : `  fails=${JSON.stringify(s.fail_reasons)}`;
  console.log(
    `${s.controller.padEnd(4)}  ${s.successes}/${s.n} (${pct}%)  ` +
      `miss mean=${s.mean_miss.toFixed(1)} p90=${s.p90_miss.toFixed(1)}  ` +
      `|vz| mean=${s.mean_abs_vz.toFixed(2)} p90=${s.p90_abs_vz.toFixed(2)}  ` +
      `fuel mean=${s.mean_fuel.toFixed(1)}${fails}`
  );
}

function pairedWins(runResults) {
  const byId = {};
  for (const r of runResults) {
    byId[r.id] = byId[r.id] || {};
    byId[r.id][r.controller] = r;
  }
  let miss = 0;
  let vz = 0;
  let fuel = 0;
  let bothOk = 0;
  let onlyPid = 0;
  let onlyMpc = 0;
  for (const id of Object.keys(byId)) {
    const p = byId[id].pid;
    const m = byId[id].mpc;
    if (!p || !m) continue;
    if (p.score.ok && m.score.ok) {
      bothOk += 1;
      if (m.score.miss < p.score.miss) miss += 1;
      if (Math.abs(m.score.vz) < Math.abs(p.score.vz)) vz += 1;
      if (m.score.fuel > p.score.fuel) fuel += 1;
    } else if (p.score.ok && !m.score.ok) {
      onlyPid += 1;
    } else if (!p.score.ok && m.score.ok) {
      onlyMpc += 1;
    }
  }
  return { bothOk, onlyPid, onlyMpc, mpc_better_miss: miss, mpc_better_vz: vz, mpc_better_fuel: fuel };
}

function countReasons(classifications) {
  const counts = {};
  for (const c of classifications) {
    for (const r of c.reasons) {
      counts[r] = (counts[r] || 0) + 1;
    }
  }
  return counts;
}

const { n, seed, jobs, binary, scenario } = parseArgs(process.argv.slice(2));
const scenarioCfg = SCENARIOS[scenario];
const rng = mulberry32(seed);
const sim = baseSim(scenario);
const fuel0 = sim.initial_fuel ?? 600.0;

const draws = [];
for (let i = 0; i < n; ++i) {
  const raw = makeDraw(rng, sim, scenarioCfg);
  const ic = classifyDraw(raw, fuel0);
  draws.push({ id: i, ...raw, ic });
}

const feasibleDraws = draws.filter((d) => d.ic.feasible);
const envelopeDraws = draws.filter((d) => !d.ic.feasible);

const controllers = ["pid", "mpc"];
const jobs_list = [];
for (const draw of draws) {
  for (const controller of controllers) {
    jobs_list.push({ draw, controller });
  }
}

console.log(
  `Monte Carlo: scenario=${scenario} n=${n} seed=${seed} jobs=${jobs} ` +
    `glide=${sim.glide_cone_deg}° point=${sim.pointing_cone_deg}° ` +
    `thrust=[${sim.thrust_min ?? 0}, ${sim.thrust_max}] N binary=${binary}`
);
console.log(
  `IC split: feasible=${feasibleDraws.length}/${n}  envelope=${envelopeDraws.length}/${n}` +
    (envelopeDraws.length
      ? `  envelope_reasons=${JSON.stringify(countReasons(envelopeDraws.map((d) => d.ic)))}`
      : "")
);

let done = 0;
const total = jobs_list.length;
const t0 = Date.now();

const runResults = await mapPool(jobs_list, jobs, async (job) => {
  const caseObj = buildCase(job.draw, job.controller, scenarioCfg);
  const out = await runOne(binary, caseObj);
  done += 1;
  if (done % Math.max(1, Math.floor(total / 10)) === 0 || done === total) {
    const elapsed = ((Date.now() - t0) / 1000).toFixed(1);
    process.stdout.write(`  ${done}/${total} (${elapsed}s)\n`);
  }
  return {
    id: job.draw.id,
    controller: job.controller,
    feasible_ic: job.draw.ic.feasible,
    ic: job.draw.ic,
    draw: {
      wind_x_velocity: job.draw.wind_x_velocity,
      wind_y_velocity: job.draw.wind_y_velocity,
      gust_magnitude: job.draw.gust_magnitude,
      thrust_max: job.draw.thrust_max,
      thrust_min: job.draw.thrust_min,
      thrust_scale: job.draw.thrust_scale,
      pointing_cone_deg: job.draw.pointing_cone_deg,
      glide_cone_deg: job.draw.glide_cone_deg,
      initial_x_pos: job.draw.initial_x_pos,
      initial_y_pos: job.draw.initial_y_pos,
      initial_z_pos: job.draw.initial_z_pos,
      initial_z_velocity: job.draw.initial_z_velocity,
    },
    score: out.score,
  };
});

const feasibleResults = runResults.filter((r) => r.feasible_ic);
const envelopeResults = runResults.filter((r) => !r.feasible_ic);

console.log("\n--- Controller score (feasible ICs only) ---");
const summariesFeasible = controllers.map((c) =>
  summarize(
    c,
    feasibleResults.filter((r) => r.controller === c)
  )
);
for (const s of summariesFeasible) printSummary(s);
const pairedFeasible = pairedWins(feasibleResults);
console.log(
  `paired: both_ok=${pairedFeasible.bothOk} only_pid=${pairedFeasible.onlyPid} only_mpc=${pairedFeasible.onlyMpc} ` +
    `mpc_better_miss=${pairedFeasible.mpc_better_miss} mpc_better_vz=${pairedFeasible.mpc_better_vz} ` +
    `mpc_better_fuel=${pairedFeasible.mpc_better_fuel}`
);

console.log("\n--- Envelope ICs (infeasible cases; not a controller score) ---");
console.log(
  `draws=${envelopeDraws.length}/${n}  reasons=${JSON.stringify(countReasons(envelopeDraws.map((d) => d.ic)))}`
);
if (envelopeResults.length > 0) {
  const summariesEnvelope = controllers.map((c) =>
    summarize(
      c,
      envelopeResults.filter((r) => r.controller === c)
    )
  );
  for (const s of summariesEnvelope) printSummary(s);
}

const outDir = join(root, "src", "results", "mc");
mkdirSync(outDir, { recursive: true });
const outPath = join(outDir, `mc_${scenario}_${seed}_${n}.json`);
const payload = {
  scenario,
  scenario_fixed: scenarioCfg.fixed,
  seed,
  n,
  jobs,
  success_gate: SUCCESS,
  binary,
  elapsed_s: (Date.now() - t0) / 1000,
  ic_split: {
    feasible: feasibleDraws.length,
    envelope: envelopeDraws.length,
    envelope_reasons: countReasons(envelopeDraws.map((d) => d.ic)),
  },
  summaries_feasible: summariesFeasible,
  paired_feasible: pairedFeasible,
  summaries_envelope:
    envelopeResults.length > 0
      ? controllers.map((c) =>
          summarize(
            c,
            envelopeResults.filter((r) => r.controller === c)
          )
        )
      : [],
  runs: runResults,
};
writeFileSync(outPath, JSON.stringify(payload, null, 2));
console.log(`\nwrote ${outPath}`);
