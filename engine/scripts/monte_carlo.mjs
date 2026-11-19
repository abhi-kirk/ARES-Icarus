#!/usr/bin/env node
// Local Monte Carlo: dispersed landings through IcarusStream, PID vs MPC.
// Lab tool only — runs on this machine, not on Fly.
//
// Usage:
//   node scripts/monte_carlo.mjs
//   node scripts/monte_carlo.mjs --scenario constrained --n 40 --seed 1
//   node scripts/monte_carlo.mjs --scenario storm --n 40 --seed 1 --jobs 8
//   node scripts/monte_carlo.mjs --scenario landing_bench --n 40 --seed 1
//   node scripts/monte_carlo.mjs --scenario moon --n 40 --seed 1
//   node scripts/monte_carlo.mjs --scenario moon --n 40 --seed 1 --controllers mpc,scvx

import { spawn } from "node:child_process";
import { cpus } from "node:os";
import { mkdirSync, readFileSync, writeFileSync } from "node:fs";
import { dirname, join } from "node:path";
import { fileURLToPath } from "node:url";

const root = join(dirname(fileURLToPath(import.meta.url)), "..");
const base = JSON.parse(readFileSync(join(root, "config.json"), "utf8"));
const FALCON_ENVELOPE = JSON.parse(
  readFileSync(join(root, "scripts", "data", "falcon_landing_envelope.json"), "utf8")
);
const WALLS = FALCON_ENVELOPE.academic_walls;
const PLANT = FALCON_ENVELOPE.plant_scale;

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
const G_EARTH = 9.81;
const G_MOON = 1.62;
// Same fraction of weight as the Earth band. 7.2–18 kN cannot hover at 1.62 m/s².
const MOON_SCALE = G_MOON / G_EARTH;

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
  // Falcon webcast landing-burn shape (tau = h/|v|) scaled into the plant hop,
  // plus the same academic throttle/cone walls. Horizontal/wind are academic
  // (webcast has no horiz fields). See scripts/data/falcon_landing_envelope.json.
  landing_bench: {
    label: "landing_bench",
    fixed: {
      glide_cone_deg: WALLS.glide_cone_deg,
      pointing_cone_deg: WALLS.pointing_cone_deg,
      thrust_max: WALLS.thrust_max_N,
      thrust_min: WALLS.thrust_min_N,
      initial_x_pos: 80,
      initial_y_pos: -40,
      initial_z_pos: PLANT.nominal_z_m,
      initial_z_velocity: PLANT.nominal_vz_mps,
      wind_x_velocity: 3.0,
      wind_y_velocity: 1.5,
      gust_magnitude: 2.0,
    },
    thrust_scale_min: 0.9,
    wind_sigma: 4.0,
    gust_sigma: 2.0,
    sample_falcon: true,
  },
  // Regime 2. The Moon: no drag, no wind, lunar gravity. The thrust
  // band is the Earth 7.2–18 kN band scaled by 1.62/9.81, so the floor
  // still sits under lunar weight. The Simulator flies this same 70 m by 40 m
  // divert on Earth. Here the tank is cut so the burn is propellant-limited.
  moon: {
    label: "moon",
    fixed: {
      glide_cone_deg: WALLS.glide_cone_deg,
      pointing_cone_deg: WALLS.pointing_cone_deg,
      gravity: G_MOON,
      thrust_max: THROTTLE_MAX * MOON_SCALE,
      thrust_min: THROTTLE_MIN * MOON_SCALE,
      initial_z_pos: PLANT.nominal_z_m,
      initial_z_velocity: PLANT.nominal_vz_mps,
      initial_x_velocity: 0.0,
      initial_y_velocity: 0.0,
      wind_x_velocity: 0.0,
      wind_y_velocity: 0.0,
      gust_magnitude: 0.0,
      drag_scale: 0.0,
      initial_fuel: 80.0,
      initial_pad_x_velocity: 0.0,
      initial_pad_y_velocity: 0.0,
      initial_x_pos: 70,
      initial_y_pos: -40,
    },
    thrust_scale_min: 1.0,
    wind_sigma: 0.0,
    gust_sigma: 0.0,
    sample_divert: true,
  },
};

function parseArgs(argv) {
  let n = 50;
  let seed = 1;
  let jobs = Math.max(1, cpus().length);
  let scenario = "default";
  let binary = join(root, "build", "IcarusStream");
  let controllers = ["pid", "mpc"];

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
    } else if (a === "--controllers" && argv[i + 1] != null) {
      controllers = argv[++i].split(",").map((s) => s.trim()).filter((s) => s.length > 0);
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
  return { n, seed, jobs, binary, scenario, controllers };
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
  const gravity = draw.gravity ?? G_EARTH;
  const hover = m * gravity;
  const thrustMax = draw.thrust_max;
  const thrustMin = draw.thrust_min ?? 0.0;
  const aBrakeMax = thrustMax / m - gravity;

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

// Sample a value from a committed percentile table (p10..p90).
function samplePercentiles(rng, dist) {
  const knots = [
    [0.1, dist.p10],
    [0.25, dist.p25],
    [0.5, dist.p50],
    [0.75, dist.p75],
    [0.9, dist.p90],
  ];
  const u = 0.1 + 0.8 * rng();
  for (let i = 0; i < knots.length - 1; ++i) {
    const [p0, v0] = knots[i];
    const [p1, v1] = knots[i + 1];
    if (u <= p1) {
      const a = (u - p0) / (p1 - p0);
      return v0 + a * (v1 - v0);
    }
  }
  return knots[knots.length - 1][1];
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

  let x0;
  let y0;
  let z0;
  let vz0;
  let falconMeta = null;
  let dragScale = sim.drag_scale ?? 1.0;
  let fuel = sim.initial_fuel ?? 600.0;
  let vx0 = sim.initial_x_velocity ?? 0.0;
  let vy0 = sim.initial_y_velocity ?? 0.0;

  if (scenarioCfg.sample_falcon) {
    // Scale Falcon landing-burn alt/tau into the plant hop. Horizontal is
    // academic: most draws sit inside the glide cone; a few sit outside so
    // the feasible/envelope split still maps the boundary.
    const falcon = FALCON_ENVELOPE.falcon;
    const altF = samplePercentiles(rng, falcon.landing_burn_alt_m);
    const tau = samplePercentiles(rng, falcon.tau_s);
    const scale = PLANT.nominal_z_m / PLANT.falcon_alt_ref_m;
    z0 = clamp(altF * scale, 200.0, 360.0);
    vz0 = clamp(-z0 / Math.max(8.0, tau), -40.0, -8.0);

    const glide = sim.glide_cone_deg ?? 20.0;
    const tanGlide = Math.tan((glide * Math.PI) / 180.0);
    const allow = z0 * tanGlide + GLIDE_PAD_RADIUS;
    // ~85% inside cone (uniform in disk), ~15% outside for envelope scoring.
    const inside = rng() < 0.85;
    const range = inside
      ? allow * Math.sqrt(rng()) * 0.9
      : allow * (1.05 + 0.35 * rng());
    const az = 2.0 * Math.PI * rng();
    x0 = range * Math.cos(az);
    y0 = range * Math.sin(az);
    falconMeta = {
      falcon_alt_m: altF,
      falcon_tau_s: tau,
      plant_scale: scale,
      range,
      allow,
      inside_glide_draw: inside,
    };
  } else if (scenarioCfg.sample_divert) {
    // Long divert that still sits inside the cone most of the time.
    // Fuel is the scarce resource; thrust is not also derated.
    dragScale = 0.0;
    // Probe at 100 kg: PID burned ~108 kg and arrived empty, MPC burned ~53 kg.
    // 80 kg sits between those two bills.
    fuel = clamp((sim.initial_fuel ?? 80.0) + 8.0 * randn(rng), 65.0, 95.0);
    vx0 = 0.0;
    vy0 = 0.0;
    z0 = sim.initial_z_pos;
    vz0 = clamp(sim.initial_z_velocity + 1.5 * randn(rng), -32.0, -12.0);
    const glide = sim.glide_cone_deg ?? 20.0;
    const tanGlide = Math.tan((glide * Math.PI) / 180.0);
    const allow = z0 * tanGlide + GLIDE_PAD_RADIUS;
    const inside = rng() < 0.85;
    const frac = inside ? 0.65 + 0.3 * rng() : 1.08 + 0.2 * rng();
    const range = allow * frac;
    const az = 2.0 * Math.PI * rng();
    x0 = range * Math.cos(az);
    y0 = range * Math.sin(az);
  } else {
    x0 = sim.initial_x_pos + 15.0 * randn(rng);
    y0 = sim.initial_y_pos + 15.0 * randn(rng);
    z0 = sim.initial_z_pos;
    vz0 = clamp(sim.initial_z_velocity + 3.0 * randn(rng), -40, -5);
  }

  const draw = {
    wind_x_velocity: windX,
    wind_y_velocity: windY,
    gust_magnitude: gust,
    thrust_max: thrustMax,
    thrust_scale: thrustScale,
    thrust_min: thrustMin,
    pointing_cone_deg: sim.pointing_cone_deg,
    glide_cone_deg: sim.glide_cone_deg,
    drag_scale: dragScale,
    gravity: sim.gravity ?? G_EARTH,
    initial_fuel: fuel,
    initial_x_velocity: vx0,
    initial_y_velocity: vy0,
    initial_x_pos: x0,
    initial_y_pos: y0,
    initial_z_pos: z0,
    initial_z_velocity: vz0,
  };
  if (falconMeta) draw.falcon = falconMeta;
  return draw;
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
    drag_scale: draw.drag_scale,
    initial_fuel: draw.initial_fuel,
    initial_x_velocity: draw.initial_x_velocity,
    initial_y_velocity: draw.initial_y_velocity,
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
  const used = okRows
    .map((r) => {
      const f0 = r.draw && r.draw.initial_fuel;
      if (f0 == null || r.score.fuel == null) return null;
      return f0 - r.score.fuel;
    })
    .filter((x) => x != null);
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
    mean_fuel_used: mean(used),
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
      `fuel_left mean=${s.mean_fuel.toFixed(1)}  fuel_used mean=${s.mean_fuel_used.toFixed(1)}${fails}`
  );
}

function pairedWins(runResults, leftName, rightName) {
  const byId = {};
  for (const r of runResults) {
    byId[r.id] = byId[r.id] || {};
    byId[r.id][r.controller] = r;
  }
  let miss = 0;
  let vz = 0;
  let fuel = 0;
  let bothOk = 0;
  let onlyLeft = 0;
  let onlyRight = 0;
  for (const id of Object.keys(byId)) {
    const left = byId[id][leftName];
    const right = byId[id][rightName];
    if (!left || !right) continue;
    if (left.score.ok && right.score.ok) {
      bothOk += 1;
      if (right.score.miss < left.score.miss) miss += 1;
      if (Math.abs(right.score.vz) < Math.abs(left.score.vz)) vz += 1;
      if (right.score.fuel > left.score.fuel) fuel += 1;
    } else if (left.score.ok && !right.score.ok) {
      onlyLeft += 1;
    } else if (!left.score.ok && right.score.ok) {
      onlyRight += 1;
    }
  }
  return {
    left: leftName,
    right: rightName,
    bothOk,
    onlyLeft,
    onlyRight,
    right_better_miss: miss,
    right_better_vz: vz,
    right_better_fuel: fuel,
  };
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

const { n, seed, jobs, binary, scenario, controllers } = parseArgs(process.argv.slice(2));
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

const jobs_list = [];
for (const draw of draws) {
  for (const controller of controllers) {
    jobs_list.push({ draw, controller });
  }
}

console.log(
  `Monte Carlo: scenario=${scenario} n=${n} seed=${seed} jobs=${jobs} ` +
    `glide=${sim.glide_cone_deg}° point=${sim.pointing_cone_deg}° ` +
    `drag=${sim.drag_scale ?? 1} fuel0=${sim.initial_fuel ?? 600} ` +
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
      drag_scale: job.draw.drag_scale,
      initial_fuel: job.draw.initial_fuel,
      initial_x_pos: job.draw.initial_x_pos,
      initial_y_pos: job.draw.initial_y_pos,
      initial_z_pos: job.draw.initial_z_pos,
      initial_z_velocity: job.draw.initial_z_velocity,
      falcon: job.draw.falcon ?? null,
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
const pairs = [];
if (controllers.includes("pid") && controllers.includes("mpc")) {
  pairs.push(pairedWins(feasibleResults, "pid", "mpc"));
}
if (controllers.includes("mpc") && controllers.includes("scvx")) {
  pairs.push(pairedWins(feasibleResults, "mpc", "scvx"));
}
for (const paired of pairs) {
  console.log(
    `paired ${paired.left} vs ${paired.right}: both_ok=${paired.bothOk} ` +
      `only_${paired.left}=${paired.onlyLeft} only_${paired.right}=${paired.onlyRight} ` +
      `${paired.right}_better_miss=${paired.right_better_miss} ` +
      `${paired.right}_better_vz=${paired.right_better_vz} ` +
      `${paired.right}_better_fuel=${paired.right_better_fuel}`
  );
}

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
const controllerTag = controllers.join("-");
const outName =
  controllerTag === "pid-mpc"
    ? `mc_${scenario}_${seed}_${n}.json`
    : `mc_${scenario}_${seed}_${n}_${controllerTag}.json`;
const outPath = join(outDir, outName);
const payload = {
  scenario,
  scenario_fixed: scenarioCfg.fixed,
  falcon_envelope:
    scenarioCfg.sample_falcon
      ? {
          n_landings: FALCON_ENVELOPE.source.n_landings,
          plant_scale: FALCON_ENVELOPE.plant_scale,
          falcon_tau_s: FALCON_ENVELOPE.falcon.tau_s,
          falcon_alt_m: FALCON_ENVELOPE.falcon.landing_burn_alt_m,
        }
      : null,
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
  controllers,
  paired_feasible: pairs,
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
