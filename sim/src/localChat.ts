import { chartColumns, clampNamed, columnName, columnWords, rowAt, rowsFor, sceneById, springHeight, type Column, type Knobs, type KnobKey, type SceneId } from "./scenes";

export type ChartAsk = { title: string; y: Column[]; x?: Column };

export type LocalTurn = {
  text: string;
  time?: number;
  chart?: ChartAsk;
  knob?: { key: KnobKey; value: number };
};

function wants(text: string, words: string[]): boolean {
  const lower = text.toLowerCase();
  return words.some((word) => lower.includes(word));
}

function readNumber(text: string): number | undefined {
  const match = text.match(/(-?\d+(?:\.\d+)?)/);
  if (!match) return undefined;
  return Number(match[1]);
}

function plotTurn(text: string, scene: SceneId): LocalTurn {
  const spec = sceneById(scene);
  const allowed = chartColumns(scene);
  const picked = allowed.filter((column) => mentioned(text, column));
  if (scene === "state" && wants(text, ["phase", "phase plane", "against rate", "height against rate"])) {
    return {
      text: "Here is the state as a phase plot: rate against height. A point on this loop is the whole spring state.",
      chart: { title: "Phase plane", x: "height", y: ["rate"] },
    };
  }
  const columns = picked.length ? picked : spec.live;
  const title = columns.length === spec.live.length ? spec.chartTitle : columns.map(labelOf).join(" and ");
  return {
    text: "Here is that series across the whole run. The gold line is the time you are on.",
    chart: { title, y: columns },
  };
}

function mentioned(text: string, column: Column): boolean {
  return wants(text, columnWords(column));
}

function labelOf(column: Column): string {
  return columnName(column);
}

export function localTurn(text: string, time: number, scene: SceneId, knobs: Knobs, question = ""): LocalTurn {
  const lower = text.toLowerCase();
  const number = readNumber(text);

  if (wants(lower, ["plot", "chart", "graph", "show"])) {
    return plotTurn(lower, scene);
  }

  if (/\b(who are you|what are you|your name|what is icarus|what is this page|what is this site)\b/.test(lower)) {
    return {
      text: "I am Icarus. Rocket Dynamics is where you learn rocket dynamics fundamentals. Simulator is for vertical landing sims and says Coming Soon on the home page. Ask me about the lesson you are on, or the ideas behind it.",
    };
  }

  if (question && /\b(the answer|correct|am i right|is it right)\b/.test(lower)) {
    return { text: "That one is graded in the card. I can still define a symbol, move the time, or plot a series." };
  }

  if (number !== undefined && scene === "step" && wants(lower, ["step", "dt"])) {
    const value = clampNamed("dt", number);
    return {
      text: `The step is $${value.toFixed(2)}\\,\\mathrm{s}$. Play it and compare the gold mass with the wire box.`,
      knob: { key: "dt", value },
    };
  }

  if (number !== undefined && scene === "mass" && wants(lower, ["thrust", "newton", "force"])) {
    const value = clampNamed("thrust", number);
    return {
      text: `Thrust is $${value.toFixed(0)}\\,\\mathrm{N}$ until four seconds, then it stops.`,
      knob: { key: "thrust", value },
    };
  }

  if (number !== undefined && scene === "engine" && wants(lower, ["lag", "tau"])) {
    const value = clampNamed("tau", number);
    return {
      text: `The lag $\\tau$ is $${value.toFixed(2)}\\,\\mathrm{s}$. Watch how long the gold line takes to follow the pale one.`,
      knob: { key: "tau", value },
    };
  }

  if (number !== undefined && scene === "engine" && wants(lower, ["ask", "command"])) {
    const value = clampNamed("ask", number);
    return {
      text: `The command asks for $${value.toFixed(0)}\\,\\mathrm{N}$ after $0.4\\,\\mathrm{s}$. The gold line is what the engine actually does.`,
      knob: { key: "ask", value },
    };
  }

  if (scene === "state" && wants(lower, ["top", "bottom", "turn around"])) {
    const bottom = lower.includes("bottom");
    return {
      text: bottom
        ? "You are at the bottom of the swing. Read the rate beside the slider."
        : "You are at the top of the swing. Read the rate beside the slider.",
      time: bottom ? Math.PI : 0,
    };
  }

  if (number !== undefined && wants(lower, ["second", "time", " at "])) {
    return { text: explain(scene, number, knobs), time: number };
  }

  // Readout at the current slider time, not general fundamentals (those need Grok).
  if (
    wants(lower, ["right now", "at this time", "beside the slider", "readout", "on the chart"]) ||
    (wants(lower, ["what is the", "what's the"]) && wants(lower, ["height", "rate", "mass", "thrust", "accel", "line"]))
  ) {
    return { text: explain(scene, time, knobs) };
  }

  if (wants(lower, ["set", "go to", "move", "scrub", "jump"])) {
    if (number === undefined) return { text: "Give me a time in seconds, like 1.5." };
    return { text: `Time is $${number.toFixed(2)}\\,\\mathrm{s}$.`, time: number };
  }

  return { text: sceneById(scene).opener };
}

function explain(scene: SceneId, time: number, knobs: Knobs): string {
  if (scene === "state") {
    const height = springHeight(time);
    return `$t = ${time.toFixed(2)}\\,\\mathrm{s}$. Height from rest is $${height.toFixed(2)}\\,\\mathrm{m}$. The rate $v$ is how fast that height is changing. Read it beside the slider.`;
  }
  if (scene === "step") {
    return `The step is $${knobs.dt.toFixed(2)}\\,\\mathrm{s}$. At $t = ${time.toFixed(2)}\\,\\mathrm{s}$, the gold mass is the stepped height and the wire box is the true height.`;
  }
  const row = rowAt(rowsFor(scene, knobs), time);
  if (scene === "mass") {
    return `Thrust input is $${knobs.thrust.toFixed(0)}\\,\\mathrm{N}$ while the tank has fuel. At $t = ${time.toFixed(2)}\\,\\mathrm{s}$, height is $${row.height.toFixed(2)}\\,\\mathrm{m}$ and rate is $${row.rate.toFixed(2)}\\,\\mathrm{m/s}$. Mass is $m = ${row.mass.toFixed(2)}\\,\\mathrm{kg}$.`;
  }
  return `At $t = ${time.toFixed(2)}\\,\\mathrm{s}$ the command is $${row.command.toFixed(0)}\\,\\mathrm{N}$ and thrust is $${row.actual.toFixed(0)}\\,\\mathrm{N}$. Height is $${row.height.toFixed(2)}\\,\\mathrm{m}$ and rate is $${row.rate.toFixed(2)}\\,\\mathrm{m/s}$. Lag is $\\tau = ${knobs.tau.toFixed(2)}\\,\\mathrm{s}$.`;
}
