import type { Column } from "./model";

export type ChartAsk = { title: string; y: Column[] };

export type LocalTurn = {
  text: string;
  time?: number;
  chart?: ChartAsk;
};

function wants(text: string, words: string[]): boolean {
  const lower = text.toLowerCase();
  return words.some((word) => lower.includes(word));
}

function readTime(text: string): number | undefined {
  const match = text.match(/(-?\d+(?:\.\d+)?)/);
  if (!match) return undefined;
  return Number(match[1]);
}

export function localTurn(text: string, time: number): LocalTurn {
  const lower = text.toLowerCase();

  if (wants(lower, ["plot", "chart", "graph", "show"])) {
    const wantRate = wants(lower, ["rate", "slope", "speed", "velocity"]);
    const wantHeight = wants(lower, ["height", "state", "position", "bead"]);
    let columns: Column[];
    if (wantRate && !wantHeight) columns = ["rate"];
    else if (wantHeight && !wantRate) columns = ["height"];
    else columns = ["height", "rate"];
    const title = columns.length === 2 ? "Height and rate" : columns[0] === "rate" ? "Rate" : "Height";
    return {
      text: "The samples are already in the buffer. Here is that series. The gold line is the time you are on.",
      chart: { title, y: columns },
    };
  }

  if (wants(lower, ["set", "go to", "move", "scrub", "jump"])) {
    const next = readTime(text);
    if (next === undefined) {
      return { text: "Give me a time in seconds, like 1.5." };
    }
    return { text: `Time is ${next.toFixed(2)} s. Look at where the bead is, and where the gold line sits on the chart.`, time: next };
  }

  if (wants(lower, ["zero", "top", "bottom", "turn"])) {
    return {
      text: "The rate is zero when the bead turns around. That is the top, near 1.57 s, and the bottom, near 4.71 s. The height is largest at the top and smallest at the bottom.",
      time: lower.includes("bottom") ? 4.71 : 1.57,
    };
  }

  if (wants(lower, ["what", "state", "rate", "height", "why"])) {
    return {
      text: `Right now t = ${time.toFixed(2)} s. Height is the state, in meters. Rate is how fast that height is changing, in meters per second. When the rate is positive the bead is rising.`,
    };
  }

  return {
    text: "Ask what the picture is doing, tell me a time to go to, or ask me to plot the height, the rate, or both.",
  };
}
