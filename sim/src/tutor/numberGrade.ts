import { freqBode, type FreqKnobs } from "../frequencyPlant";
import { eqBurnout, lossScore, stageScore, type PropKnobs } from "../propellantPlant";
import { ascentWeight, noseBreakEvenDeg } from "../vehiclePlant";
import type { GradeResponse } from "./types";

/**
 * Grades a seed question that asks for a number the plant already computes.
 * A regenerated question keeps its own wording and falls through to Grok.
 * The verdict is the green check, a partial, or a miss. There is no score.
 */

type Target = {
  value: number;
  abs: number;
  name: string;
};

const FREQ: FreqKnobs = { gain: 11.2, damp: 5.5, lag: 0, sample: 0.2, bend: 8, notch: 0 };

function prop(over: Partial<PropKnobs>): PropKnobs {
  return {
    ratio: 4,
    isp: 300,
    expand: 16,
    split: 0.75,
    structure: 0.08,
    throttle: 1,
    ...over,
  };
}

function plain(text: string): string {
  return text
    .replace(/\$/g, "")
    .replace(/\\[a-zA-Z]+/g, " ")
    .replace(/[{}_^]/g, "")
    .replace(/\s+/g, " ")
    .trim()
    .toLowerCase();
}

function asks(question: string, parts: string[]): boolean {
  const text = plain(question);
  return parts.every((part) => text.includes(part));
}

function valuesIn(answer: string): number[] {
  const matches = answer.match(/-?\d+(?:\.\d+)?/g) ?? [];
  const values: number[] = [];
  for (const match of matches) {
    const value = Number(match);
    if (Number.isFinite(value)) values.push(value);
  }
  return values;
}

function score(
  answer: string,
  targets: Target[],
  attemptAbove: number,
  reason: RegExp | null,
  truth: string,
  reasonNote: string,
): GradeResponse | null {
  const values = valuesIn(answer);
  const pool = values.slice();
  const hit = targets.map((target) => {
    let best = -1;
    let bestGap = Infinity;
    for (let index = 0; index < pool.length; index += 1) {
      const gap = Math.abs(pool[index] - target.value);
      if (gap <= target.abs && gap < bestGap) {
        best = index;
        bestGap = gap;
      }
    }
    if (best < 0) return false;
    pool.splice(best, 1);
    return true;
  });

  const tried = values.some((value) => Math.abs(value) >= attemptAbove);
  const any = hit.some(Boolean);
  if (!any && !tried) return null;

  const all = hit.every(Boolean);
  const told = answer.toLowerCase();
  if (all && reason && !reason.test(told)) {
    return { verdict: "partial", note: reasonNote };
  }
  if (all) return { verdict: "right", note: truth };

  if (any) {
    const need = targets.filter((_, index) => !hit[index]).map((target) => target.name);
    return { verdict: "partial", note: `${truth} The answer still needs ${need.join(" and ")}.` };
  }
  return { verdict: "wrong", note: truth };
}

function gradeMargins(answer: string): GradeResponse | null {
  const margins = freqBode("freqMargin", FREQ).margins;
  if (margins.pm === null) return null;
  const phase = Math.round(margins.pm);
  const gain = margins.gmDb;
  const truth = `Phase margin is ${phase}° and gain margin is ${gain.toFixed(1)} dB at Kp 11.2 and Kd 5.5.`;
  return score(
    answer,
    [
      { value: margins.pm, abs: 2, name: "the phase margin" },
      { value: gain, abs: 0.4, name: "the gain margin" },
    ],
    0,
    null,
    truth,
    truth,
  );
}

function gradeDelay(answer: string): GradeResponse | null {
  const margins = freqBode("freqDelay", { ...FREQ, lag: 0.2, sample: 0.2 }).margins;
  const delay = Math.round(margins.delayDeg);
  const hold = Math.round(margins.holdDeg);
  const truth = `At 0.2 s each, the delay takes ${delay}° and the sample-and-hold takes ${hold}°.`;
  return score(
    answer,
    [
      { value: margins.delayDeg, abs: 2, name: "the delay" },
      { value: margins.holdDeg, abs: 2, name: "the sample-and-hold" },
    ],
    0,
    null,
    truth,
    truth,
  );
}

function gradeRocket(answer: string): GradeResponse | null {
  const end = eqBurnout(prop({ ratio: 4, isp: 300 }));
  const speed = Math.round(end.speed);
  const start = end.accel0;
  const finish = end.accel1;
  const truth = `At mass ratio 4 and Isp 300 s the final speed is ${speed} m/s, the acceleration is ${start.toFixed(0)} m/s² at ignition, and it is ${finish.toFixed(0)} m/s² at burnout.`;
  return score(
    answer,
    [
      { value: end.speed, abs: 40, name: "the final speed" },
      { value: start, abs: 15, name: "the ignition acceleration" },
      { value: finish, abs: 25, name: "the burnout acceleration" },
    ],
    10,
    null,
    truth,
    truth,
  );
}

function gradeRatioStep(answer: string): GradeResponse | null {
  const low = eqBurnout(prop({ ratio: 2, isp: 300 }));
  const high = eqBurnout(prop({ ratio: 4, isp: 300 }));
  const added = high.speed - low.speed;
  const shown = Math.round(added);
  const truth = `Raising the mass ratio from 2 to 4 adds ${shown} m/s, which is the exhaust speed times ln 2.`;
  return score(
    answer,
    [{ value: added, abs: 50, name: "the added speed" }],
    50,
    /\\ln\b|\bln\b|\\log\b|\blog\b/,
    truth,
    `That addition is ${shown} m/s. It is the exhaust speed times ln 2, because the mass ratio doubled.`,
  );
}

function gradeLoss(answer: string): GradeResponse | null {
  const scoreLine = lossScore(prop({ throttle: 1 }));
  const speed = Math.round(scoreLine.speed);
  const ideal = Math.round(scoreLine.ideal);
  const truth = `Full thrust burns out at ${speed} m/s, and the ideal free-space speed of this tank is ${ideal} m/s.`;
  return score(
    answer,
    [
      { value: scoreLine.speed, abs: 40, name: "the burnout speed" },
      { value: scoreLine.ideal, abs: 40, name: "the ideal free-space speed" },
    ],
    100,
    null,
    truth,
    truth,
  );
}

function gradeStages(answer: string): GradeResponse | null {
  const built = stageScore(prop({ split: 0.75, structure: 0.08 }));
  const first = Math.round(built.first);
  const second = Math.round(built.second);
  const single = Math.round(built.single);
  const truth = `At split 0.75 and structure 0.08 the first burn is ${first} m/s, the second is ${second} m/s, and the single-stage vehicle ends at ${single} m/s.`;
  return score(
    answer,
    [
      { value: built.first, abs: 30, name: "the first burn" },
      { value: built.second, abs: 30, name: "the second burn" },
      { value: built.single, abs: 30, name: "the single-stage speed" },
    ],
    100,
    null,
    truth,
    truth,
  );
}

function gradeWeight(answer: string): GradeResponse | null {
  const weight = ascentWeight();
  const shown = weight.toFixed(1);
  const truth = `The weight is ${shown} N. T/W is thrust divided by that weight, and a value above 1 is what speeds the vehicle upward.`;
  return score(
    answer,
    [{ value: weight, abs: 0.25, name: "the weight" }],
    1,
    /thrust|divid|t\/w|t\\\/w/,
    truth,
    `The weight is ${shown} N. T/W is thrust divided by that weight, and a value above 1 is what speeds the vehicle upward.`,
  );
}

function gradeTilt(answer: string): GradeResponse | null {
  const tilt = noseBreakEvenDeg();
  const shown = tilt.toFixed(1);
  const truth = `Vertical acceleration is zero at ${shown}°. Past that tilt the vehicle stays on the pad and the remaining thrust goes downrange.`;
  return score(
    answer,
    [{ value: tilt, abs: 1.5, name: "the tilt" }],
    10,
    /height|downrange|pad|climb|side/,
    truth,
    `Vertical acceleration is zero at ${shown}°. Past that tilt the vehicle stays on the pad and the remaining thrust goes downrange.`,
  );
}

/** Null when this question is not one of the numbered seeds, or the answer has no number yet. */
export function gradeByNumber(question: string, answer: string): GradeResponse | null {
  if (asks(question, ["11.2", "5.5", "phase margin", "gain margin"])) return gradeMargins(answer);
  if (asks(question, ["0.2", "how many degrees", "each"])) return gradeDelay(answer);
  if (asks(question, ["mass ratio 4", "300", "final speed"])) return gradeRocket(answer);
  if (asks(question, ["metres per second", "that add"])) return gradeRatioStep(answer);
  if (asks(question, ["burnout for full thrust", "ideal"])) return gradeLoss(answer);
  if (asks(question, ["two burn speeds", "0.75"])) return gradeStages(answer);
  if (asks(question, ["what is the weight", "t/w"])) return gradeWeight(answer);
  if (asks(question, ["at what tilt", "50"])) return gradeTilt(answer);
  return null;
}
