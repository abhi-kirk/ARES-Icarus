import type { ChapterId } from "../catalog";

const DEFAULT = [
  "What is state in rocket motion?",
  "How do thrust and gravity interact?",
  "What should I look at on the chart?",
];

const BY_CHAPTER: Record<ChapterId, string[]> = {
  state: [
    "What is state in rocket motion?",
    "What is thrust compared to weight?",
    "Why does a bigger time step matter?",
  ],
  loop: [
    "What is an open loop?",
    "What is feedback for?",
    "Why can an unstable hover still land?",
  ],
  frequency: [
    "What is a phase margin?",
    "Why does a delay subtract phase?",
    "What does a notch do to a bending peak?",
  ],
  turn: [
    "Why do we need more than one axis?",
    "What is gimbal lock?",
    "How does spin affect control?",
  ],
  vehicle: [
    "What does guidance do on the way down?",
    "How does wind push the rocket?",
    "What is an estimate?",
  ],
  descent: [
    "What is a path under constraints?",
    "Why replan during descent?",
    "What makes a landing miss the pad?",
  ],
  propellant: [
    "What does the rocket equation say about final speed?",
    "Why does a wide bell lose thrust at the pad?",
    "Why do two stages finish faster than one?",
  ],
};

export function startersForChapter(chapterId: ChapterId): string[] {
  return BY_CHAPTER[chapterId] ?? DEFAULT;
}
