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
  turn: [
    "Why do we need more than one axis?",
    "What is gimbal lock?",
    "How does spin affect control?",
  ],
  vehicle: [
    "What does guidance do on the way down?",
    "How does wind push the rocket?",
    "What is an estimate in plain words?",
  ],
  descent: [
    "What is a path under constraints?",
    "Why replan during descent?",
    "What makes a landing miss the pad?",
  ],
};

export function startersForChapter(chapterId: ChapterId): string[] {
  return BY_CHAPTER[chapterId] ?? DEFAULT;
}
