// What each tab is for. The questions themselves come from Grok, not from this file.

export const LEVELS = [
  {
    id: "beginner",
    label: "Beginner",
    aim: "Ask for one symbol or one number the reader can read off the picture or the equations. One sentence.",
  },
  {
    id: "intermediate",
    label: "Intermediate",
    aim: "Ask how two parts of the picture fit together. One or two sentences.",
  },
  {
    id: "advanced",
    label: "Advanced",
    aim: "Ask what changes if the reader moves a knob or waits. One or two sentences. Do not ask for a derivation.",
  },
] as const;

export type LevelId = (typeof LEVELS)[number]["id"];

export function levelById(id: string) {
  return LEVELS.find((level) => level.id === id);
}
