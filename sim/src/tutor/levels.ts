// What each tab is for. The questions themselves come from Grok, not from this file.

export const LEVELS = [
  {
    id: "beginner",
    label: "Beginner",
    aim: "Ask for one symbol, value, or visible relationship, and make the reader connect it to the picture or chart. One sentence.",
  },
  {
    id: "intermediate",
    label: "Intermediate",
    aim: "Ask how an equation, the moving picture, and the chart fit together. One or two sentences.",
  },
  {
    id: "advanced",
    label: "Advanced",
    aim: "Ask the reader to predict what changes if a knob moves or the clock advances. One or two sentences. Do not ask for a derivation.",
  },
] as const;

export type LevelId = (typeof LEVELS)[number]["id"];

export function levelById(id: string) {
  return LEVELS.find((level) => level.id === id);
}
