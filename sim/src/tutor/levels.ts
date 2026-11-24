// What each tab is for. The first question for a block lives in qa/questions.json.
// Refresh asks Grok for a new one.

export const LEVELS = [
  {
    id: "beginner",
    label: "Beginner",
    aim: "Ask for one symbol, value, or visible relationship, and make the reader connect it to the picture or the chart. The question can carry the reason in the same sentence.",
  },
  {
    id: "intermediate",
    label: "Intermediate",
    aim: "Ask how an equation, the moving picture, and the chart fit together, and keep the question in sentences that hold the fact and what it does to the motion.",
  },
  {
    id: "advanced",
    label: "Advanced",
    aim: "Ask the reader to predict what changes if a knob moves or the clock advances, and stay with that prediction through the reason.",
  },
] as const;

export type LevelId = (typeof LEVELS)[number]["id"];

export function levelById(id: string) {
  return LEVELS.find((level) => level.id === id);
}
