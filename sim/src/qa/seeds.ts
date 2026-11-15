// First questions for every block. Refresh still asks Grok and stores that
// result in the browser for an hour. A later database will keep generated
// questions per user. This file is the set the card can show before any of that.

import questions from "./questions.json";
import { READING } from "../reading";
import type { LevelId } from "../tutor/levels";

type LevelBank = Partial<Record<LevelId, string>>;
type Bank = Record<string, Record<string, LevelBank>>;

const bank = questions as Bank;

export function seedPrompt(scene: string, block: string | undefined, level: LevelId): string {
  if (!block) return "";
  const prompt = bank[scene]?.[block]?.[level];
  if (typeof prompt !== "string") return "";
  const text = prompt.trim();
  return text;
}

export function missingSeeds(): string[] {
  const missing: string[] = [];
  for (const [scene, page] of Object.entries(READING)) {
    for (const thread of page.threads) {
      const levels = thread.levels && thread.levels.length > 0
        ? thread.levels
        : (["beginner", "intermediate", "advanced"] as const);
      for (const level of levels) {
        if (!seedPrompt(scene, thread.id, level)) missing.push(`${scene}/${thread.id}/${level}`);
      }
    }
  }
  return missing;
}

if (import.meta.env.DEV) {
  const missing = missingSeeds();
  if (missing.length > 0) console.warn("Q&A seeds missing", missing);
}
