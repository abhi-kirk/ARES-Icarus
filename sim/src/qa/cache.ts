// Questions stay in the browser for an hour. A page reload reads them back.
// The refresh button writes over the one it replaces.

import type { LevelId } from "../tutor/levels";

const KEY = "icarus-qa-v1";
const HOUR = 60 * 60 * 1000;

type Entry = { prompt: string; at: number };
type Store = Record<string, Entry>;

function read(): Store {
  try {
    const raw = localStorage.getItem(KEY);
    if (!raw) return {};
    const parsed = JSON.parse(raw) as Store;
    if (!parsed || typeof parsed !== "object") return {};
    return parsed;
  } catch {
    return {};
  }
}

function write(store: Store) {
  try {
    localStorage.setItem(KEY, JSON.stringify(store));
  } catch {
    // Private mode, or a full disk. The card still works for this visit.
  }
}

function slot(block: string, level: LevelId) {
  return `${block}:${level}`;
}

export function cachedPrompt(block: string, level: LevelId): string {
  const entry = read()[slot(block, level)];
  if (!entry || typeof entry.prompt !== "string" || entry.prompt === "") return "";
  if (Date.now() - entry.at > HOUR) return "";
  return entry.prompt;
}

export function rememberPrompt(block: string, level: LevelId, prompt: string) {
  if (!prompt) return;
  const store = read();
  const now = Date.now();
  for (const key of Object.keys(store)) {
    const entry = store[key];
    if (!entry || now - entry.at > HOUR) delete store[key];
  }
  store[slot(block, level)] = { prompt, at: now };
  write(store);
}
