import type { ChartAsk } from "../localChat";

export type StoredMessage = {
  role: "user" | "assistant";
  text: string;
  chart?: ChartAsk;
  failed?: boolean;
  actions?: { time?: number; chart?: ChartAsk };
};

const PREFIX = "icarus-ask:";

function keyForPath(path: string): string {
  return PREFIX + path;
}

export function loadThread(path: string): StoredMessage[] {
  try {
    const raw = sessionStorage.getItem(keyForPath(path));
    if (!raw) return [];
    const parsed = JSON.parse(raw) as StoredMessage[];
    if (!Array.isArray(parsed)) return [];
    return parsed.filter((item) => item.role === "user" || item.role === "assistant");
  } catch {
    return [];
  }
}

export function saveThread(path: string, messages: StoredMessage[]): void {
  const stored = messages.filter(
    (item) => item.role === "user" || (item.role === "assistant" && item.text.trim().length > 0),
  );
  try {
    if (stored.length === 0) sessionStorage.removeItem(keyForPath(path));
    else sessionStorage.setItem(keyForPath(path), JSON.stringify(stored));
  } catch {
    // Quota or private mode; chat still works in memory.
  }
}

export function clearThread(path: string): void {
  try {
    sessionStorage.removeItem(keyForPath(path));
  } catch {
    // ignore
  }
}
