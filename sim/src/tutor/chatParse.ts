import { readJson } from "./grok";
import type { ChatReply } from "./types";

/** Best-effort extract of the text field while JSON is still streaming. */
export function chatDisplayFromPartial(raw: string): string {
  const start = raw.indexOf('"text"');
  if (start < 0) return "";
  const quote = raw.indexOf('"', start + 6);
  if (quote < 0) return "";
  let out = "";
  let escaped = false;
  for (let i = quote + 1; i < raw.length; i++) {
    const ch = raw[i];
    if (escaped) {
      if (ch === "n") out += "\n";
      else if (ch === "t") out += "\t";
      else out += ch;
      escaped = false;
      continue;
    }
    if (ch === "\\") {
      escaped = true;
      continue;
    }
    if (ch === '"') break;
    out += ch;
  }
  return out;
}

export function parseChatReply(raw: string): ChatReply {
  const parsed = readJson(raw) as Partial<ChatReply>;
  if (!parsed.text?.trim()) throw new Error("Grok did not return text");
  return {
    text: parsed.text.trim(),
    t: typeof parsed.t === "number" ? parsed.t : null,
    chart: parsed.chart ?? null,
  };
}
