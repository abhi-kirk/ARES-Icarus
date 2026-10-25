// One call to the xAI Responses API. Question, grade, and chat all use this.
// grok-4.7 is the current public model. "low" is the fastest effort it allows.

const MODEL = "grok-4.7";

type GrokPayload = {
  output?: Array<{ content?: Array<{ text?: string }> }>;
};

export async function grokText(key: string, instructions: string, user: string, effort: "low" | "medium"): Promise<string> {
  const response = await fetch("https://api.x.ai/v1/responses", {
    method: "POST",
    headers: {
      Authorization: `Bearer ${key}`,
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      model: MODEL,
      store: false,
      stream: false,
      reasoning: { effort },
      max_output_tokens: effort === "low" ? 450 : 650,
      instructions,
      input: [{ role: "user", content: user }],
    }),
  });
  if (!response.ok) throw new Error("Grok request failed");
  const payload = (await response.json()) as GrokPayload;
  let text = "";
  for (const item of payload.output ?? []) {
    for (const part of item.content ?? []) {
      if (part.text) text += part.text;
    }
  }
  return text.trim();
}

function deltaFromEvent(payload: string): string | null {
  if (!payload || payload === "[DONE]") return null;
  try {
    const evt = JSON.parse(payload) as { type?: string; delta?: string; text?: string };
    if (typeof evt.delta === "string") return evt.delta;
    if (typeof evt.text === "string") return evt.text;
  } catch {
    return null;
  }
  return null;
}

/** Token deltas from the Responses API (SSE). */
export async function* grokStreamDeltas(
  key: string,
  instructions: string,
  user: string,
  effort: "low" | "medium",
): AsyncGenerator<string> {
  const response = await fetch("https://api.x.ai/v1/responses", {
    method: "POST",
    headers: {
      Authorization: `Bearer ${key}`,
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      model: MODEL,
      store: false,
      stream: true,
      reasoning: { effort },
      max_output_tokens: effort === "low" ? 450 : 650,
      instructions,
      input: [{ role: "user", content: user }],
    }),
  });
  if (!response.ok) throw new Error("Grok request failed");
  const reader = response.body?.getReader();
  if (!reader) throw new Error("Grok request failed");

  const decoder = new TextDecoder();
  let buffer = "";
  while (true) {
    const { done, value } = await reader.read();
    if (done) break;
    buffer += decoder.decode(value, { stream: true });
    let split = buffer.indexOf("\n\n");
    while (split >= 0) {
      const block = buffer.slice(0, split);
      buffer = buffer.slice(split + 2);
      for (const line of block.split("\n")) {
        const trimmed = line.trim();
        if (!trimmed.startsWith("data:")) continue;
        const piece = deltaFromEvent(trimmed.slice(5).trim());
        if (piece) yield piece;
      }
      split = buffer.indexOf("\n\n");
    }
  }
}

export function readJson(text: string): unknown {
  const start = text.indexOf("{");
  const end = text.lastIndexOf("}");
  if (start < 0 || end < start) throw new Error("Grok did not return JSON");
  return JSON.parse(text.slice(start, end + 1));
}

export function missingKey(): Response {
  return Response.json({ error: "GROK_API_KEY is not set" }, { status: 503 });
}
