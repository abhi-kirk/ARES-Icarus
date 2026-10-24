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
      max_output_tokens: effort === "low" ? 250 : 400,
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

export function readJson(text: string): unknown {
  const start = text.indexOf("{");
  const end = text.lastIndexOf("}");
  if (start < 0 || end < start) throw new Error("Grok did not return JSON");
  return JSON.parse(text.slice(start, end + 1));
}

export function missingKey(): Response {
  return Response.json({ error: "GROK_API_KEY is not set" }, { status: 503 });
}
