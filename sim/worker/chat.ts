// Reserved route for Grok. The page you run locally answers without this.
// Once GROK_API_KEY is set, POST /api/chat returns one turn as JSON.

type Env = {
  GROK_API_KEY?: string;
  ASSETS: { fetch: (request: Request) => Promise<Response> };
};

const INSTRUCTIONS = `You are the other person looking at one picture with the reader.
The picture is a bead on a rod. Height h = 4 sin(t) meters. Rate = 4 cos(t) meters per second.
t is seconds, from 0 to about 6.28.
Answer in JSON: {"text": string, "t": number or null, "chart": {"title": string, "y": ["height"|"rate", ...]} or null}.
text is two or three sentences. Set t only when the reader should be moved to a time. Set chart only when they asked to see a series.
Do not invent samples. The page fills the chart from its own buffer.
No em dashes.`;

export default {
  async fetch(request: Request, env: Env): Promise<Response> {
    const url = new URL(request.url);
    if (url.pathname === "/api/chat" && request.method === "POST") {
      if (!env.GROK_API_KEY) {
        return Response.json({ error: "GROK_API_KEY is not set" }, { status: 503 });
      }
      const body = (await request.json()) as { text?: string; t?: number };
      const asked = `Current time is ${body.t ?? 0} seconds. The reader said: ${body.text ?? ""}`;
      const upstream = await fetch("https://api.x.ai/v1/responses", {
        method: "POST",
        headers: {
          Authorization: `Bearer ${env.GROK_API_KEY}`,
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          model: "grok-4.7",
          store: false,
          stream: false,
          instructions: INSTRUCTIONS,
          input: [{ role: "user", content: asked }],
          max_output_tokens: 400,
        }),
      });
      if (!upstream.ok) {
        return Response.json({ error: "Grok request failed" }, { status: 502 });
      }
      const payload = (await upstream.json()) as {
        output?: Array<{ type?: string; content?: Array<{ type?: string; text?: string }> }>;
      };
      let text = "";
      for (const item of payload.output ?? []) {
        for (const part of item.content ?? []) {
          if (part.text) text += part.text;
        }
      }
      try {
        return Response.json(JSON.parse(text));
      } catch {
        return Response.json({ text, t: null, chart: null });
      }
    }
    return env.ASSETS.fetch(request);
  },
};
