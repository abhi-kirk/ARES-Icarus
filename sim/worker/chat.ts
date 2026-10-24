import { Hono } from "hono";
import { handleChat, handleTutor } from "../src/tutor/handle";
import { mintSession, sessionCookie, sessionIsValid, sessionToken } from "../src/tutor/session";

type Env = {
  GROK_API_KEY?: string;
  ICARUS_SESSION_SECRET?: string;
  ASSETS: { fetch: (request: Request) => Promise<Response> };
};

const app = new Hono<{ Bindings: Env }>();

app.get("/api/session", async (c) => {
  const secret = c.env.ICARUS_SESSION_SECRET;
  if (!secret) return Response.json({ error: "ICARUS_SESSION_SECRET is not set" }, { status: 503 });
  const token = await mintSession(secret, Date.now());
  const forwarded = c.req.header("x-forwarded-proto");
  const secure = forwarded === "https" || new URL(c.req.url).protocol === "https:";
  return new Response(null, {
    status: 204,
    headers: {
      "Cache-Control": "no-store",
      "Set-Cookie": sessionCookie(token, secure),
    },
  });
});

app.post("/api/tutor", async (c) => {
  const denied = await refuseWithoutSession(c.req.raw, c.env.ICARUS_SESSION_SECRET);
  if (denied) return denied;
  return handleTutor(c.req.raw, c.env.GROK_API_KEY);
});

app.post("/api/chat", async (c) => {
  const denied = await refuseWithoutSession(c.req.raw, c.env.ICARUS_SESSION_SECRET);
  if (denied) return denied;
  return handleChat(c.req.raw, c.env.GROK_API_KEY);
});

app.all("*", (c) => c.env.ASSETS.fetch(c.req.raw));

async function refuseWithoutSession(request: Request, secret: string | undefined): Promise<Response | null> {
  if (!secret) return Response.json({ error: "ICARUS_SESSION_SECRET is not set" }, { status: 503 });
  const token = sessionToken(request.headers.get("Cookie") ?? undefined);
  const ok = token !== "" && (await sessionIsValid(secret, token, Date.now()));
  if (!ok) return Response.json({ error: "session" }, { status: 401 });
  return null;
}

export default app;
