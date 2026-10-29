import { Hono } from "hono";
import { handleChat, handleTutor } from "../src/tutor/handle";
import { mintSession, sessionCookie, sessionIsValid, sessionToken } from "../src/tutor/session";

type Env = {
  GROK_API_KEY?: string;
  ICARUS_SESSION_SECRET?: string;
  FLIGHT_TOKEN?: string;
  ASSETS: { fetch: (request: Request) => Promise<Response> };
};

const FLIGHT_URL = "https://icarus-arc-flight.fly.dev/flight";
const MAX_CASE = 64_000;

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

// The flight program runs on its own machine. This host checks the session,
// then forwards the case and streams the samples back. The token stays here.
app.post("/api/flight", async (c) => {
  const denied = await refuseWithoutSession(c.req.raw, c.env.ICARUS_SESSION_SECRET);
  if (denied) return denied;

  const token = c.env.FLIGHT_TOKEN;
  if (!token) {
    return Response.json({ error: "The flight program is not on this host." }, { status: 503 });
  }

  const declared = Number(c.req.header("content-length") ?? "0");
  if (Number.isFinite(declared) && declared > MAX_CASE) {
    return Response.json({ error: "The case could not be read." }, { status: 400 });
  }

  const text = await c.req.text();
  if (text.length > MAX_CASE) {
    return Response.json({ error: "The case could not be read." }, { status: 400 });
  }

  const upstream = await fetch(FLIGHT_URL, {
    method: "POST",
    headers: {
      "content-type": "application/json",
      authorization: `Bearer ${token}`,
    },
    body: text,
    signal: c.req.raw.signal,
  });

  const type = upstream.headers.get("content-type") ?? "application/json";
  return new Response(upstream.body, {
    status: upstream.status,
    headers: {
      "content-type": type,
      "cache-control": "no-store",
    },
  });
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
