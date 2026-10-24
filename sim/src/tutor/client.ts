// The page talks to the Worker through these two calls. Both fail the same way
// when the key is missing, so the card and the chat can show one sentence.

const OFFLINE = import.meta.env.DEV
  ? "Grok is not configured on this machine. Put GROK_API_KEY and ICARUS_SESSION_SECRET in sim/.dev.vars and restart the dev server."
  : "Grok is not configured.";

// One session for this page load. The cookie is HttpOnly, so the script cannot
// see it. A 401 means it expired, and the next call asks for a fresh one.
let session: Promise<void> | null = null;

function openSession(): Promise<void> {
  if (session) return session;
  session = fetch("/api/session")
    .then((response) => {
      if (response.ok) return;
      session = null;
      if (response.status === 503) throw new Error(OFFLINE);
      throw new Error("Icarus cannot reach the server.");
    })
    .catch((err: unknown) => {
      session = null;
      if (err instanceof Error) throw err;
      throw new Error("Icarus cannot reach the server.");
    });
  return session;
}

async function send(path: string, body: unknown): Promise<Response> {
  return fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
}

export async function postJson<T>(path: string, body: unknown): Promise<T> {
  await openSession();
  let response: Response;
  try {
    response = await send(path, body);
  } catch {
    throw new Error("Icarus cannot reach the server.");
  }
  if (response.status === 401) {
    session = null;
    await openSession();
    try {
      response = await send(path, body);
    } catch {
      throw new Error("Icarus cannot reach the server.");
    }
  }
  if (response.ok) return (await response.json()) as T;
  if (response.status === 503) throw new Error(OFFLINE);
  throw new Error("Grok did not answer. Try again.");
}
