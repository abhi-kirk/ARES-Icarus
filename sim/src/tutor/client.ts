// The page talks to the Worker through these calls. Both fail the same way
// when the key is missing, so the card and the chat can show one sentence.

import type { ChatReply, ChatRequest } from "./types";

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

async function send(path: string, body: unknown, signal?: AbortSignal): Promise<Response> {
  return fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
    signal,
  });
}

type ChatStreamEvent =
  | { partial: string }
  | { done: true; text: string; t?: number | null; chart?: ChatReply["chart"] }
  | { error: string };

function parseSseBlock(block: string): ChatStreamEvent | null {
  for (const line of block.split("\n")) {
    const trimmed = line.trim();
    if (!trimmed.startsWith("data:")) continue;
    try {
      return JSON.parse(trimmed.slice(5).trim()) as ChatStreamEvent;
    } catch {
      return null;
    }
  }
  return null;
}

export async function postChatStream(
  body: ChatRequest,
  onPartial: (text: string) => void,
  signal?: AbortSignal,
): Promise<ChatReply> {
  await openSession();
  let response = await send("/api/chat", body, signal);
  if (response.status === 401) {
    session = null;
    await openSession();
    response = await send("/api/chat", body, signal);
  }
  if (response.status === 503) throw new Error(OFFLINE);
  if (!response.ok) throw new Error("Grok did not answer. Try again.");
  const reader = response.body?.getReader();
  if (!reader) throw new Error("Grok did not answer. Try again.");

  const decoder = new TextDecoder();
  let buffer = "";
  let final: ChatReply | null = null;
  let lastPartial = "";

  while (true) {
    if (signal?.aborted) {
      await reader.cancel();
      throw new DOMException("Aborted", "AbortError");
    }
    const { done, value } = await reader.read();
    if (done) break;
    buffer += decoder.decode(value, { stream: true });
    let split = buffer.indexOf("\n\n");
    while (split >= 0) {
      const block = buffer.slice(0, split);
      buffer = buffer.slice(split + 2);
      const evt = parseSseBlock(block);
      if (evt && "error" in evt) throw new Error(evt.error);
      if (evt && "partial" in evt && evt.partial) {
        lastPartial = evt.partial;
        onPartial(evt.partial);
      }
      if (evt && "done" in evt && evt.done) {
        final = { text: evt.text, t: evt.t ?? null, chart: evt.chart ?? null };
      }
      split = buffer.indexOf("\n\n");
    }
  }

  if (final?.text) return final;
  if (lastPartial.trim()) {
    return { text: lastPartial.trim(), t: null, chart: null };
  }
  throw new Error("Grok did not answer. Try again.");
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
