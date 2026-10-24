import { grokText, missingKey, readJson } from "./grok";
import { levelById } from "./levels";
import type { ChatRequest, GradeResponse, QuestionResponse, TutorRequest } from "./types";

const QUESTION = `You write one question about the picture the reader is looking at.
Reply with JSON only: {"prompt": string}.
The prompt uses $...$ around symbols and short equations. No em dashes. Do not include the answer. Do not say which choice is right.`;

const GRADE = `You grade one short answer about the picture.
Reply with JSON only: {"correct": boolean, "note": string}.
note is one sentence. If the answer is wrong, say what is true in a calm tone, without saying the reader failed. If it is right, confirm why. Use $...$ around symbols. No em dashes.`;

const CHAT = `You are Icarus, the guide on this site. Icarus shows how a rocket comes down. This page is the model: one scene at a time, with a picture, a chart, and a clock.
Reply with JSON only: {"text": string, "t": number or null, "chart": {"title": string, "y": string[]} or null}.
Speak as yourself, in the first person. If the reader asks who you are, or what this site is, say you are Icarus, that the site is Rocket Dynamics and a landing simulator, and that this page is Rocket Dynamics. Do not recite the scene's equations unless they asked about the picture or a symbol.
text is one to three sentences. Use $...$ around symbols. No em dashes.
Set t only when they should be moved to a time. Set chart only when they asked to see a series, and only with allowed columns.
Do not invent samples. The page fills the chart from its own buffer.
Questions on the cards are for the reader to try. Do not answer those, and do not say whether they are right. You may define a symbol, move the time, or plot an allowed series.`;

async function question(key: string, level: NonNullable<ReturnType<typeof levelById>>, body: TutorRequest): Promise<Response> {
  if (body.op !== "question") return Response.json({ error: "Unknown request" }, { status: 400 });
  const previous = body.previous ? clip(body.previous, 400) : "(none yet)";
  const user = `Scene: ${clip(body.brief, 1200)}\nLevel: ${level.label}\nAim: ${level.aim}\nDo not repeat: ${previous}`;
  const raw = await grokText(key, QUESTION, user, "low");
  const parsed = readJson(raw) as Partial<QuestionResponse>;
  if (!parsed.prompt) return Response.json({ error: "Grok did not return a question" }, { status: 502 });
  return Response.json({ prompt: clip(parsed.prompt, 500) });
}

async function grade(key: string, level: NonNullable<ReturnType<typeof levelById>>, body: TutorRequest): Promise<Response> {
  if (body.op !== "grade") return Response.json({ error: "Unknown request" }, { status: 400 });
  if (!body.question || !body.answer?.trim()) return Response.json({ error: "Missing question or answer" }, { status: 400 });
  const user = `Scene: ${clip(body.brief, 1200)}\nLevel: ${level.label}\nQuestion: ${clip(body.question, 500)}\nThe reader answered: ${clip(body.answer, 800)}`;
  const raw = await grokText(key, GRADE, user, "low");
  const parsed = readJson(raw) as Partial<GradeResponse>;
  if (typeof parsed.correct !== "boolean" || !parsed.note) {
    return Response.json({ error: "Grok did not grade that answer" }, { status: 502 });
  }
  return Response.json({ correct: parsed.correct, note: clip(parsed.note, 280) });
}

function clip(text: string, max: number): string {
  const clean = text.trim();
  if (clean.length <= max) return clean;
  return clean.slice(0, max);
}

export async function handleTutor(request: Request, key: string | undefined): Promise<Response> {
  if (!key) return missingKey();
  const body = (await request.json()) as TutorRequest;
  const level = levelById(body.level);
  if (!level || !body.brief) return Response.json({ error: "Missing level or scene" }, { status: 400 });

  try {
    if (body.op === "question") return await question(key, level, body);
    if (body.op === "grade") return await grade(key, level, body);
  } catch {
    return Response.json({ error: "Grok request failed" }, { status: 502 });
  }

  return Response.json({ error: "Unknown request" }, { status: 400 });
}

export async function handleChat(request: Request, key: string | undefined): Promise<Response> {
  if (!key) return missingKey();
  const body = (await request.json()) as ChatRequest;
  const question = body.question?.trim() ? clip(body.question, 500) : "(none on the card yet)";
  const user = `The reader is on Rocket Dynamics, a page of Icarus.\nScene: ${clip(body.brief ?? "", 1200)}\nCurrent time is ${body.t ?? 0} seconds.\nQuestion on the card, which you must not answer: ${question}\nThe reader said: ${clip(body.text ?? "", 800)}`;
  try {
    const raw = await grokText(key, CHAT, user, "low");
    return Response.json(readJson(raw));
  } catch {
    return Response.json({ error: "Grok request failed" }, { status: 502 });
  }
}
