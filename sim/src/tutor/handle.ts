import { chatDisplayFromPartial, parseChatReply } from "./chatParse";
import { grokStreamDeltas, grokText, missingKey, readJson } from "./grok";
import { levelById } from "./levels";
import { renderLessonContext } from "../lessonContext";
import { buildChatInstructions, buildChatUserMessage, GRADE_INSTRUCTIONS, QUESTION_INSTRUCTIONS } from "./prompts";
import type { ChatRequest, GradeResponse, QuestionResponse, TutorRequest } from "./types";

async function question(key: string, level: NonNullable<ReturnType<typeof levelById>>, body: TutorRequest): Promise<Response> {
  if (body.op !== "question") return Response.json({ error: "Unknown request" }, { status: 400 });
  const previous = body.previous ? clip(body.previous, 400) : "(none yet)";
  const user = `Lesson context:\n${clip(renderLessonContext(body.context), 2200)}\n\nLevel: ${level.label}\nAim: ${level.aim}\nDo not repeat: ${previous}`;
  const raw = await grokText(key, QUESTION_INSTRUCTIONS, user, "low");
  const parsed = readJson(raw) as Partial<QuestionResponse>;
  if (!parsed.prompt) return Response.json({ error: "Grok did not return a question" }, { status: 502 });
  return Response.json({ prompt: clip(parsed.prompt, 500) });
}

async function grade(key: string, level: NonNullable<ReturnType<typeof levelById>>, body: TutorRequest): Promise<Response> {
  if (body.op !== "grade") return Response.json({ error: "Unknown request" }, { status: 400 });
  if (!body.question || !body.answer?.trim()) return Response.json({ error: "Missing question or answer" }, { status: 400 });
  const user = `Lesson context:\n${clip(renderLessonContext(body.context), 2200)}\n\nLevel: ${level.label}\nQuestion: ${clip(body.question, 500)}\nThe reader answered: ${clip(body.answer, 1000)}`;
  const raw = await grokText(key, GRADE_INSTRUCTIONS, user, "low");
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
  if (!level || !body.context) return Response.json({ error: "Missing level or scene" }, { status: 400 });

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
  const card = body.question?.trim() ? clip(body.question, 500) : "None yet.";
  const instructions = buildChatInstructions();
  const user = buildChatUserMessage({
    context: clip(renderLessonContext(body.context), 2200),
    time: body.t ?? 0,
    cardQuestion: card,
    readerMessage: clip(body.text ?? "", 800),
  });

  const stream = new ReadableStream({
    async start(controller) {
      const enc = new TextEncoder();
      let raw = "";
      try {
        for await (const delta of grokStreamDeltas(key, instructions, user, "low")) {
          raw += delta;
          const partial = chatDisplayFromPartial(raw);
          controller.enqueue(enc.encode(`data: ${JSON.stringify({ partial })}\n\n`));
        }
        const reply = parseChatReply(raw);
        controller.enqueue(enc.encode(`data: ${JSON.stringify({ done: true, ...reply })}\n\n`));
        controller.close();
      } catch {
        controller.enqueue(enc.encode(`data: ${JSON.stringify({ error: "Grok request failed" })}\n\n`));
        controller.close();
      }
    },
  });

  return new Response(stream, {
    headers: {
      "Content-Type": "text/event-stream; charset=utf-8",
      "Cache-Control": "no-cache, no-transform",
      Connection: "keep-alive",
    },
  });
}
