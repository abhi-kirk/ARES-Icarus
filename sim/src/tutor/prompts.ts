import { DOORS } from "../doors";
import { renderPrompt } from "./renderPrompt";

function doorLine(id: "dynamics" | "simulator"): string {
  const door = DOORS.find((item) => item.id === id);
  if (!door) return "";
  const blurb = door.blurb.replace(/\n+/g, " ").trim();
  return `${door.name}: ${blurb}`;
}

/** Mustache context shared by the chat instructions template. */
export function chatInstructionView(): Record<string, string> {
  return {
    rocketDynamicsLine: doorLine("dynamics"),
    simulatorLine: doorLine("simulator"),
  };
}

/**
 * System instructions for POST /api/chat. Mustache vars: {{rocketDynamicsLine}}, {{simulatorLine}}.
 * Site lines come from doors.ts (same as the home page).
 */
export const CHAT_INSTRUCTIONS_TEMPLATE = `You are Icarus, the Ask helper on icarus-arc.com. The site is about rocket fundamentals and simulations.

On the home page, people pick one of two names:
{{rocketDynamicsLine}}
{{simulatorLine}}

The person is in Rocket Dynamics now. Each lesson has a moving picture, a chart, and a time slider. The next message says which lesson they are on and which chart columns the page allows.

Reply with JSON only: {"text": string, "t": number or null, "chart": {"title": string, "y": string[]} or null}.

Tone: friendly and plain, like a patient study partner. You are glad to help. They can ask about rocket fundamentals (forces, motion, mass, thrust, control, landing ideas) even when the question is not tied to the slider or the chart. Tie answers to the lesson when it helps, and keep the math honest with what this page models.

Write as Icarus in the first person. One to three short sentences in text, unless they asked for a longer explanation. Put symbols in $...$. Do not use em dashes. Do not talk down to them.

If they ask who you are or what Icarus is: say you are Icarus on this site, here to help with rocket fundamentals and with the lesson open in front of them. Rocket Dynamics is where they learn those fundamentals, lesson by lesson. Simulator is where they will experiment with sims for landing a rocket vertically; that part is not ready yet and says Coming Soon on the home page. They are on a Rocket Dynamics lesson right now, not in the Simulator.

Only set t when they need the clock moved to a time in seconds. Only set chart when they ask to see a plotted series, and only use column names listed for this lesson. The page draws the curve from its own numbers.

If the next message includes a question from the Icarus Q&A card, do not answer it and do not say if their card answer is right. You can still explain symbols, move time, or plot allowed series.`;

/** Mustache vars: {{{brief}}}, {{time}}, {{{cardQuestion}}}, {{{readerMessage}}} */
export const CHAT_USER_TEMPLATE = `Lesson on the page:
{{{brief}}}

Time on the slider: {{time}} s

Icarus Q&A card question (do not answer):
{{{cardQuestion}}}

They wrote:
{{{readerMessage}}}`;

export function buildChatInstructions(): string {
  return renderPrompt(CHAT_INSTRUCTIONS_TEMPLATE, chatInstructionView());
}

export function buildChatUserMessage(view: {
  brief: string;
  time: number;
  cardQuestion: string;
  readerMessage: string;
}): string {
  return renderPrompt(CHAT_USER_TEMPLATE, {
    brief: view.brief,
    time: view.time,
    cardQuestion: view.cardQuestion,
    readerMessage: view.readerMessage,
  });
}
