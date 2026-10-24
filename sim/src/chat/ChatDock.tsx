import { useEffect, useRef, useState, type FormEvent } from "react";
import { ChartView } from "../ChartView";
import { MathText } from "../Formula";
import { chartOption } from "../chartOption";
import { localTurn, type ChartAsk } from "../localChat";
import { readingFor } from "../reading";
import { chatBrief, sceneById, type Column } from "../scenes";
import { setKnob, setTime, useKnobs, useScene, useTime } from "../sceneStore";
import { postJson } from "../tutor/client";

type Message = {
  role: "user" | "assistant";
  text: string;
  chart?: ChartAsk;
};

type ChatReply = {
  text?: string;
  t?: number | null;
  chart?: ChartAsk | null;
};

export function ChatDock({
  open,
  onClose,
  question,
}: {
  open: boolean;
  onClose: () => void;
  question: string;
}) {
  const scene = useScene();
  const time = useTime();
  const knobs = useKnobs();
  const page = readingFor(scene);
  const [draft, setDraft] = useState("");
  const [messages, setMessages] = useState<Message[]>([]);
  const thread = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const box = thread.current;
    if (box) box.scrollTop = box.scrollHeight;
  }, [messages, open]);

  useEffect(() => {
    if (!open) return;
    const onKey = (event: KeyboardEvent) => {
      if (event.key === "Escape") onClose();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [open, onClose]);

  async function send(event: FormEvent) {
    event.preventDefault();
    const text = draft.trim();
    if (!text) return;
    setDraft("");
    const spec = sceneById(scene);
    const allowed = new Set(spec.live);
    let reply = localTurn(text, time, scene, knobs, question);
    try {
      const data = await postJson<ChatReply>("/api/chat", { text, t: time, brief: chatBrief(scene, knobs), question });
      const columns = (data.chart?.y ?? []).filter((name): name is Column => allowed.has(name as Column));
      reply = {
        text: data.text || "I could not read that reply.",
        time: typeof data.t === "number" ? data.t : undefined,
        chart: columns.length && data.chart ? { title: data.chart.title || "Chart", y: columns } : undefined,
      };
    } catch (error) {
      const localIsUseful = reply.text !== spec.opener;
      if (!localIsUseful) {
        const message = error instanceof Error ? error.message : "Grok did not answer. Try again.";
        reply = { text: message };
      }
    }
    if (reply.knob) setKnob(reply.knob.key, reply.knob.value);
    if (reply.time !== undefined) setTime(reply.time);
    setMessages((prev) => [...prev, { role: "user", text }, { role: "assistant", text: reply.text, chart: reply.chart }]);
  }

  return (
    <aside className={open ? "drawer is-open" : "drawer"} aria-label="Ask Icarus" aria-hidden={!open} inert={!open}>
      <div className="drawer-body">
        <header className="drawer-bar">
          <p className="drawer-title">Ask</p>
          <button type="button" className="ask" onClick={onClose}>
            Close
          </button>
        </header>
        <div className="thread" ref={thread}>
          {messages.map((message, index) => (
            <div className={message.role} key={index}>
              <p className="prose">
                <MathText text={message.text} />
              </p>
              {message.chart ? <ChartView option={chartOption(scene, knobs, message.chart.y, message.chart.title, time)} /> : null}
            </div>
          ))}
        </div>
        <form
          className="composer"
          onSubmit={(event) => {
            void send(event);
          }}
        >
          <label>
            <span className="field">
              <input value={draft} onChange={(event) => setDraft(event.target.value)} aria-label="Message" />
              {draft === "" ? (
                <span className="ghost prose" aria-hidden="true">
                  <MathText text={page.placeholder} />
                </span>
              ) : null}
            </span>
          </label>
          <button type="submit">Send</button>
        </form>
      </div>
    </aside>
  );
}
