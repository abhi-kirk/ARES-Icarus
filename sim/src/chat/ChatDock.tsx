import { Copy, RotateCcw, Send, Square } from "lucide-react";
import { useCallback, useEffect, useRef, useState, type FormEvent, type KeyboardEvent } from "react";
import { chapterForSceneId, readyScenesInChapter } from "../catalog";
import { ChartView } from "../ChartView";
import { MathText } from "../Formula";
import { chartOption } from "../chartOption";
import { lessonContext } from "../lessonContext";
import { localTurn } from "../localChat";
import { pathForScene } from "../paths";
import { usePath, visit } from "../route";
import { chartColumns, durationOf, sceneById, type Column, type SceneId } from "../scenes";
import { getKnobs, getTime, setKnob, setTime, useKnobs, useScene, useTime } from "../sceneStore";
import { postChatStream } from "../tutor/client";
import { RocketMark } from "./RocketMark";
import { clearThread, loadThread, saveThread, type StoredMessage } from "./storage";
import { startersForChapter } from "./starters";

type Message = StoredMessage & {
  id: string;
  pending?: boolean;
  /** Sim time captured when this user turn was sent. */
  sentAt?: number;
};

const ASK_HINT = "Ask a question";

function nextId(): string {
  return `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`;
}

function toStored(messages: Message[]): StoredMessage[] {
  return messages
    .filter((item) => item.role === "user" || (item.text.trim() && !item.pending))
    .map(({ role, text, chart, failed, actions }) => ({ role, text, chart, failed, actions }));
}

/** Keep Ask jump-to times inside the open scene's horizon. */
function clampActionTime(raw: number | null | undefined, horizon: number): number | undefined {
  if (typeof raw !== "number" || !Number.isFinite(raw)) return undefined;
  if (raw < 0) return 0;
  if (raw > horizon) return horizon;
  return raw;
}

export function ChatDock({
  open,
  onClose,
  question,
}: {
  open: boolean;
  onClose: () => void;
  question: string;
}) {
  const path = usePath();
  const scene = useScene();
  const time = useTime();
  const knobs = useKnobs();
  const spec = sceneById(scene);
  const chapter = chapterForSceneId(scene);
  const siblings = readyScenesInChapter(chapter.id).filter((item) => item.sceneId !== scene);

  const [draft, setDraft] = useState("");
  const [messages, setMessages] = useState<Message[]>([]);
  const [sending, setSending] = useState(false);
  const [liveText, setLiveText] = useState("");
  const [chartTall, setChartTall] = useState<Record<string, boolean>>({});

  const thread = useRef<HTMLDivElement>(null);
  const panel = useRef<HTMLElement>(null);
  const abortRef = useRef<AbortController | null>(null);
  const streamedRef = useRef("");

  useEffect(() => {
    setMessages(loadThread(path).map((item) => ({ ...item, id: nextId() })));
  }, [path]);

  useEffect(() => {
    saveThread(path, toStored(messages));
  }, [messages, path]);

  useEffect(() => {
    const box = thread.current;
    if (box) box.scrollTop = box.scrollHeight;
  }, [messages, open, liveText]);

  useEffect(() => {
    if (!open) return;
    const onKey = (event: globalThis.KeyboardEvent) => {
      if (event.key === "Escape") onClose();
    };
    window.addEventListener("keydown", onKey);
    return () => window.removeEventListener("keydown", onKey);
  }, [open, onClose]);

  useEffect(() => {
    if (!open) return;
    const onPointerDown = (event: PointerEvent) => {
      const target = event.target;
      if (!(target instanceof Node)) return;
      if (panel.current?.contains(target)) return;
      if (target instanceof Element && target.closest("[data-ask-toggle]")) return;
      onClose();
    };
    document.addEventListener("pointerdown", onPointerDown);
    return () => document.removeEventListener("pointerdown", onPointerDown);
  }, [open, onClose]);

  function patchAssistant(next: Partial<Message>) {
    setMessages((prev) => {
      const copy = [...prev];
      const index = copy.length - 1;
      if (copy[index]?.role !== "assistant") return prev;
      copy[index] = { ...copy[index], ...next };
      return copy;
    });
  }

  const runTurn = useCallback(
    async (text: string) => {
      // Read the live store at send so the chip and context match the scrubber.
      const sendTime = getTime();
      const sendKnobs = getKnobs();
      const horizon = durationOf(scene);

      setSending(true);
      setLiveText("");
      streamedRef.current = "";
      setMessages((prev) => [
        ...prev,
        { id: nextId(), role: "user", text, sentAt: sendTime },
        { id: nextId(), role: "assistant", text: "", pending: true },
      ]);

      const allowed = new Set(chartColumns(scene));
      const controller = new AbortController();
      abortRef.current = controller;

      try {
        const data = await postChatStream(
          { text, t: sendTime, context: lessonContext(scene, sendKnobs, sendTime), question },
          (partial) => {
            streamedRef.current = partial;
            setLiveText(partial);
            patchAssistant({ text: partial, pending: true });
          },
          controller.signal,
        );
        const columns = (data.chart?.y ?? []).filter((name): name is Column => allowed.has(name as Column));
        const x = data.chart?.x && allowed.has(data.chart.x as Column) ? (data.chart.x as Column) : undefined;
        const modelChart = columns.length && data.chart ? { title: data.chart.title || "Chart", y: columns, x } : undefined;
        const localChart =
          !modelChart && /\b(plot|chart|graph|show)\b/i.test(text)
            ? localTurn(text, sendTime, scene, sendKnobs, question).chart
            : undefined;
        const chart = modelChart ?? localChart;
        const replyText = data.text || streamedRef.current || "I could not read that reply.";
        const moved = clampActionTime(data.t, horizon);
        const actions: Message["actions"] = {};
        if (moved !== undefined) actions.time = moved;
        if (chart) actions.chart = chart;
        if (moved !== undefined) setTime(moved);
        patchAssistant({ text: replyText, chart, pending: false, failed: false, actions });
      } catch (error) {
        if (error instanceof DOMException && error.name === "AbortError") {
          patchAssistant({ text: streamedRef.current.trim() || "Stopped.", pending: false, failed: false });
        } else {
          const offline = localTurn(text, sendTime, scene, sendKnobs, question);
          const streamed = streamedRef.current.trim();
          const message = streamed
            ? streamed
            : offline.text !== spec.opener
              ? offline.text
              : error instanceof Error
                ? error.message
                : "Grok did not answer. Try again.";
          if (!streamed && offline.knob) setKnob(offline.knob.key, offline.knob.value);
          const moved = clampActionTime(offline.time, horizon);
          if (!streamed && moved !== undefined) setTime(moved);
          patchAssistant({
            text: message,
            chart: offline.chart,
            pending: false,
            failed: true,
            actions: moved !== undefined || offline.chart ? { time: moved, chart: offline.chart } : undefined,
          });
        }
      }

      setLiveText("");
      abortRef.current = null;
      setSending(false);
    },
    [question, scene, spec.opener],
  );

  function sendText(text: string) {
    const trimmed = text.trim();
    if (!trimmed || sending) return;
    setDraft("");
    void runTurn(trimmed);
  }

  function send(event: FormEvent) {
    event.preventDefault();
    sendText(draft);
  }

  function onComposerKey(event: KeyboardEvent<HTMLTextAreaElement>) {
    if (event.key === "Enter" && !event.shiftKey) {
      event.preventDefault();
      sendText(draft);
    }
  }

  function stopStream() {
    abortRef.current?.abort();
  }

  function newChat() {
    if (sending) stopStream();
    if (messages.length > 0 && !window.confirm("Start a new chat on this scene?")) return;
    clearThread(path);
    setMessages([]);
    setDraft("");
    setLiveText("");
  }

  async function copyText(text: string) {
    try {
      await navigator.clipboard.writeText(text);
    } catch {
      // ignore
    }
  }

  function applyTime(seconds: number) {
    setTime(seconds);
  }

  function openLesson(sceneId: SceneId) {
    visit(pathForScene(sceneId));
  }

  const starters = startersForChapter(chapter.id);
  const empty = messages.length === 0;

  return (
    <aside
      ref={panel}
      className={open ? "drawer is-open" : "drawer"}
      aria-label="Ask Icarus"
      aria-hidden={!open}
      inert={!open}
    >
      <div className="drawer-body">
        <header className="drawer-bar">
          <div className="drawer-headings">
            <p className="drawer-title">Ask</p>
            <p className="chat-context">
              {chapter.title} · {spec.name}
            </p>
          </div>
          <div className="drawer-actions">
            <button type="button" className="chat-tool" onClick={newChat} disabled={empty && !sending}>
              New chat
            </button>
            <button type="button" className="ask" onClick={onClose}>
              Close
            </button>
          </div>
        </header>

        <div className="thread" ref={thread}>
          {empty ? (
            <div className="chat-empty">
              <RocketMark />
              <p className="chat-empty-lede">Ask about rocket fundamentals or this scene, and I can see the picture and the chart on the page.</p>
              <ul className="chat-starters">
                {starters.map((line) => (
                  <li key={line}>
                    <button type="button" className="chat-starter" onClick={() => sendText(line)}>
                      {line}
                    </button>
                  </li>
                ))}
              </ul>
              {siblings.length > 0 ? (
                <div className="chat-lessons">
                  <p className="meta">Other scenes in this chapter</p>
                  <div className="chat-lesson-links">
                    {siblings.map((item) =>
                      item.sceneId ? (
                        <button
                          key={item.slug}
                          type="button"
                          className="chat-lesson-link"
                          onClick={() => openLesson(item.sceneId!)}
                        >
                          {item.name}
                        </button>
                      ) : null,
                    )}
                  </div>
                </div>
              ) : null}
            </div>
          ) : null}

          {messages.map((message) => (
            <article className={`chat-turn is-${message.role}`} key={message.id}>
              <p className="chat-label">{message.role === "user" ? "You" : "Icarus"}</p>
              <div className="chat-bubble">
                {message.pending && message.text === "" ? (
                  <p className="chat-thinking">
                    Thinking<span className="chat-dots" aria-hidden="true" />
                  </p>
                ) : (
                  <p className="prose">
                    <MathText text={message.text} streaming={Boolean(message.pending)} />
                  </p>
                )}
                {message.role === "assistant" && message.text && !message.pending ? (
                  <div className="chat-bubble-tools">
                    <button type="button" className="chat-tool" aria-label="Copy reply" onClick={() => void copyText(message.text)}>
                      <Copy size={13} strokeWidth={1.75} />
                    </button>
                    {message.failed ? (
                      <button
                        type="button"
                        className="chat-tool"
                        onClick={() => {
                          const index = messages.findIndex((item) => item.id === message.id);
                          for (let j = index - 1; j >= 0; j--) {
                            if (messages[j].role === "user") {
                              void runTurn(messages[j].text);
                              break;
                            }
                          }
                        }}
                      >
                        <RotateCcw size={13} strokeWidth={1.75} />
                        Retry
                      </button>
                    ) : null}
                  </div>
                ) : null}
              </div>
              {message.role === "user" && message.sentAt !== undefined ? (
                <div className="chat-chips">
                  <span className="chat-chip is-meta">Time {message.sentAt.toFixed(2)} s</span>
                </div>
              ) : null}
              {message.actions?.time !== undefined ? (
                <div className="chat-chips">
                  <button type="button" className="chat-chip" onClick={() => applyTime(message.actions!.time!)}>
                    Jump to {message.actions.time.toFixed(2)} s
                  </button>
                </div>
              ) : null}
              {message.chart ? (
                <div className={chartTall[message.id] ? "chat-chart is-tall" : "chat-chart"}>
                  <ChartView option={chartOption(scene, knobs, message.chart.y, message.chart.title, time, message.chart.x)} />
                  <button
                    type="button"
                    className="chat-tool"
                    onClick={() => setChartTall((prev) => ({ ...prev, [message.id]: !prev[message.id] }))}
                  >
                    {chartTall[message.id] ? "Smaller chart" : "Larger chart"}
                  </button>
                </div>
              ) : null}
            </article>
          ))}
        </div>

        <p className="sr-only" aria-live="polite" aria-atomic="true">
          {liveText}
        </p>

        <form
          className="composer"
          onSubmit={(event) => {
            void send(event);
          }}
        >
          <label className="composer-field">
            <span className="field field-textarea">
              <textarea
                rows={2}
                value={draft}
                onChange={(event) => setDraft(event.target.value)}
                onKeyDown={onComposerKey}
                aria-label="Message"
                disabled={sending}
                placeholder={ASK_HINT}
              />
            </span>
          </label>
          {sending ? (
            <button type="button" className="composer-send" aria-label="Stop" onClick={stopStream}>
              <Square size={14} strokeWidth={1.75} fill="currentColor" />
            </button>
          ) : (
            <button type="submit" className="composer-send" aria-label="Send" disabled={draft.trim() === ""}>
              <Send size={15} strokeWidth={1.75} />
            </button>
          )}
        </form>
      </div>
    </aside>
  );
}
