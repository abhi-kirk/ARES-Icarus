import { useEffect, useMemo, useRef, useState } from "react";
import { ChartView } from "./ChartView";
import { Viewport } from "./Viewport";
import { buildOption, liveOption } from "./chartOption";
import { localTurn, type ChartAsk, type LocalTurn } from "./localChat";
import { T_END, height, rate } from "./model";
import { setTime, togglePlay, usePlaying, useTime } from "./sceneStore";

type Message = {
  role: "user" | "assistant";
  text: string;
  chart?: ChartAsk;
};

async function ask(text: string, time: number): Promise<LocalTurn> {
  try {
    const response = await fetch("/api/chat", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ text, t: time }),
    });
    if (response.ok) {
      const data = (await response.json()) as { text?: string; t?: number | null; chart?: ChartAsk | null };
      const columns = (data.chart?.y ?? []).filter((name) => name === "height" || name === "rate");
      return {
        text: data.text || "I could not read that reply.",
        time: typeof data.t === "number" ? data.t : undefined,
        chart: columns.length && data.chart ? { title: data.chart.title || "Chart", y: columns } : undefined,
      };
    }
  } catch {
    // The dev server has no worker. The local reply is the loop.
  }
  return localTurn(text, time);
}

const PARTS = [
  { id: "model", name: "The model", here: true },
  { id: "loop", name: "The loop", here: false },
  { id: "turn", name: "The turn", here: false },
  { id: "vehicle", name: "The vehicle", here: false },
  { id: "descent", name: "The best descent", here: false },
];

export function App() {
  const time = useTime();
  const playing = usePlaying();
  const live = useMemo(() => liveOption(time), [time]);
  const [messages, setMessages] = useState<Message[]>([
    {
      role: "assistant",
      text: "The bead's height is the state. The slope of that height is the rate. Drag the bead, or tell me a time.",
    },
  ]);
  const [draft, setDraft] = useState("");
  const [note, setNote] = useState<string | null>(null);
  const thread = useRef<HTMLDivElement>(null);

  useEffect(() => {
    const box = thread.current;
    if (box) box.scrollTop = box.scrollHeight;
  }, [messages]);

  async function send() {
    const text = draft.trim();
    if (!text) return;
    setDraft("");
    const turn = await ask(text, time);
    if (turn.time !== undefined) setTime(turn.time);
    setMessages((prev) => [...prev, { role: "user", text }, { role: "assistant", text: turn.text, chart: turn.chart }]);
  }

  return (
    <div className="frame">
      <main className="reading">
        <header>
          <p className="mark">Icarus</p>
          <p className="meta">The model</p>
        </header>
        <article>
          <h1>A state and its rate</h1>
          <p className="lede">
            The bead moves on a straight rod. Its height is the state. How fast that height is changing is the rate.
            At the top and the bottom, the bead turns around and the rate is zero.
          </p>
        </article>
        <form
          className="chat"
          onSubmit={(event) => {
            event.preventDefault();
            send();
          }}
        >
          <div className="thread" ref={thread}>
            {messages.map((message, index) => (
              <div className={message.role} key={index}>
                <p>{message.text}</p>
                {message.chart ? (
                  <ChartView option={buildOption(message.chart.y, message.chart.title, time)} />
                ) : null}
              </div>
            ))}
          </div>
          <div className="composer">
            <label>
              <span className="prompt-label">Talk about the picture</span>
              <input
                value={draft}
                onChange={(event) => setDraft(event.target.value)}
                placeholder="Go to 1.57, or plot the rate"
              />
            </label>
            <button type="submit">Send</button>
          </div>
          <p className="note">This reply is local. Grok joins when the worker has a key.</p>
        </form>
      </main>

      <section className="stage">
        <ol className="scale">
          {PARTS.map((part, index) => (
            <li key={part.id}>
              <button
                type="button"
                aria-current={part.here ? "page" : undefined}
                onClick={() => setNote(part.here ? null : `${part.name} is not drawn yet.`)}
              >
                <i className="tick" />
                <span className="num">0{index + 1}</span>
                {part.name}
              </button>
            </li>
          ))}
        </ol>
        <div className="viewport">
          <Viewport />
          {note ? <p className="stage-note">{note}</p> : null}
          <div className="slider-row">
            <label htmlFor="time">Time</label>
            <button type="button" className="play" aria-pressed={playing} onClick={togglePlay}>
              {playing ? "Pause" : "Play"}
            </button>
            <input
              id="time"
              type="range"
              min={0}
              max={T_END}
              step={0.01}
              value={time}
              onChange={(event) => setTime(Number(event.target.value))}
            />
            <output>
              <span>{time.toFixed(2)} s</span>
              <span>{height(time).toFixed(2)} m</span>
              <span className="rate">{rate(time).toFixed(2)} m/s</span>
            </output>
          </div>
          <ChartView option={live} />
        </div>
      </section>
    </div>
  );
}
