import * as Tabs from "@radix-ui/react-tabs";
import { Check, RefreshCw } from "lucide-react";
import { useEffect, useRef, useState, type FormEvent } from "react";
import { MathText } from "../Formula";
import { cachedPrompt, rememberPrompt } from "./cache";
import { postJson } from "../tutor/client";
import { LEVELS, type LevelId } from "../tutor/levels";
import type { GradeResponse, QuestionResponse } from "../tutor/types";

type Slot = {
  prompt: string;
  draft: string;
  grade: GradeResponse | null;
  error: string;
  busy: "question" | "grade" | "";
};

const EMPTY: Slot = { prompt: "", draft: "", grade: null, error: "", busy: "" };

export function QaCard({
  cacheKey,
  brief,
  onPrompt,
}: {
  cacheKey: string;
  brief: string;
  onPrompt: (prompt: string) => void;
}) {
  const [level, setLevel] = useState<LevelId>("beginner");
  const [slots, setSlots] = useState<Record<LevelId, Slot>>(() => {
    const next = {
      beginner: { ...EMPTY },
      intermediate: { ...EMPTY },
      advanced: { ...EMPTY },
    };
    for (const item of LEVELS) {
      const prompt = cachedPrompt(cacheKey, item.id);
      if (prompt) next[item.id] = { ...EMPTY, prompt };
    }
    return next;
  });
  const briefRef = useRef(brief);
  const tickets = useRef<Record<LevelId, number>>({ beginner: 0, intermediate: 0, advanced: 0 });
  briefRef.current = brief;
  const current = slots[level];

  const tell = useRef(onPrompt);
  tell.current = onPrompt;
  const asked = useRef(false);

  useEffect(() => {
    tell.current(current.prompt);
  }, [current.prompt]);

  useEffect(() => {
    if (asked.current) return;
    asked.current = true;
    for (const item of LEVELS) {
      if (cachedPrompt(cacheKey, item.id)) continue;
      void load(item.id, "");
    }
    // All three questions load together. A tab only shows the one already stored.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  function patch(id: LevelId, next: Partial<Slot>) {
    setSlots((prev) => ({ ...prev, [id]: { ...prev[id], ...next } }));
  }

  async function load(id: LevelId, previous: string) {
    const ticket = tickets.current[id] + 1;
    tickets.current[id] = ticket;
    patch(id, { busy: "question", error: "", grade: null, prompt: previous });
    try {
      const data = await postJson<QuestionResponse>("/api/tutor", {
        op: "question",
        level: id,
        brief: briefRef.current,
        previous,
      });
      if (tickets.current[id] !== ticket) return;
      rememberPrompt(cacheKey, id, data.prompt);
      patch(id, { busy: "", prompt: data.prompt, draft: "", grade: null, error: "" });
    } catch (error) {
      if (tickets.current[id] !== ticket) return;
      const message = error instanceof Error ? error.message : "Grok did not answer. Try again.";
      patch(id, { busy: "", error: message });
    }
  }

  async function check(event: FormEvent) {
    event.preventDefault();
    const answer = current.draft.trim();
    if (!answer || !current.prompt || current.busy) return;
    const id = level;
    patch(id, { busy: "grade", error: "", grade: null });
    try {
      const data = await postJson<GradeResponse>("/api/tutor", {
        op: "grade",
        level: id,
        brief: briefRef.current,
        question: current.prompt,
        answer,
      });
      patch(id, { busy: "", grade: data });
    } catch (error) {
      const message = error instanceof Error ? error.message : "Grok did not answer. Try again.";
      patch(id, { busy: "", error: message });
    }
  }

  return (
    <section className="qa" aria-label="Icarus Q&A">
      <p className="meta">Icarus Q&A</p>
      <Tabs.Root value={level} onValueChange={(value) => setLevel(value as LevelId)}>
        <div className="qa-bar">
          <Tabs.List className="qa-tabs" aria-label="Difficulty">
            {LEVELS.map((item) => (
              <Tabs.Trigger className="qa-trigger" value={item.id} key={item.id}>
                {item.label}
              </Tabs.Trigger>
            ))}
          </Tabs.List>
          <button
            type="button"
            className="qa-refresh"
            aria-label={`New ${LEVELS.find((item) => item.id === level)?.label ?? ""} question`}
            disabled={current.busy === "question"}
            onClick={() => void load(level, current.prompt)}
          >
            <RefreshCw size={14} className={current.busy === "question" ? "spin" : undefined} />
          </button>
        </div>
        <Tabs.Content value={level} className="qa-body">
          {current.busy === "question" && !current.prompt ? <p className="qa-wait">Writing a question.</p> : null}
          {current.prompt ? (
            <form onSubmit={(event) => void check(event)}>
              <p className="prompt prose">
                <MathText text={current.prompt} />
              </p>
              <div className="qa-answer">
                <input
                  value={current.draft}
                  onChange={(event) => patch(level, { draft: event.target.value })}
                  aria-label="Your answer"
                  placeholder="Your answer"
                  disabled={current.busy === "grade"}
                />
                <button type="submit" disabled={current.busy === "grade" || current.draft.trim() === ""}>
                  {current.busy === "grade" ? "Checking" : "Check"}
                </button>
              </div>
            </form>
          ) : null}
          {current.grade ? (
            <div className={current.grade.correct ? "verdict is-right" : "verdict is-wrong"}>
              <p className="verdict-mark">
                {current.grade.correct ? (
                  <span className="check" aria-hidden="true">
                    <Check size={15} strokeWidth={2.75} />
                  </span>
                ) : null}
                {current.grade.correct ? "Right." : "Not quite."}
              </p>
              <p className="prose">
                <MathText text={current.grade.note} />
              </p>
            </div>
          ) : null}
          {current.error ? <p className="qa-error">{current.error}</p> : null}
        </Tabs.Content>
      </Tabs.Root>
    </section>
  );
}
