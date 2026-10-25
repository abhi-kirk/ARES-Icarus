import * as Tooltip from "@radix-ui/react-tooltip";
import { Formula, MathText } from "./Formula";
import { InfoTip } from "./InfoTip";
import { lessonContext } from "./lessonContext";
import { QaCard } from "./qa/QaCard";
import { readingFor, type Segment, type Thread } from "./reading";
import type { Knobs, SceneId } from "./scenes";

export function Essay({
  scene,
  knobs,
  time,
  onPrompt,
}: {
  scene: SceneId;
  knobs: Knobs;
  time: number;
  onPrompt: (id: string, prompt: string) => void;
}) {
  const page = readingFor(scene);

  return (
    <Tooltip.Provider delayDuration={350}>
    <div className="essay">
      {page.threads.map((thread) => (
        <section className="scene-thread" key={`${scene}-${thread.id}`}>
          <h2>{thread.title}</h2>
          {thread.segments.map((segment, index) => (
            <SegmentView segment={segment} key={`${thread.id}-${index}`} />
          ))}
          <QaCard
            cacheKey={`${scene}-${thread.id}-${threadCacheKey(thread)}-${lessonContext(scene, knobs, time, thread.id).model}`}
            context={lessonContext(scene, knobs, time, thread.id)}
            onPrompt={(prompt) => onPrompt(thread.id, prompt)}
          />
        </section>
      ))}
    </div>
    </Tooltip.Provider>
  );
}

function threadCacheKey(thread: Thread): string {
  const text = [thread.title, ...thread.focus, ...thread.segments
    .map((segment) => {
      if (segment.kind === "look" || segment.kind === "trace") return segment.text;
      return `${segment.tex}|${segment.stake}|${segment.gloss ?? ""}`;
    })]
    .join("|");
  let hash = 0;
  for (let i = 0; i < text.length; i += 1) {
    hash = (hash * 31 + text.charCodeAt(i)) >>> 0;
  }
  return hash.toString(36);
}

function SegmentView({ segment }: { segment: Segment }) {
  if (segment.kind === "look") {
    return (
      <p className="scene-thread-look prose">
        <MathText text={segment.text} />
      </p>
    );
  }

  if (segment.kind === "trace") {
    return (
      <p className="scene-thread-trace prose">
        <MathText text={segment.text} />
      </p>
    );
  }

  return (
    <figure className={segment.kind === "contrast" ? "relation is-contrast" : "relation"}>
      <div className="formula-row">
        <Formula tex={segment.tex} />
        {segment.gloss ? <InfoTip text={segment.gloss} /> : null}
      </div>
      <figcaption className="prose">
        <MathText text={segment.stake} />
      </figcaption>
    </figure>
  );
}
