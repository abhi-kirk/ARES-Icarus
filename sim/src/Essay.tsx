import * as Tooltip from "@radix-ui/react-tooltip";
import { Formula, MathText } from "./Formula";
import { InfoTip } from "./InfoTip";
import { lessonContext } from "./lessonContext";
import { QaCard } from "./qa/QaCard";
import { readingFor } from "./reading";
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
      {page.blocks.map((block) => (
        <section className="block" key={`${scene}-${block.id}`}>
          <h2>{block.title}</h2>
          {block.visualCue ? (
            <p className="block-bridge prose">
              <MathText text={block.visualCue} />
            </p>
          ) : null}
          {block.notes.map((note) => (
            <figure key={note.tex}>
              <div className="formula-row">
                <Formula tex={note.tex} />
                {note.tip ? <InfoTip text={note.tip} /> : null}
              </div>
              <figcaption className="prose">
                <MathText text={note.text} />
              </figcaption>
            </figure>
          ))}
          {block.chartHint ? (
            <p className="chart-hint prose">
              <MathText text={block.chartHint} />
            </p>
          ) : null}
          <QaCard
            cacheKey={`${scene}-${block.id}-${lessonContext(scene, knobs, time, block.id).model}`}
            context={lessonContext(scene, knobs, time, block.id)}
            onPrompt={(prompt) => onPrompt(block.id, prompt)}
          />
        </section>
      ))}
    </div>
    </Tooltip.Provider>
  );
}
