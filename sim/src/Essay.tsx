import * as Tooltip from "@radix-ui/react-tooltip";
import { Formula, MathText } from "./Formula";
import { InfoTip } from "./InfoTip";
import { QaCard } from "./qa/QaCard";
import { readingFor } from "./reading";
import type { SceneId } from "./scenes";

export function Essay({
  scene,
  brief,
  onPrompt,
}: {
  scene: SceneId;
  brief: string;
  onPrompt: (id: string, prompt: string) => void;
}) {
  const page = readingFor(scene);

  return (
    <Tooltip.Provider delayDuration={350}>
    <div className="essay">
      {page.blocks.map((block) => (
        <section className="block" key={`${scene}-${block.id}`}>
          <h2>{block.title}</h2>
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
          <QaCard
            cacheKey={`${scene}-${block.id}`}
            brief={`${brief}\nThis part is: ${block.title}.`}
            onPrompt={(prompt) => onPrompt(block.id, prompt)}
          />
        </section>
      ))}
    </div>
    </Tooltip.Provider>
  );
}
