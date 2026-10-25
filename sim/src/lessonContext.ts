import { readingFor, type Thread } from "./reading";
import {
  chartColumns,
  chatBrief,
  columnLabel,
  columnName,
  readout,
  sceneById,
  visualContext,
  type Knobs,
  type SceneId,
} from "./scenes";

export type LessonContext = {
  sceneId: SceneId;
  title: string;
  lede: string;
  model: string;
  visual: string;
  time: number;
  readout: string[];
  columns: { id: string; name: string; label: string }[];
  block?: {
    id: string;
    title: string;
    focus: string[];
    segments: string[];
  };
};

function segmentLine(thread: Thread, index: number): string {
  const segment = thread.segments[index];
  if (segment.kind === "look") return `Look: ${segment.text}`;
  if (segment.kind === "trace") return `Trace: ${segment.text}`;
  const label = segment.kind === "contrast" ? "Contrast" : "Relation";
  const gloss = segment.gloss ? ` Gloss: ${segment.gloss}` : "";
  return `${label}: ${segment.tex}. Stake: ${segment.stake}${gloss}`;
}

function threadContext(thread: Thread | undefined): LessonContext["block"] {
  if (!thread) return undefined;
  return {
    id: thread.id,
    title: thread.title,
    focus: thread.focus,
    segments: thread.segments.map((_, index) => segmentLine(thread, index)),
  };
}

export function lessonContext(scene: SceneId, knobs: Knobs, time: number, blockId?: string): LessonContext {
  const spec = sceneById(scene);
  const page = readingFor(scene);
  const thread = blockId ? page.threads.find((item) => item.id === blockId) : undefined;
  return {
    sceneId: scene,
    title: spec.title,
    lede: spec.lede,
    model: chatBrief(scene, knobs),
    visual: visualContext(scene),
    time,
    readout: readout(scene, time, knobs).map((item) => `${item.name} ${item.text}`),
    columns: chartColumns(scene).map((column) => ({
      id: column,
      name: columnName(column),
      label: columnLabel(column),
    })),
    block: threadContext(thread),
  };
}

export function renderLessonContext(context: LessonContext): string {
  const lines = [
    `Scene: ${context.title}`,
    `Scene id: ${context.sceneId}`,
    `Lede: ${context.lede}`,
    `Model: ${context.model}`,
    `Visual: ${context.visual}`,
    `Slider time: ${context.time.toFixed(2)} s`,
    `Readout: ${context.readout.join(", ")}`,
    `Allowed chart columns: ${context.columns.map((item) => `${item.id} (${item.label})`).join(", ")}`,
  ];

  if (context.block) {
    lines.push(`Thread: ${context.block.title}`);
    if (context.block.focus.length) lines.push(`Tutor focus: ${context.block.focus.join(" | ")}`);
    for (const segment of context.block.segments) lines.push(segment);
  }

  return lines.join("\n");
}
