import { readingFor, type Block } from "./reading";
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
    objectives: string[];
    visualCue?: string;
    chartHint?: string;
    notes: { tex: string; text: string; tip?: string }[];
  };
};

function blockContext(block: Block | undefined): LessonContext["block"] {
  if (!block) return undefined;
  return {
    id: block.id,
    title: block.title,
    objectives: block.objectives ?? [],
    visualCue: block.visualCue,
    chartHint: block.chartHint,
    notes: block.notes,
  };
}

export function lessonContext(scene: SceneId, knobs: Knobs, time: number, blockId?: string): LessonContext {
  const spec = sceneById(scene);
  const page = readingFor(scene);
  const block = blockId ? page.blocks.find((item) => item.id === blockId) : undefined;
  return {
    sceneId: scene,
    title: spec.title,
    lede: spec.lede,
    model: chatBrief(scene, knobs),
    visual: visualContext(scene),
    time,
    readout: readout(scene, time, knobs).map((item) => item.text),
    columns: chartColumns(scene).map((column) => ({
      id: column,
      name: columnName(column),
      label: columnLabel(column),
    })),
    block: blockContext(block),
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
    lines.push(`Block: ${context.block.title}`);
    if (context.block.objectives.length) lines.push(`Objectives: ${context.block.objectives.join(" | ")}`);
    if (context.block.visualCue) lines.push(`Visual cue: ${context.block.visualCue}`);
    if (context.block.chartHint) lines.push(`Chart hint: ${context.block.chartHint}`);
    for (const note of context.block.notes) {
      lines.push(`Equation: ${note.tex}`);
      lines.push(`Caption: ${note.text}`);
      if (note.tip) lines.push(`Symbol tip: ${note.tip}`);
    }
  }

  return lines.join("\n");
}
