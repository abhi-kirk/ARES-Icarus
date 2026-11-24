import {
  CHAPTERS,
  chapterForSceneId,
  pathForEntry,
  type CatalogChapter,
} from "./catalog";
import type { SceneId } from "./scenes";

export type ChapterCloser = {
  title: string;
  body: string;
  href: string;
  linkLabel: string;
};

const RECAP: Record<CatalogChapter["id"], string> = {
  state:
    "Height, rate, mass, and thrust are the numbers the rest of the course steers, and the engine scene is where a command and the thrust the vehicle feels come apart.",
  loop:
    "Open loop holds a thrust chosen ahead of time, so a leftover push becomes a growing height error, while feedback answers that error from the gap, the lingering gap, and the rate, and sample time and delay decide how fresh the answer is.",
  frequency:
    "Phase margin is how many degrees of extra lag the hover can take at the gain crossover before it rings, and gain margin is how far the proportional gain sits above the gain that just holds. A delay spends phase equal to frequency times the delay, a sample-and-hold spends about half as much, and a notch tuned to a bend takes that peak out.",
  turn:
    "A direction on the rocket is one arrow whose world numbers change with yaw, a quaternion keeps a pitch past vertical as two ordinary numbers, and allocation turns a side force and a moment into gimbal angles that stop at ±15°.",
  vehicle:
    "Ascent is thrust against weight, and the vehicle that follows estimates its height, aims the nose from that estimate, and schedules the gain as mass falls and air stiffness grows with speed squared.",
  descent:
    "A cost on height picks the gain, a cone and a thrust floor and ceiling shape the path, successive passes and GuSTO iterates stay inside a trust region, and a replan recovers a gust only while those limits still have room, because fuel spent early is gone for the landing.",
  propellant:
    "Final speed is the exhaust speed times the log of the mass ratio, the bell adds a pressure term that is zero where the exit matches the air, dropping the empty lower stage raises that speed, and on a vertical climb gravity takes g times the burn back.",
};

/** Closing card on the last ready scene of a chapter. */
export function chapterCloser(scene: SceneId): ChapterCloser | null {
  const chapter = chapterForSceneId(scene);
  const ready = chapter.scenes.filter((item) => item.ready && item.sceneId);
  const last = ready[ready.length - 1];
  if (!last || last.sceneId !== scene) return null;

  const index = CHAPTERS.findIndex((item) => item.id === chapter.id);
  const next = index >= 0 ? CHAPTERS[index + 1] : undefined;
  if (next) {
    const first = next.scenes.find((item) => item.ready && item.sceneId);
    if (!first) return null;
    return {
      title: `After ${chapter.title}`,
      body: RECAP[chapter.id],
      href: pathForEntry(next.slug, first.slug),
      linkLabel: `Continue to ${next.title}`,
    };
  }

  return {
    title: "Fly the landing",
    body: `${RECAP[chapter.id]} The Simulator is the powered landing console.`,
    href: "/simulator",
    linkLabel: "Open the Simulator",
  };
}
