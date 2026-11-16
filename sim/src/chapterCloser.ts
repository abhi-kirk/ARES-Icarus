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
    "Height, rate, mass, and thrust are the numbers the rest of the course will steer. The engine taught that a command and the force the vehicle feels can differ.",
  loop:
    "Open loop drifts. Feedback answers an error from the gap, the lingering gap, and the rate. Sample time and delay decide how fresh that answer is.",
  turn:
    "Attitude lives in frames. Quaternions keep a pitch past vertical. Allocation turns force and moment commands into limited gimbal angles.",
  vehicle:
    "Ascent hands off to a vehicle that estimates, aims, and schedules gains as mass and air change.",
  descent:
    "A cost picks a gain, a cone and thrust bounds shape the path, successive and GuSTO steps iterate under a trust region, and replanning recovers only while the limits still have room. Fuel spent early is gone for the landing.",
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
    body: `${RECAP.descent} The Simulator is the powered landing console: fuel, wind, gust, gain, thrust ring, tilt, glide, lag, gimbal, and inertia sit on the same 6-DOF flight the engine streams.`,
    href: "/simulator",
    linkLabel: "Open the Simulator",
  };
}
