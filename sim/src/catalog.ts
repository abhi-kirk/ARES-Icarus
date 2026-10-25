import type { SceneId } from "./scenes";

/** Story order for Rocket Dynamics. URL segment is `slug`. */
export type ChapterId = "state" | "loop" | "turn" | "vehicle" | "descent";

export type CatalogScene = {
  slug: string;
  name: string;
  /** Set when the picture and buffer exist in `scenes.ts`. */
  sceneId?: SceneId;
  ready: boolean;
};

export type CatalogChapter = {
  id: ChapterId;
  slug: string;
  title: string;
  scenes: CatalogScene[];
};

export const CHAPTERS: CatalogChapter[] = [
  {
    id: "state",
    slug: "state",
    title: "The state",
    scenes: [
      { slug: "spring", name: "The spring", sceneId: "state", ready: true },
      { slug: "step", name: "The step", sceneId: "step", ready: true },
      { slug: "burn", name: "The burn", sceneId: "mass", ready: true },
      { slug: "engine", name: "The engine", sceneId: "engine", ready: true },
    ],
  },
  {
    id: "loop",
    slug: "loop",
    title: "The loop",
    scenes: [
      { slug: "open-loop", name: "Open loop", ready: false },
      { slug: "error-response", name: "Three reactions to an error", ready: false },
      { slug: "equilibrium", name: "Equilibrium and linearization", ready: false },
      { slug: "unstable-hover", name: "Hover that is unstable and still lands", ready: false },
      { slug: "hidden-mode", name: "A mode you cannot steer or see", ready: false },
      { slug: "sample-time", name: "Sample time and delay", ready: false },
    ],
  },
  {
    id: "turn",
    slug: "turn",
    title: "The turn",
    scenes: [
      { slug: "frames", name: "Frames", ready: false },
      { slug: "gimbal-lock", name: "Gimbal lock", ready: false },
      { slug: "quaternions", name: "Quaternions", ready: false },
      { slug: "spin", name: "Spin", ready: false },
      { slug: "inertia-torque", name: "Inertia and torque", ready: false },
      { slug: "gimbal-allocation", name: "Gimbal and allocation", ready: false },
      { slug: "air-cp", name: "Air and center of pressure", ready: false },
      { slug: "slosh", name: "Tank that drains and sloshes", ready: false },
    ],
  },
  {
    id: "vehicle",
    slug: "vehicle",
    title: "The vehicle",
    scenes: [
      { slug: "ascent", name: "Ascent", ready: false },
      { slug: "thrust-axis", name: "Thrust along the nose", ready: false },
      { slug: "wind", name: "Wind", ready: false },
      { slug: "estimate", name: "Sensors and an estimate", ready: false },
      { slug: "guidance", name: "Guidance", ready: false },
      { slug: "autopilot", name: "Autopilot as mass and air change", ready: false },
    ],
  },
  {
    id: "descent",
    slug: "descent",
    title: "The best descent",
    scenes: [
      { slug: "cost", name: "A cost that makes a gain", ready: false },
      { slug: "constraints", name: "A path under constraints", ready: false },
      { slug: "lossless", name: "Lossless convexification", ready: false },
      { slug: "scvx", name: "Successive convexification", ready: false },
      { slug: "replanning", name: "Replanning", ready: false },
      { slug: "missed", name: "Runs that do not land", ready: false },
    ],
  },
];

export const DYNAMICS_PREFIX = "/dynamics";

/** First scene a visitor should see when a path is missing or not ready yet. */
export const DEFAULT_SCENE: SceneId = "state";

export function chapterBySlug(slug: string): CatalogChapter | undefined {
  return CHAPTERS.find((chapter) => chapter.slug === slug);
}

export function chapterById(id: ChapterId): CatalogChapter {
  const found = CHAPTERS.find((chapter) => chapter.id === id);
  if (!found) return CHAPTERS[0];
  return found;
}

export function sceneEntry(chapterSlug: string, sceneSlug: string): { chapter: CatalogChapter; scene: CatalogScene } | null {
  const chapter = chapterBySlug(chapterSlug);
  if (!chapter) return null;
  const scene = chapter.scenes.find((item) => item.slug === sceneSlug);
  if (!scene) return null;
  return { chapter, scene };
}

export function pathForEntry(chapterSlug: string, sceneSlug: string): string {
  return `${DYNAMICS_PREFIX}/${chapterSlug}/${sceneSlug}`;
}

export function pathForSceneId(id: SceneId): string {
  for (const chapter of CHAPTERS) {
    for (const scene of chapter.scenes) {
      if (scene.sceneId === id) return pathForEntry(chapter.slug, scene.slug);
    }
  }
  return pathForEntry("state", "spring");
}

export function defaultDynamicsPath(): string {
  return pathForSceneId(DEFAULT_SCENE);
}

export function chapterForSceneId(id: SceneId): CatalogChapter {
  for (const chapter of CHAPTERS) {
    if (chapter.scenes.some((scene) => scene.sceneId === id)) return chapter;
  }
  return CHAPTERS[0];
}

export type ReadyScene = CatalogScene & { sceneId: SceneId };

export function readyScenesInChapter(chapterId: ChapterId): ReadyScene[] {
  return chapterById(chapterId).scenes.filter(
    (scene): scene is ReadyScene => scene.ready && scene.sceneId !== undefined,
  );
}

export function sceneIdFromPath(path: string): SceneId | null {
  const parsed = parseDynamicsPath(path);
  if (!parsed) return null;
  const entry = sceneEntry(parsed.chapterSlug, parsed.sceneSlug);
  if (!entry || !entry.scene.ready || !entry.scene.sceneId) return null;
  return entry.scene.sceneId;
}

export function parseDynamicsPath(path: string): { chapterSlug: string; sceneSlug: string } | null {
  const clean = path.endsWith("/") && path.length > 1 ? path.slice(0, -1) : path;
  if (!clean.startsWith(`${DYNAMICS_PREFIX}/`)) return null;
  const rest = clean.slice(DYNAMICS_PREFIX.length + 1);
  if (!rest) return null;
  const parts = rest.split("/").filter(Boolean);
  if (parts.length === 2) {
    return { chapterSlug: parts[0], sceneSlug: parts[1] };
  }
  return null;
}

/** Old flat URLs: /dynamics/spring → state/spring */
const LEGACY_SCENE_SLUG: Record<string, { chapterSlug: string; sceneSlug: string }> = {
  spring: { chapterSlug: "state", sceneSlug: "spring" },
  step: { chapterSlug: "state", sceneSlug: "step" },
  burn: { chapterSlug: "state", sceneSlug: "burn" },
  engine: { chapterSlug: "state", sceneSlug: "engine" },
};

const LEGACY_MODEL = "/model";

export function redirectForPath(path: string): string | null {
  const clean = path.endsWith("/") && path.length > 1 ? path.slice(0, -1) : path;

  if (clean === LEGACY_MODEL) return defaultDynamicsPath();

  if (clean === DYNAMICS_PREFIX) return defaultDynamicsPath();

  if (!clean.startsWith(`${DYNAMICS_PREFIX}/`)) return null;

  const rest = clean.slice(DYNAMICS_PREFIX.length + 1);
  const parts = rest.split("/").filter(Boolean);

  if (parts.length === 1) {
    const only = parts[0];
    const legacy = LEGACY_SCENE_SLUG[only];
    if (legacy) return pathForEntry(legacy.chapterSlug, legacy.sceneSlug);
    const chapter = chapterBySlug(only);
    if (chapter) {
      const first = chapter.scenes.find((scene) => scene.ready && scene.sceneId);
      if (first) return pathForEntry(chapter.slug, first.slug);
      return defaultDynamicsPath();
    }
    return defaultDynamicsPath();
  }

  if (parts.length === 2) {
    const entry = sceneEntry(parts[0], parts[1]);
    if (!entry) return defaultDynamicsPath();
    if (!entry.scene.ready || !entry.scene.sceneId) return defaultDynamicsPath();
    return null;
  }

  return defaultDynamicsPath();
}

export function isDynamicsPath(path: string): boolean {
  if (redirectForPath(path) !== null) return true;
  return sceneIdFromPath(path) !== null;
}
