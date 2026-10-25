import type { SceneId } from "../scenes";

export type LookSegment = {
  kind: "look";
  text: string;
};

export type RelationSegment = {
  kind: "relation" | "contrast";
  tex: string;
  stake: string;
  gloss?: string;
};

export type TraceSegment = {
  kind: "trace";
  text: string;
};

export type Segment = LookSegment | RelationSegment | TraceSegment;

export type Thread = {
  id: string;
  title: string;
  focus: string[];
  segments: Segment[];
};

export type Reading = {
  threads: Thread[];
};

export type ReadingMap = Record<SceneId, Reading>;
