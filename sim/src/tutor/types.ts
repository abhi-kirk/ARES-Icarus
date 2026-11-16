import type { LevelId } from "./levels";
import type { LessonContext } from "../lessonContext";

export type QuestionRequest = {
  op: "question";
  level: LevelId;
  context: LessonContext;
  previous?: string;
};

export type GradeRequest = {
  op: "grade";
  level: LevelId;
  context: LessonContext;
  question: string;
  answer: string;
};

export type TutorRequest = QuestionRequest | GradeRequest;

export type QuestionResponse = { prompt: string };

/** Fully right, partly right with a named gap, or wrong. */
export type GradeVerdict = "right" | "partial" | "wrong";

export type GradeResponse = { verdict: GradeVerdict; note: string };

export type ChatRequest = {
  text: string;
  t: number;
  context: LessonContext;
  question?: string;
};

export type ChatReply = {
  text: string;
  t?: number | null;
  chart?: { title: string; y: string[]; x?: string | null } | null;
};
