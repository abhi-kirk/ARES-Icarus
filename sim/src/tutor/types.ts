import type { LevelId } from "./levels";

export type QuestionRequest = {
  op: "question";
  level: LevelId;
  brief: string;
  previous?: string;
};

export type GradeRequest = {
  op: "grade";
  level: LevelId;
  brief: string;
  question: string;
  answer: string;
};

export type TutorRequest = QuestionRequest | GradeRequest;

export type QuestionResponse = { prompt: string };

export type GradeResponse = { correct: boolean; note: string };

export type ChatRequest = {
  text: string;
  t: number;
  brief: string;
  question?: string;
};

export type ChatReply = {
  text: string;
  t?: number | null;
  chart?: { title: string; y: string[] } | null;
};
