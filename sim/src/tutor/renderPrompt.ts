import Mustache from "mustache";

// Plain-text prompts; do not HTML-escape values from the page.
Mustache.escape = (text: string) => text;

export function renderPrompt(template: string, view: Record<string, string | number>): string {
  return Mustache.render(template, view).trim();
}
