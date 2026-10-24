import type { ReactNode } from "react";

// Paragraphs, **bold**, and a list whose lines start with "- ".

function inline(text: string): ReactNode[] {
  const pieces = text.split(/(\*\*[^*]+\*\*)/g);
  return pieces.map((piece, index) => {
    const bold = piece.startsWith("**") && piece.endsWith("**") && piece.length > 4;
    if (!bold) return piece;
    return <strong key={index}>{piece.slice(2, -2)}</strong>;
  });
}

function blocks(source: string): string[] {
  return source
    .trim()
    .split(/\n\s*\n/)
    .map((block) => block.trim())
    .filter((block) => block.length > 0);
}

export function Markdown({ text }: { text: string }) {
  return (
    <div className="markdown">
      {blocks(text).map((block) => {
        const lines = block.split("\n").map((line) => line.trim());
        const list = lines.every((line) => line.startsWith("- "));
        if (list) {
          return (
            <ul key={block}>
              {lines.map((line) => (
                <li key={line}>{inline(line.slice(2))}</li>
              ))}
            </ul>
          );
        }
        return <p key={block}>{inline(lines.join(" "))}</p>;
      })}
    </div>
  );
}
