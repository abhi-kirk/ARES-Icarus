import * as Tooltip from "@radix-ui/react-tooltip";
import { ArrowUpRight } from "lucide-react";
import { useEffect, useRef } from "react";
import katex from "katex";

export function Formula({ tex }: { tex: string }) {
  const host = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!host.current) return;
    katex.render(tex, host.current, { displayMode: true, throwOnError: false });
  }, [tex]);

  return <div className="formula" ref={host} />;
}

type Piece =
  | { kind: "text"; text: string }
  | { kind: "math"; tex: string }
  | { kind: "link"; label: string; url: string };

const piecePattern = /\$([^$]+)\$|\[([^\]]+)\]\((https?:\/\/(?:[^()\s]|\([^()\s]*\))+)\)/g;

function piecesOf(text: string): Piece[] {
  const pieces: Piece[] = [];
  let last = 0;
  for (const match of text.matchAll(piecePattern)) {
    const index = match.index ?? 0;
    if (index > last) pieces.push({ kind: "text", text: text.slice(last, index) });
    if (match[1] !== undefined) pieces.push({ kind: "math", tex: match[1] });
    else pieces.push({ kind: "link", label: match[2], url: match[3] });
    last = index + match[0].length;
  }
  if (last < text.length) pieces.push({ kind: "text", text: text.slice(last) });
  return pieces;
}

function InlineMath({ tex }: { tex: string }) {
  const html = katex.renderToString(tex, { displayMode: false, throwOnError: false });
  return <span dangerouslySetInnerHTML={{ __html: html }} />;
}

function SourceLink({ label, url }: { label: string; url: string }) {
  return (
    <>
      {label}
      <Tooltip.Provider delayDuration={350}>
        <Tooltip.Root>
          <Tooltip.Trigger asChild>
            <a className="source" href={url} target="_blank" rel="noopener noreferrer" aria-label={`Source for: ${label}`}>
              <ArrowUpRight size={11} strokeWidth={2} aria-hidden="true" />
            </a>
          </Tooltip.Trigger>
          <Tooltip.Portal>
            <Tooltip.Content className="tip source-tip" side="top" align="start" sideOffset={8}>
              {url}
            </Tooltip.Content>
          </Tooltip.Portal>
        </Tooltip.Root>
      </Tooltip.Provider>
    </>
  );
}

/**
 * When streaming is true, an unfinished trailing $... block stays as plain text
 * so KaTeX does not flash half-parsed LaTeX while the JSON is still arriving.
 */
export function MathText({ text, streaming = false }: { text: string; streaming?: boolean }) {
  let display = text;
  if (streaming) {
    const dollars = (text.match(/\$/g) ?? []).length;
    if (dollars % 2 === 1) {
      const cut = text.lastIndexOf("$");
      display = text.slice(0, cut);
    }
  }
  const pieces = piecesOf(display);
  return (
    <>
      {pieces.map((piece, index) => {
        if (piece.kind === "math") return <InlineMath tex={piece.tex} key={index} />;
        if (piece.kind === "link") return <SourceLink label={piece.label} url={piece.url} key={index} />;
        return <span key={index}>{piece.text}</span>;
      })}
    </>
  );
}
