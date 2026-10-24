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

function escapeHtml(text: string): string {
  return text.replaceAll("&", "&amp;").replaceAll("<", "&lt;").replaceAll(">", "&gt;");
}

export function mathHtml(text: string): string {
  const parts = text.split(/(\$[^$]+\$)/g);
  let html = "";
  for (const part of parts) {
    const isMath = part.startsWith("$") && part.endsWith("$") && part.length > 2;
    if (!isMath) {
      html += escapeHtml(part);
      continue;
    }
    html += katex.renderToString(part.slice(1, -1), { displayMode: false, throwOnError: false });
  }
  return html;
}

export function MathText({ text }: { text: string }) {
  return <span dangerouslySetInnerHTML={{ __html: mathHtml(text) }} />;
}
