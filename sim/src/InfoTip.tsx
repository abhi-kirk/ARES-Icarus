import * as Tooltip from "@radix-ui/react-tooltip";
import { Info } from "lucide-react";
import { MathText } from "./Formula";

export function InfoTip({ text }: { text: string }) {
  return (
    <Tooltip.Root>
      <Tooltip.Trigger asChild>
        <button type="button" className="info" aria-label="What these symbols mean">
          <Info size={13} strokeWidth={1.75} />
        </button>
      </Tooltip.Trigger>
      <Tooltip.Portal>
        <Tooltip.Content className="tip" side="top" align="start" sideOffset={8}>
          <div className="tip-lines">
            {text.split("\n").map((line) => line.trim()).filter((line) => line.length > 0).map((line) => (
              <p className="prose" key={line}>
                <MathText text={line} />
              </p>
            ))}
          </div>
        </Tooltip.Content>
      </Tooltip.Portal>
    </Tooltip.Root>
  );
}
