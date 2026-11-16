import * as Popover from "@radix-ui/react-popover";
import { Info } from "lucide-react";
import { MathText } from "./Formula";

/** Click to open: hover tooltips were easy to miss and silent on touch. */
export function InfoTip({ text }: { text: string }) {
  return (
    <Popover.Root>
      <Popover.Trigger asChild>
        <button type="button" className="info" aria-label="What these symbols mean" aria-haspopup="dialog">
          <Info size={13} strokeWidth={1.75} />
        </button>
      </Popover.Trigger>
      <Popover.Portal>
        <Popover.Content className="tip" side="top" align="start" sideOffset={8} collisionPadding={12}>
          <div className="tip-lines">
            {text
              .split("\n")
              .map((line) => line.trim())
              .filter((line) => line.length > 0)
              .map((line) => (
                <p className="prose" key={line}>
                  <MathText text={line} />
                </p>
              ))}
          </div>
          <Popover.Arrow className="tip-arrow" width={10} height={6} />
        </Popover.Content>
      </Popover.Portal>
    </Popover.Root>
  );
}
