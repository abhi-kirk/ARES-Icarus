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
          <p className="prose">
            <MathText text={text} />
          </p>
        </Tooltip.Content>
      </Tooltip.Portal>
    </Tooltip.Root>
  );
}
