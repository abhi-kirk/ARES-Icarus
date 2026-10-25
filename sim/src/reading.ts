import type { SceneId } from "./scenes";

// Words beside the picture. A scene can hold several blocks. Each block has its own Q&A.
// Questions are not stored here. Grok writes them.

export type Note = {
  tex: string;
  text: string;
  tip?: string;
};

export type Block = {
  id: string;
  title: string;
  notes: Note[];
};

export type Reading = {
  blocks: Block[];
};

export const READING: Record<SceneId, Reading> = {
  state: {
    blocks: [
      {
        id: "force",
        title: "The force",
        notes: [
          {
            tex: String.raw`m\ddot{x} = -kx`,
            text: "The damper is open, so there is no term in the speed.",
            tip: String.raw`$m$ is the mass, $1\,\mathrm{kg}$. $k$ is the spring, $1\,\mathrm{N/m}$. $x$ is the height from the rest point, in meters.`,
          },
          {
            tex: String.raw`\ddot{x} = -x`,
            text: "The mass starts at rest, $1\\,\\mathrm{m}$ above the rest point. Those starting values are the initial conditions.",
          },
        ],
      },
      {
        id: "state",
        title: "The state",
        notes: [
          {
            tex: String.raw`x = \cos t \qquad v = -\sin t`,
            text: "The height and the rate stay on this curve for the whole swing.",
            tip: String.raw`$x$ is the state, the height from rest. $v$ is the rate, in meters per second. $t$ is the clock on the slider.`,
          },
          {
            tex: String.raw`(x,\, v)`,
            text: "Plot height against rate and you trace one closed loop: a phase plane for this spring.",
          },
        ],
      },
    ],
  },
  step: {
    blocks: [
      {
        id: "step",
        title: "One step",
        notes: [
          {
            tex: String.raw`v_{n+1} = v_n - \Delta t\, x_n`,
            text: "The new rate uses the height and the rate from the start of the step, without looking up the true curve.",
            tip: String.raw`$n$ is this step and $n+1$ is the next one. $\Delta t$ is the length of the step. $x_n$ and $v_n$ are the height and the rate at the start of it.`,
          },
          {
            tex: String.raw`x_{n+1} = x_n + \Delta t\, v_n`,
            text: "The new height uses that same old rate, and the gold mass holds it until the next step. The wire box is the true height, $x = \\cos t$.",
          },
        ],
      },
    ],
  },
  mass: {
    blocks: [
      {
        id: "tank",
        title: "The tank",
        notes: [
          {
            tex: String.raw`m(t) = 1.5 + 1.5\,\max(0,\, 1 - t/4)`,
            text: "The tank empties in a straight line, and then the thrust stops.",
            tip: String.raw`The first $1.5\,\mathrm{kg}$ is the rocket with an empty tank. The other $1.5\,\mathrm{kg}$ is fuel. The fuel is gone at $4\,\mathrm{s}$.`,
          },
          {
            tex: String.raw`a = F/m - g`,
            text: "Acceleration is thrust over mass, with gravity taken off. $F$ on the slider is an input you choose, not part of the state.",
            tip: String.raw`$F$ is the thrust on the slider, in newtons. $g$ is $9.81\,\mathrm{m/s^2}$. $a$ is the acceleration you can ask the chart to plot.`,
          },
        ],
      },
      {
        id: "integrate",
        title: "Height and rate",
        notes: [
          {
            tex: String.raw`\dot v = a \qquad \dot x = v`,
            text: "Each little step adds $a\\,\\Delta t$ to the rate and $v\\,\\Delta t$ to the height. The chart shows $x$ and $v$; ask for acceleration or mass if you want those too.",
            tip: String.raw`$x$ is height in meters, from the pad at zero. $v$ is the rate in meters per second. They start at zero on the pad.`,
          },
        ],
      },
    ],
  },
  engine: {
    blocks: [
      {
        id: "lag",
        title: "The lag",
        notes: [
          {
            tex: String.raw`\dot{F} = (u - F)/\tau`,
            text: "The engine does not jump to the command. $u$ is an input; $F$ is what the engine is doing now. Dragging the rocket changes the lag.",
            tip: String.raw`$u$ is the command, the Ask slider. $F$ is the thrust the engine is making. $\tau$ is how many seconds that closing takes.`,
          },
          {
            tex: String.raw`0 \le F \le 50`,
            text: "After each little bit of lag, the thrust is held inside this range, so the pale line can ask for more while the gold line is $F$.",
          },
        ],
      },
      {
        id: "motion",
        title: "Height and rate",
        notes: [
          {
            tex: String.raw`\dot v = F/m - g \qquad \dot x = v`,
            text: "The same integration as the burn: thrust sets acceleration, and height and rate are the state. The readout beside the slider shows them while the live chart follows command and actual thrust.",
          },
        ],
      },
    ],
  },
};

export function readingFor(id: SceneId): Reading {
  return READING[id];
}
