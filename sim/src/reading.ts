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
  objectives?: string[];
  visualCue?: string;
  chartHint?: string;
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
        objectives: ["Name the state variables", "Explain why the acceleration points back toward rest"],
        visualCue: "The mass is above or below the rest point, and the spring force points back toward the middle.",
        chartHint: "Plot height and rate together to see how one reaches zero when the other is largest.",
        notes: [
          {
            tex: String.raw`m\ddot{x} = -kx`,
            text: "The spring force is proportional to how far the mass is from rest, and it points the other way. The damper is open, so speed does not appear in this force law.",
            tip: String.raw`$m$ is the mass, $1\,\mathrm{kg}$. $k$ is the spring, $1\,\mathrm{N/m}$. $x$ is the height from the rest point, in meters.`,
          },
          {
            tex: String.raw`\ddot{x} = -x`,
            text: "With $m = 1$ and $k = 1$, acceleration is just the negative of height. The mass starts at rest, $1\\,\\mathrm{m}$ above the rest point, so those starting values choose one motion from all possible motions.",
          },
        ],
      },
      {
        id: "state",
        title: "The state",
        objectives: ["Connect height and rate", "Use the phase curve as a state picture"],
        visualCue: "The readout shows height and rate at the same time, not as separate lessons.",
        chartHint: "Ask for a phase plot to see the loop made by plotting rate against height.",
        notes: [
          {
            tex: String.raw`x = \cos t \qquad v = -\sin t`,
            text: "Height and rate are enough to describe where the mass is and where it is going. The same time gives both numbers.",
            tip: String.raw`$x$ is the state, the height from rest. $v$ is the rate, in meters per second. $t$ is the clock on the slider.`,
          },
          {
            tex: String.raw`(x,\, v)`,
            text: "If you plot height against rate, the motion traces a closed loop. That phase plane is a picture of the state, not just a graph against time.",
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
        objectives: ["Explain one explicit Euler update", "Predict what a larger step does to error"],
        visualCue: "The gold mass holds one computed value while the wire box keeps following the true spring.",
        chartHint: "Compare stepped and true height, then change the step size and watch the gap.",
        notes: [
          {
            tex: String.raw`v_{n+1} = v_n - \Delta t\, x_n`,
            text: "The new rate uses only the height from the start of the step. It does not ask where the true curve went during the step.",
            tip: String.raw`$n$ is this step and $n+1$ is the next one. $\Delta t$ is the length of the step. $x_n$ and $v_n$ are the height and the rate at the start of it.`,
          },
          {
            tex: String.raw`x_{n+1} = x_n + \Delta t\, v_n`,
            text: "The new height uses the old rate too. A smaller $\\Delta t$ gives the method less time to be wrong before it checks the force again.",
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
        objectives: ["Separate mass from thrust", "Explain why the same thrust gives changing acceleration"],
        visualCue: "The flame shows the thrust input while the tank mass is changing in the background.",
        chartHint: "Ask for acceleration and mass together to see why the burn changes shape.",
        notes: [
          {
            tex: String.raw`m(t) = 1.5 + 1.5\,\max(0,\, 1 - t/4)`,
            text: "The dry rocket is $1.5\\,\\mathrm{kg}$, and the fuel starts as another $1.5\\,\\mathrm{kg}$. The tank empties in a straight line, and when it is empty the thrust stops.",
            tip: String.raw`The first $1.5\,\mathrm{kg}$ is the rocket with an empty tank. The other $1.5\,\mathrm{kg}$ is fuel. The fuel is gone at $4\,\mathrm{s}$.`,
          },
          {
            tex: String.raw`a = F/m - g`,
            text: "Acceleration is thrust divided by the mass you have now, with gravity taken off. $F$ on the slider is an input you choose, while height and rate are the state that results.",
            tip: String.raw`$F$ is the thrust on the slider, in newtons. $g$ is $9.81\,\mathrm{m/s^2}$. $a$ is the acceleration you can ask the chart to plot.`,
          },
        ],
      },
      {
        id: "integrate",
        title: "Height and rate",
        objectives: ["Connect acceleration to rate", "Connect rate to height"],
        visualCue: "The rocket position is not commanded directly. It is the result of integrating acceleration over time.",
        chartHint: "Plot height, rate, and acceleration to see the chain from input to state.",
        notes: [
          {
            tex: String.raw`\dot v = a \qquad \dot x = v`,
            text: "Each small step adds $a\\,\\Delta t$ to the rate and then uses the rate to move height. A rocket never teleports to a height; the state arrives there through acceleration.",
            tip: String.raw`$x$ is height in meters, from the pad at zero. $v$ is the rate in meters per second. They start at zero on the pad.`,
          },
        ],
      },
    ],
  },
  loopOpen: {
    blocks: [
      {
        id: "open",
        title: "No feedback",
        objectives: ["Define open loop", "Predict why a fixed disturbance accumulates"],
        visualCue: "The command does not look at height, so the rocket cannot correct after the extra thrust arrives.",
        chartHint: "Plot thrust with height to see the disturbance and the resulting drift.",
        notes: [
          {
            tex: String.raw`F = F_0 + \Delta F`,
            text: "Thrust is just a schedule you picked ahead of time. After one second an extra $\\Delta F$ is added and held, but the current height still does not change the command.",
            tip: String.raw`$F_0$ is hover thrust, about $m g$. $\Delta F$ is the Extra knob. The chart compares reference height to actual height.`,
          },
          {
            tex: String.raw`\dot v = F/m - g \qquad \dot x = v`,
            text: "The state is still height and rate. Open loop only means the command ignores that state, so a small wrong acceleration becomes a growing velocity and then a growing position error.",
          },
        ],
      },
    ],
  },
  loopError: {
    blocks: [
      {
        id: "pid",
        title: "Three reactions",
        objectives: ["Name the error", "Compare P, I, and D as three different reactions"],
        visualCue: "All three traces see the same target step, but each one reacts to a different signal.",
        chartHint: "Plot reference with the three responses, then change gain and compare overshoot and delay.",
        notes: [
          {
            tex: String.raw`e = x_{\mathrm{ref}} - x`,
            text: "Error is the signed gap between the target height and the measured height. At two seconds the target steps up by two meters, so each controller suddenly has something to fix.",
          },
          {
            tex: String.raw`F = F_0 + K_p e \;\|\; F_0 + K_i\!\int\! e\,dt \;\|\; F_0 - K_d v`,
            text: "The proportional run pushes on the error now. The integral run pushes on error that has built up. The derivative run pushes against rate, which can slow motion before the height is perfect.",
          },
        ],
      },
    ],
  },
  loopEquil: {
    blocks: [
      {
        id: "lin",
        title: "Linearization",
        objectives: ["Explain what small-signal means", "Predict when the approximation breaks"],
        visualCue: "When the offset is small, the nonlinear and linear traces nearly cover each other.",
        chartHint: "Increase the offset and watch when the two traces stop agreeing.",
        notes: [
          {
            tex: String.raw`\ddot x \approx -\frac{K_p}{m}(x - x_{\mathrm{eq}})`,
            text: "Near hover, the closed-loop law looks like a spring pulling the rocket back toward equilibrium. The linear trace keeps only that local behavior.",
          },
          {
            tex: String.raw`\ddot x = F(x)/m - g`,
            text: "The nonlinear trace uses the full thrust law and its limits. Linearization is useful when the starting offset is small enough that those extra details barely matter.",
          },
        ],
      },
    ],
  },
  loopHover: {
    blocks: [
      {
        id: "unstable",
        title: "Unstable hover",
        objectives: ["Recognize an unstable equilibrium", "Explain how feedback can still land"],
        visualCue: "The open-loop trace leaves hover on its own. The closed-loop trace follows a moving reference toward the pad.",
        chartHint: "Plot reference, open loop, and closed loop to separate the target from the two plant responses.",
        notes: [
          {
            tex: String.raw`\ddot x = \mu^2 x`,
            text: "Open loop, the acceleration has the same sign as the height offset. A small positive height creates more positive acceleration, so hover is an equilibrium you cannot leave alone.",
          },
          {
            tex: String.raw`\ddot x = \mu^2 x + \frac{K_p}{m}(x_{\mathrm{ref}} - x)`,
            text: "Feedback adds a correction based on the gap to the reference. The reference can move toward the pad while the loop keeps fighting the unstable term.",
          },
        ],
      },
    ],
  },
  loopMode: {
    blocks: [
      {
        id: "hidden",
        title: "Hidden mode",
        objectives: ["Separate measured state from internal motion", "Explain why an unmeasured mode can be dangerous"],
        visualCue: "The measured height is calm compared with the hidden mode that the controller does not directly see.",
        chartHint: "Plot measured height and hidden mode together, then increase coupling.",
        notes: [
          {
            tex: String.raw`\ddot q = \lambda^2 q`,
            text: "The internal coordinate $q$ can grow on its own. If your sensor only reports height, this motion is real but not directly visible to the controller.",
          },
          {
            tex: String.raw`\ddot x = F/m - g + \epsilon q`,
            text: "Coupling $\\epsilon$ lets the hidden mode bend the height trajectory. The loop closes on $x$, so the chart shows what can be missed by watching only the measured state.",
          },
        ],
      },
    ],
  },
  loopSample: {
    blocks: [
      {
        id: "digital",
        title: "Sample and delay",
        objectives: ["Compare continuous and sampled feedback", "Predict what delay does to a correction"],
        visualCue: "The discrete trace reacts in held steps and may use an old height measurement.",
        chartHint: "Plot command with continuous and discrete height to see the held control action.",
        notes: [
          {
            tex: String.raw`F = F_0 + K_p e + K_i\!\int\! e\,dt`,
            text: "Continuous feedback updates the thrust at every instant. It is the ideal version of the controller, before sampling and delay make the timing more physical.",
          },
          {
            tex: String.raw`e_k = x_{\mathrm{ref}}(t) - x(t-\tau)`,
            text: "Discrete feedback samples the state, holds thrust between updates, and can use a delayed height reading. When the correction is late, the same gains can feel much more aggressive.",
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
        objectives: ["Separate command from actual thrust", "Predict how lag changes the motion"],
        visualCue: "The command jumps first. Actual thrust approaches it over time and is clamped by the engine limit.",
        chartHint: "Plot command, actual thrust, and height to connect actuator lag to motion.",
        notes: [
          {
            tex: String.raw`\dot{F} = (u - F)/\tau`,
            text: "The engine does not jump to the command. $u$ is what you ask for, $F$ is what the engine is actually making, and $\\tau$ sets how quickly one catches the other.",
            tip: String.raw`$u$ is the command, the Ask slider. $F$ is the thrust the engine is making. $\tau$ is how many seconds that closing takes.`,
          },
          {
            tex: String.raw`0 \le F \le 50`,
            text: "After each little bit of lag, thrust is clamped inside this range. The command can ask for more than the engine can make, so command and actual thrust are different ideas.",
          },
        ],
      },
      {
        id: "motion",
        title: "Height and rate",
        objectives: ["Trace command to thrust to acceleration", "Explain why a slow engine changes the trajectory"],
        visualCue: "The rocket moves from actual thrust, not from the command line.",
        chartHint: "Plot actual thrust with height or rate when you want to see the actuator affect the state.",
        notes: [
          {
            tex: String.raw`\dot v = F/m - g \qquad \dot x = v`,
            text: "The same integration as the burn turns thrust into acceleration, then rate, then height. A slow engine changes the state because the rocket only feels the thrust that has arrived.",
          },
        ],
      },
    ],
  },
};

export function readingFor(id: SceneId): Reading {
  return READING[id];
}
