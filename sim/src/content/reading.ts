import type { SceneId } from "../scenes";
import type { ReadingMap } from "./types";

// Public left-side copy. Tutor-only focus strings travel with each thread, but
// the reader sees a single scene thread instead of objectives or lesson steps.
export const READING: ReadingMap = {
  state: {
    threads: [
      {
        id: "force",
        title: "The spring pulls home",
        focus: ["The force has the opposite sign from displacement", "Acceleration points back toward the rest point"],
        segments: [
          {
            kind: "look",
            text: "Start above the rest point and the spring is already arguing with the motion. The mass is high, but the acceleration points back down.",
          },
          {
            kind: "look",
            text: "That is the first habit this page is trying to build: do not read position as destiny. A rocket, a spring, or any moving body also carries a direction of change and a force story.",
          },
          {
            kind: "relation",
            tex: String.raw`m\ddot{x} = -kx`,
            stake: "This is the claim the scene is making: displacement creates a restoring force. With the damper open, speed is not in the force law.",
            gloss: String.raw`$m$ is the mass. $k$ is the spring stiffness. $x$ is height from the rest point.`,
          },
          {
            kind: "contrast",
            tex: String.raw`\ddot{x} = -x`,
            stake: "Here the constants are both one, so the sign is the thing to watch. Positive height gives negative acceleration, and negative height gives positive acceleration.",
          },
          {
            kind: "trace",
            text: "On the live chart, height and rate trade places: one is largest near the moments when the other crosses zero.",
          },
          {
            kind: "trace",
            text: "That trade is the beginning of state thinking. You are not memorizing a spring equation; you are watching how one number is not enough to say what happens next.",
          },
        ],
      },
      {
        id: "state",
        title: "State is the pair",
        focus: ["Height and rate together describe the state", "The phase loop is a state picture"],
        segments: [
          {
            kind: "look",
            text: "The readout is not two separate facts. Height says where the mass is, and rate says what it is about to do.",
          },
          {
            kind: "look",
            text: "This is why a state is more than a label. Two moments can have the same height and different rates, which means the next moment will not be the same.",
          },
          {
            kind: "relation",
            tex: String.raw`x = \cos t \qquad v = -\sin t`,
            stake: "The clock chooses both numbers at once. If you know only height, you can miss whether the mass is climbing or falling.",
            gloss: String.raw`$x$ is height from rest. $v$ is rate in meters per second. $t$ is the slider time.`,
          },
          {
            kind: "trace",
            text: "A phase plot turns those two numbers into one point. As time moves, that point loops instead of marching left to right.",
          },
          {
            kind: "trace",
            text: "The loop is useful because it strips away the clock for a moment. It shows the shape of the motion as a relationship between state variables.",
          },
        ],
      },
    ],
  },
  step: {
    threads: [
      {
        id: "step",
        title: "The computed mass waits",
        focus: ["Explicit Euler uses the old state", "Larger step size increases visible error"],
        segments: [
          {
            kind: "look",
            text: "The gold mass is the computed value. The wire box keeps following the smooth spring, so any gap is the cost of holding a step too long.",
          },
          {
            kind: "look",
            text: "A simulation is always making a bargain like this. It cannot know the whole curve at once, so it takes a small promise from the current state and lives with the result until the next update.",
          },
          {
            kind: "relation",
            tex: String.raw`v_{n+1} = v_n - \Delta t\,x_n`,
            stake: "The update uses the height at the start of the step. It does not peek at the curve between samples.",
            gloss: String.raw`$n$ is the current step. $\Delta t$ is the step length. $x_n$ and $v_n$ are the state at the start.`,
          },
          {
            kind: "relation",
            tex: String.raw`x_{n+1} = x_n + \Delta t\,v_n`,
            stake: "The height update also trusts the old rate. When $\\Delta t$ grows, that trust lasts longer, and the gold trace drifts away from the truth.",
          },
          {
            kind: "trace",
            text: "The stepped trace is not wrong because it is jagged. It is wrong when the held value no longer matches the motion it is trying to approximate.",
          },
          {
            kind: "trace",
            text: "Shrinking the step does not make the method smarter. It just makes each guess shorter, which gives the force law more chances to correct the path.",
          },
        ],
      },
    ],
  },
  mass: {
    threads: [
      {
        id: "tank",
        title: "The same thrust changes its mind",
        focus: ["Mass changes during the burn", "Thrust over mass sets acceleration"],
        segments: [
          {
            kind: "look",
            text: "The flame can stay steady while the rocket underneath it changes. A lighter rocket reacts more strongly to the same push.",
          },
          {
            kind: "look",
            text: "This is one of the first places where a rocket stops feeling like a block in a physics problem. The vehicle is spending part of itself while it moves.",
          },
          {
            kind: "relation",
            tex: String.raw`m(t) = 1.5 + 1.5\,\max(0,\,1 - t/4)`,
            stake: "The dry vehicle is always there. The fuel disappears over four seconds, and after that the thrust is gone too.",
            gloss: String.raw`Mass is in kilograms. The fuel term reaches zero at $4\,\mathrm{s}$.`,
          },
          {
            kind: "relation",
            tex: String.raw`a = F/m - g`,
            stake: "Thrust is the input, but acceleration is what the vehicle feels. The same $F$ is less impressive when $m$ is large.",
            gloss: String.raw`$F$ is thrust in newtons. $g$ is $9.81\,\mathrm{m/s^2}$.`,
          },
          {
            kind: "trace",
            text: "The acceleration trace tells the hidden story of the burn: mass is falling, gravity is constant, and thrust eventually ends.",
          },
          {
            kind: "trace",
            text: "Height is the slowest part of that story to change. By the time the path looks different, acceleration and rate have already been different for a while.",
          },
        ],
      },
      {
        id: "integrate",
        title: "Motion is accumulated",
        focus: ["Acceleration changes rate", "Rate changes height"],
        segments: [
          {
            kind: "look",
            text: "The rocket is never commanded to a height. It earns every meter through acceleration, rate, and time.",
          },
          {
            kind: "look",
            text: "That chain matters later for landing. A controller can ask for thrust now, but the effect reaches height only after it has passed through acceleration and rate.",
          },
          {
            kind: "relation",
            tex: String.raw`\dot v = a \qquad \dot x = v`,
            stake: "Acceleration bends the rate first. Height follows only after rate has had time to accumulate.",
            gloss: String.raw`$x$ is height from the pad. $v$ is vertical rate.`,
          },
          {
            kind: "trace",
            text: "When the rate crosses zero, the height has reached a turn. The chart lets you see that turn before the picture alone makes it obvious.",
          },
          {
            kind: "trace",
            text: "If the rocket is still moving upward with negative acceleration, it has not turned yet. The state tells you both what is happening and what is about to happen.",
          },
        ],
      },
    ],
  },
  engine: {
    threads: [
      {
        id: "lag",
        title: "The engine has to catch up",
        focus: ["Command and actual thrust are different", "Lag changes the vehicle state"],
        segments: [
          {
            kind: "look",
            text: "The pale command can jump because it is only a request. The rocket moves from the thrust the engine has actually made.",
          },
          {
            kind: "look",
            text: "That gap is easy to ignore in drawings and hard to ignore in flight. Any real actuator turns a decision into force through its own timing and limits.",
          },
          {
            kind: "relation",
            tex: String.raw`\dot{F} = (u - F)/\tau`,
            stake: "The farther actual thrust is from the command, the faster it tries to close the gap. A larger $\\tau$ makes that catch-up slower.",
            gloss: String.raw`$u$ is the command. $F$ is actual thrust. $\tau$ is the lag time.`,
          },
          {
            kind: "contrast",
            tex: String.raw`0 \le F \le 50`,
            stake: "Even the catch-up has limits. The command can ask for more than the engine can supply, but the vehicle only feels the clamped value.",
          },
          {
            kind: "trace",
            text: "The live chart keeps command and actual thrust together because their gap is the actuator story. Height and rate are the consequence.",
          },
          {
            kind: "trace",
            text: "When the command rises faster than actual thrust, the vehicle is already falling behind the plan. The state will show that delay after the engine has created it.",
          },
        ],
      },
      {
        id: "motion",
        title: "The state follows actual thrust",
        focus: ["Actual thrust drives acceleration", "Lag propagates into height and rate"],
        segments: [
          {
            kind: "look",
            text: "A slow engine is not just a slow line on the chart. It changes when acceleration arrives, so the whole trajectory changes.",
          },
          {
            kind: "look",
            text: "This is the same motion chain as the burn, but with an actuator in front of it. The command is upstream of the state, not part of the state itself.",
          },
          {
            kind: "relation",
            tex: String.raw`\dot v = F/m - g \qquad \dot x = v`,
            stake: "The same chain from the burn is still here. Actual thrust changes acceleration, acceleration changes rate, and rate changes height.",
          },
          {
            kind: "trace",
            text: "When actual thrust lags behind the command, the height trace is late for the same reason.",
          },
          {
            kind: "trace",
            text: "That lateness is what control has to respect. A perfect command sent too late is still too late.",
          },
        ],
      },
    ],
  },
  loopOpen: {
    threads: [
      {
        id: "open",
        title: "A plan that never looks back",
        focus: ["Open loop ignores measured state", "A disturbance becomes drift"],
        segments: [
          {
            kind: "look",
            text: "Open loop is not careless math. It is a command that never checks whether the vehicle is still where the plan expected it to be.",
          },
          {
            kind: "look",
            text: "That can work when the world matches the plan. It falls apart when a disturbance, modeling error, or changing vehicle turns the planned thrust into the wrong thrust.",
          },
          {
            kind: "relation",
            tex: String.raw`F = F_0 + \Delta F`,
            stake: "After one second, the extra thrust is simply held. Nothing in this law asks whether height is above or below the reference.",
            gloss: String.raw`$F_0$ is hover thrust. $\Delta F$ is the extra thrust knob.`,
          },
          {
            kind: "relation",
            tex: String.raw`\dot v = F/m - g \qquad \dot x = v`,
            stake: "A small extra acceleration first becomes rate, then the rate becomes height error. That is why the drift keeps growing.",
          },
          {
            kind: "trace",
            text: "The reference can stay flat while the vehicle walks away from it. The gap is the missing feedback made visible.",
          },
          {
            kind: "trace",
            text: "The important thing is not that the extra thrust is large. It is that nothing notices the drift and pushes back.",
          },
        ],
      },
    ],
  },
  loopError: {
    threads: [
      {
        id: "pid",
        title: "Three ways to answer an error",
        focus: ["Error is reference minus measured height", "P, I, and D react to different signals"],
        segments: [
          {
            kind: "look",
            text: "All three traces face the same target step. They differ because each listens to a different part of the error story.",
          },
          {
            kind: "look",
            text: "The point is not to crown one term as the right answer. Each term notices something useful and misses something else.",
          },
          {
            kind: "relation",
            tex: String.raw`e = x_{\mathrm{ref}} - x`,
            stake: "Error is signed. It says not only how far you are from the target, but which way the correction needs to push.",
          },
          {
            kind: "contrast",
            tex: String.raw`F_0 + K_p e \qquad F_0 + K_i\!\int e\,dt \qquad F_0 - K_d v`,
            stake: "Proportional reacts to the gap now. Integral reacts to error that has lingered. Derivative pushes against motion before height alone tells the whole story.",
          },
          {
            kind: "trace",
            text: "Overshoot, delay, and sluggishness are not separate topics here. They are the visible fingerprints of what each term chooses to notice.",
          },
          {
            kind: "trace",
            text: "When a trace races past the target, ask what signal kept pushing. When it never quite catches up, ask what signal was too weak or missing.",
          },
        ],
      },
    ],
  },
  loopEquil: {
    threads: [
      {
        id: "lin",
        title: "Near hover, the curve looks straight",
        focus: ["Linearization is local", "Nonlinear and linear traces agree only near equilibrium"],
        segments: [
          {
            kind: "look",
            text: "Close to hover, the full vehicle and the simpler model can look almost identical. That agreement is local, not magic.",
          },
          {
            kind: "look",
            text: "Linearization is a way of saying: near this operating point, keep the part of the model that changes first. It is a local promise, not a new vehicle.",
          },
          {
            kind: "relation",
            tex: String.raw`\ddot x \approx -\frac{K_p}{m}(x - x_{\mathrm{eq}})`,
            stake: "The local model says the controller behaves like a spring pulling height back toward equilibrium.",
          },
          {
            kind: "contrast",
            tex: String.raw`\ddot x = F(x)/m - g`,
            stake: "The nonlinear trace keeps the full thrust law and its limits. Move too far from hover and the shortcut starts to show.",
          },
          {
            kind: "trace",
            text: "When the two traces lie on top of each other, the approximation is earning trust. When they separate, the full model is speaking again.",
          },
          {
            kind: "trace",
            text: "That is why the offset knob matters. It does not just move the starting point; it tests how far the local story can be stretched.",
          },
        ],
      },
    ],
  },
  loopHover: {
    threads: [
      {
        id: "unstable",
        title: "An equilibrium you cannot leave alone",
        focus: ["Wrong-sign acceleration makes hover unstable", "Feedback can stabilize and land"],
        segments: [
          {
            kind: "look",
            text: "Hover can be a real equilibrium and still be unsafe. If a small offset accelerates away from the hover point, waiting makes it worse.",
          },
          {
            kind: "look",
            text: "This is the difference between being balanced and being stable. A pencil can balance on its tip, but the balance does not forgive a small mistake.",
          },
          {
            kind: "relation",
            tex: String.raw`\ddot x = \mu^2 x`,
            stake: "The sign is the danger. Positive height creates positive acceleration, so the open-loop trace runs from the point it was supposed to hold.",
          },
          {
            kind: "contrast",
            tex: String.raw`\ddot x = \mu^2 x + \frac{K_p}{m}(x_{\mathrm{ref}} - x)`,
            stake: "Feedback adds a term that watches the gap to the reference. The reference can move toward the pad while the loop keeps pulling the vehicle back.",
          },
          {
            kind: "trace",
            text: "The open-loop trace tells you what the plant wants to do. The closed-loop trace tells you what the controller can keep from happening.",
          },
          {
            kind: "trace",
            text: "Landing is not drawn here as a heroic final burn. It is a moving reference plus feedback that keeps correcting a plant that would rather leave.",
          },
        ],
      },
    ],
  },
  loopMode: {
    threads: [
      {
        id: "hidden",
        title: "What the sensor does not see",
        focus: ["Measured height is not the whole vehicle", "Hidden modes can couple back into motion"],
        segments: [
          {
            kind: "look",
            text: "The measured height can look reasonable while another motion grows inside the vehicle. A sensor is a choice about what counts as visible.",
          },
          {
            kind: "look",
            text: "That is why a good-looking height trace is not always proof that the vehicle is healthy. The controller can only react to the state it can see or estimate.",
          },
          {
            kind: "relation",
            tex: String.raw`\ddot q = \lambda^2 q`,
            stake: "The internal coordinate $q$ has its own dynamics. If it is not measured, the controller does not get to react to it directly.",
          },
          {
            kind: "contrast",
            tex: String.raw`\ddot x = F/m - g + \epsilon q`,
            stake: "Coupling lets the hidden motion push on height anyway. The unseen state can return through the plant.",
          },
          {
            kind: "trace",
            text: "The hidden-mode trace is not there for the controller. It is there for you, so the missing state has somewhere to show itself.",
          },
          {
            kind: "trace",
            text: "Coupling is the warning sign. Once hidden motion pushes on measured height, an invisible problem becomes a visible trajectory change.",
          },
        ],
      },
    ],
  },
  loopSample: {
    threads: [
      {
        id: "digital",
        title: "The correction arrives in pieces",
        focus: ["Discrete control holds thrust between samples", "Delay uses old state information"],
        segments: [
          {
            kind: "look",
            text: "The continuous trace is the controller you would like to have. The discrete trace is the one that samples, waits, and holds.",
          },
          {
            kind: "look",
            text: "Digital control is not worse because it is digital. It is different because time now has gates: measure, compute, hold, and measure again.",
          },
          {
            kind: "relation",
            tex: String.raw`F = F_0 + K_p e + K_i\!\int e\,dt`,
            stake: "Continuous feedback updates as the error changes. Nothing has to wait for the next sample.",
          },
          {
            kind: "contrast",
            tex: String.raw`e_k = x_{\mathrm{ref}}(t) - x(t-\tau)`,
            stake: "The sampled controller can act on old height. A correction that would be right now can be wrong by the time it arrives.",
          },
          {
            kind: "trace",
            text: "Held thrust gives the discrete trace its shape. Delay makes that shape feel like the loop is reacting to a vehicle that has already moved on.",
          },
          {
            kind: "trace",
            text: "A small sample time can look almost continuous. A longer one turns the same gains into a different behavior, because the controller spends more time acting on stale information.",
          },
        ],
      },
    ],
  },
};

export function readingFor(id: SceneId) {
  return READING[id];
}
