import type { SceneId } from "../scenes";
import type { ReadingMap, Thread } from "./types";

// Public left-side copy. Tutor-only focus strings travel with each thread.
// The first block of a scene is the plain fundamental. The blocks under it carry the equations.

function plain(title: string, focus: string[], paragraphs: string[]): Thread {
  return {
    id: "eli5",
    title,
    focus,
    levels: ["beginner"],
    segments: paragraphs.map((text) => ({ kind: "look", text })),
  };
}

export const READING: ReadingMap = {
  state: {
    threads: [
      plain(
        "A pull toward rest",
        ["The spring pulls back toward the hanging point", "Height and rate together tell the next moment"],
        [
          "A spring pulls a mass back toward the place it hangs at rest, and [the pull is stronger the farther the mass sits from that place](https://en.wikipedia.org/wiki/Hooke%27s_law). Let go from above, and the mass falls through the rest point and keeps going, because it still has speed at the moment the pull passes through zero.",
          "Height says where the mass is. Rate says how fast that height is changing. [Those two numbers are enough to know what the mass does next](https://www.fbswiki.org/wiki/index.php/System_Modeling): the same height with an upward rate climbs, and the same height with a downward rate falls. Drag the mass to set the start height. The Rate knob ($v_0$) sets the start rate, so you can hold one height and leave two ways.",
        ],
      ),
      {
        id: "force",
        title: "The spring pulls home",
        focus: ["The force has the opposite sign from displacement", "Acceleration points back toward the rest point"],
        segments: [
          {
            kind: "look",
            text: "Start above the rest point. The mass is high, and the acceleration points back down.",
          },
          {
            kind: "look",
            text: "Position is one number. A rocket, a spring, or any moving body also carries a direction of change and a force.",
          },
          {
            kind: "relation",
            tex: String.raw`m\ddot{x} = -kx`,
            stake: "The left side is mass times acceleration. The right side is a force proportional to height, and the minus sign points that force back toward the rest point. There is no friction damper here, so the force depends on the height alone.",
            gloss: String.raw`$m$ is the mass.
$k$ is the spring stiffness.
$x$ is height from the rest point.
A damper would add a force against rate; this scene leaves that term out.`,
          },
          {
            kind: "contrast",
            tex: String.raw`\ddot{x} = -x`,
            stake: "In this scene the mass and the stiffness are both one, so acceleration equals the height with the sign flipped. Positive height gives negative acceleration. Negative height gives positive acceleration.",
          },
          {
            kind: "trace",
            text: "Height and rate [trade roles through the cycle](https://en.wikipedia.org/wiki/Simple_harmonic_motion). One is largest near the moments when the other crosses zero.",
          },
          {
            kind: "trace",
            text: "Together those two numbers say what the next moment does.",
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
            text: "Height says where the mass is, and rate says what it is about to do.",
          },
          {
            kind: "look",
            text: "Two moments can share a height and carry different rates, so they leave in different directions.",
          },
          {
            kind: "relation",
            tex: String.raw`x = x_0\cos t + v_0\sin t \qquad v = -x_0\sin t + v_0\cos t`,
            stake: "The start height $x_0$ and start rate $v_0$ fix the whole orbit. The clock then chooses both numbers at once. If you know only height, you can miss whether the mass is climbing or falling.",
            gloss: String.raw`$x$ is height from rest.
$v$ is rate in meters per second.
$x_0$ is height at $t=0$.
$v_0$ is rate at $t=0$.
$t$ is the slider time.`,
          },
          {
            kind: "trace",
            text: "The live chart is that pair as one point: rate against height. The gold dot is the state now, and as time moves the point loops. A larger $|x_0|$ or $|v_0|$ opens a bigger ellipse.",
          },
          {
            kind: "trace",
            text: "Set the same $x_0$ with two different $v_0$ values. The gold dot leaves that height in two directions on the phase loop.",
          },
        ],
      },
    ],
  },
  step: {
    threads: [
      plain(
        "Explicit Euler and a growing orbit",
        ["Both updates use the old state", "That freezes a stale force, so energy grows"],
        [
          "This scene uses [explicit Euler](https://en.wikipedia.org/wiki/Euler_method): both the new rate and the new height are written from the values at the start of the step. The force is frozen for a whole $\\Delta t$, so the gold mass keeps a push that the true spring has already changed.",
          "On an undamped spring that frozen force adds mechanical energy. The pale ellipse is the true orbit. The gold path spirals out as energy grows. Semi-implicit Euler updates the rate first, then the height with that new rate, and stays nearer the ellipse. The Method knob switches those two updates.",
        ],
      ),
      {
        id: "step",
        title: "The computed mass waits",
        focus: ["Explicit Euler uses the old state", "Energy growth spirals the phase orbit"],
        segments: [
          {
            kind: "look",
            text: "The gold mass is the numerical orbit. The wire box follows the true spring. On the chart, pale is the closed ellipse and gold is the stepped orbit.",
          },
          {
            kind: "look",
            text: "Explicit Euler holds both height and rate from the start of the step. Semi-implicit updates rate first, then height.",
          },
          {
            kind: "relation",
            tex: String.raw`v_{n+1} = v_n - \Delta t\,x_n`,
            stake: "Explicit Euler: the new rate still uses the old height. For this spring the acceleration is $-x$, so the update subtracts $\\Delta t$ times the height at the start of the step.",
            gloss: String.raw`$n$ is the current step.
$\Delta t$ is the step length.
$x_n$ is height at the start of the step.
$v_n$ is rate at the start of the step.`,
          },
          {
            kind: "relation",
            tex: String.raw`x_{n+1} = x_n + \Delta t\,v_n`,
            stake: "Explicit Euler also trusts the old rate for the whole step. Semi-implicit would use $v_{n+1}$ here instead. A larger $\\Delta t$ makes either promise last longer.",
          },
          {
            kind: "relation",
            tex: String.raw`E = \tfrac12 v^2 + \tfrac12 x^2`,
            stake: "For this unit spring, energy is kinetic plus potential. Explicit Euler grows $E$ each step, so the phase orbit spirals out. The Energy readout tracks that growth.",
            gloss: String.raw`$E$ is mechanical energy.
$x$ is height.
$v$ is rate.`,
          },
          {
            kind: "trace",
            text: "Lengthen the step on Explicit Euler and the gold spiral opens. Switch Method to semi and the orbit stays closer to the pale ellipse.",
          },
        ],
      },
    ],
  },
  mass: {
    threads: [
      plain(
        "The same push, a lighter rocket",
        ["Burning fuel makes the vehicle lighter", "Height is speed accumulated over time"],
        [
          "[A rocket gets lighter while it burns, because the fuel it pushes with is leaving](https://en.wikipedia.org/wiki/Variable-mass_system). The same thrust on a lighter vehicle speeds it up more. When the tank is empty the thrust stops, and gravity is the force that remains.",
          "[Height is that speed, added up over the seconds the vehicle has been moving](https://en.wikipedia.org/wiki/Equations_of_motion). The path remembers every earlier push, so the climb can still be rising after the acceleration has already turned over.",
        ],
      ),
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
            gloss: String.raw`Mass is in kilograms.
The fuel term reaches zero at $4\,\mathrm{s}$.`,
          },
          {
            kind: "relation",
            tex: String.raw`a = F/m - g`,
            stake: "Thrust is the input. Acceleration is what the vehicle feels. The same $F$ produces less acceleration when $m$ is large.",
            gloss: String.raw`$F$ is thrust in newtons.
$g$ is $9.81\,\mathrm{m/s^2}$.`,
          },
          {
            kind: "trace",
            text: "The live chart shows acceleration and rate. Acceleration tells the hidden story of the burn: mass is falling, gravity is constant, and thrust eventually ends.",
          },
          {
            kind: "trace",
            text: "Rate follows that bend. Height is the slowest part of the story to change, so by the time the path looks different, acceleration and rate have already moved.",
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
            text: "Height arrives through acceleration, rate, and time.",
          },
          {
            kind: "look",
            text: "A controller can ask for thrust now. Height feels that thrust after it has passed through acceleration and rate.",
          },
          {
            kind: "relation",
            tex: String.raw`\dot v = a \qquad \dot x = v`,
            stake: "Acceleration bends the rate first. Height follows only after rate has had time to accumulate.",
            gloss: String.raw`$x$ is height from the pad.
$v$ is vertical rate.`,
          },
          {
            kind: "trace",
            text: "When the rate crosses zero, the height has reached a turn. The chart lets you see that turn before the picture alone makes it obvious.",
          },
          {
            kind: "trace",
            text: "Upward motion with negative acceleration is the moment before the turn. The state holds the motion now and the motion that follows.",
          },
        ],
      },
    ],
  },
  engine: {
    threads: [
      plain(
        "The ask and the thrust",
        ["A command can jump", "Lead asks early so actual thrust is nearer when you need it"],
        [
          "You can ask an engine for a new thrust in an instant. [The engine reaches that thrust on its own time, closing the gap faster when it is farther behind, and slower when its lag is longer](https://en.wikipedia.org/wiki/Time_constant). It also has a limit, so a request past that limit stays at the limit.",
          "The vehicle feels the thrust the engine has already made. If you need that thrust at a marked time, [you ask early](https://en.wikipedia.org/wiki/Feed_forward_(control)) by about one $\\tau$ (the Lead knob). By the need mark, gold actual thrust has already climbed through most of a time constant.",
        ],
      ),
      {
        id: "lag",
        title: "The engine has to catch up",
        focus: ["Command and actual thrust are different", "Lead shifts the ask one τ early"],
        segments: [
          {
            kind: "look",
            text: "The pale command can jump because it is only a request. The rocket moves from the thrust the engine has actually made.",
          },
          {
            kind: "look",
            text: "That gap is easy to ignore in drawings and hard to ignore in flight. [Any real actuator turns a decision into force through its own timing and limits](https://en.wikipedia.org/wiki/Actuator).",
          },
          {
            kind: "relation",
            tex: String.raw`\dot{F} = (u - F)/\tau`,
            stake: "The farther actual thrust is from the command, the faster it tries to close the gap. A larger $\\tau$ makes that catch-up slower.",
            gloss: String.raw`$u$ is the command.
$F$ is actual thrust.
$\tau$ is the lag time.`,
          },
          {
            kind: "contrast",
            tex: String.raw`0 \le F \le 50`,
            stake: "The command can ask for more than the engine can supply. The vehicle feels the clamped value. The Command aside marks the ceiling.",
          },
          {
            kind: "trace",
            text: "With Lead off, the ask and the need share one mark. With Lead on, the ask moves one $\\tau$ earlier and a need mark stays at 0.4 s so you can compare gold thrust at that instant.",
          },
          {
            kind: "trace",
            text: "Flip Lead on and scrub to the need mark. Actual thrust sits higher than the naive step, because the engine has already been catching up.",
          },
        ],
      },
      {
        id: "motion",
        title: "The state follows actual thrust",
        focus: ["Actual thrust drives acceleration", "Leading the lag is what control owes the actuator"],
        segments: [
          {
            kind: "look",
            text: "A slow engine changes when acceleration arrives, so the whole trajectory changes.",
          },
          {
            kind: "look",
            text: "This is the burn's motion chain with an actuator in front of it. The command sits upstream of the state.",
          },
          {
            kind: "relation",
            tex: String.raw`\dot v = F/m - g \qquad \dot x = v`,
            stake: "The same chain from the burn is still here. Actual thrust changes acceleration, acceleration changes rate, and rate changes height. Mass is 2.5 kg.",
            gloss: String.raw`$F$ is actual thrust.
$m$ is 2.5 kg.
$g$ is gravity.`,
          },
          {
            kind: "trace",
            text: "When actual thrust lags behind the command, the height trace is late for the same reason.",
          },
          {
            kind: "trace",
            text: "That lateness is what the next chapter's loop has to respect. A perfect command sent at the need time is still late once the engine is in the way; Lead is the first fix.",
          },
        ],
      },
    ],
  },
  loopOpen: {
    threads: [
      plain(
        "A plan that keeps going",
        ["Open loop flies a thrust chosen ahead of time", "A leftover push becomes a growing height error"],
        [
          "[Open loop is a thrust chosen ahead of time](https://en.wikipedia.org/wiki/Open-loop_controller). The rocket flies that plan from the thrust alone, at whatever height it has reached.",
          "A little extra push, held for the rest of the run, keeps adding speed. Speed kept for longer becomes a taller miss. The extra-thrust knob is how large that leftover push is.",
        ],
      ),
      {
        id: "open",
        title: "A plan with a fixed thrust",
        focus: ["Open loop ignores measured state", "A disturbance becomes drift"],
        segments: [
          {
            kind: "look",
            text: "Open loop is a command that keeps going on the plan, whatever height the vehicle has reached.",
          },
          {
            kind: "look",
            text: "The plan holds while the world matches it. A disturbance, a modeling error, or a changing vehicle makes that same thrust a poor match for the height.",
          },
          {
            kind: "relation",
            tex: String.raw`F = F_0 + \Delta F`,
            stake: "After one second, the extra thrust is held for the rest of the run.",
            gloss: String.raw`$F_0$ is hover thrust.
$\Delta F$ is the extra thrust knob.`,
          },
          {
            kind: "relation",
            tex: String.raw`\dot v = F/m - g \qquad \dot x = v`,
            stake: "A small extra acceleration first becomes rate, then the rate becomes height error. That is why the drift keeps growing.",
            gloss: String.raw`$v$ is vertical rate.
$x$ is height.
$m$ is mass, $g$ is gravity.`,
          },
          {
            kind: "trace",
            text: "The reference stays flat while the height walks away. That gap is the open-loop plan.",
          },
          {
            kind: "trace",
            text: "The drift grows because the law holds the extra thrust and leaves it there.",
          },
        ],
      },
    ],
  },
  loopError: {
    threads: [
      plain(
        "Three ways to answer a miss",
        ["Error is the gap from the target height", "Each term listens to a different part of that gap"],
        [
          "Error is how far the height sits from the target, and which way. Three controllers see that same miss and answer it from three different facts.",
          "[One pushes in proportion to the gap right now. One pushes for the gap that has been sitting there, so a small miss that lasts can still grow into a real push. One pushes against the speed, so it can start easing off while the height is still on the way](https://en.wikipedia.org/wiki/PID_controller). Kp, Ki, and Kd each have a slider. PID on the chart is all three together.",
        ],
      ),
      {
        id: "pid",
        title: "Three ways to answer an error",
        focus: ["Error is reference minus measured height", "P, I, and D react to different signals"],
        segments: [
          {
            kind: "look",
            text: "P, D, and PID share the left axis. I alone is on the right because it can run far from the target. Raise Kp and watch the P and PID traces react.",
          },
          {
            kind: "look",
            text: "Each alone-term listens to a different part of the error. PID uses all three gains at once.",
          },
          {
            kind: "relation",
            tex: String.raw`e = x_{\mathrm{ref}} - x`,
            stake: "Error is signed. It gives the distance to the target and the direction the correction needs to push.",
            gloss: String.raw`$e$ is the error.
$x_{\mathrm{ref}}$ is the target height.
$x$ is the measured height.`,
          },
          {
            kind: "contrast",
            tex: String.raw`F_0 + K_p e \qquad F_0 + K_i\!\int e\,dt \qquad F_0 - K_d v`,
            stake: "Proportional reacts to the gap now. Integral reacts to error that has lingered. Derivative pushes against motion before height alone tells the whole story.",
            gloss: String.raw`$F_0$ is hover thrust, $mg$.
$K_p$, $K_i$, $K_d$ are the three gains.
$v$ is vertical rate.`,
          },
          {
            kind: "trace",
            text: "Overshoot, delay, and sluggishness are the fingerprints of what each term chooses to notice.",
          },
          {
            kind: "trace",
            text: "When a trace races past the target, ask what signal kept pushing. When it stays short of the target, ask what signal was too weak.",
          },
        ],
      },
    ],
  },
  loopEquil: {
    threads: [
      plain(
        "A straight law, close to balance",
        ["Hover balances thrust and weight", "The straight stand-in agrees only near that balance"],
        [
          "[Hover is a balance: thrust equal to weight, height holding still](https://en.wikipedia.org/wiki/Equilibrium_point). Close to that balance, [the way the height springs back looks like a straight law](https://en.wikipedia.org/wiki/Linearization), the same shape as a spring pulling toward one point.",
          "Far from the balance the real thrust law bends, and it can run into a limit. The straight stand-in keeps the slope from the balance point. The offset knob is how far from hover you ask the two to start.",
        ],
      ),
      {
        id: "lin",
        title: "Near hover, the curve looks straight",
        focus: ["Linearization is local", "Nonlinear and linear traces agree only near equilibrium"],
        segments: [
          {
            kind: "look",
            text: "Close to hover, the full vehicle and the simpler model can look almost identical. That agreement is local.",
          },
          {
            kind: "look",
            text: "Near this operating point, linearization keeps the part of the model that changes first. It is a local promise.",
          },
          {
            kind: "relation",
            tex: String.raw`\ddot x \approx -\frac{K_p}{m}(x - x_{\mathrm{eq}})`,
            stake: "The local model says the controller behaves like a spring pulling height back toward equilibrium.",
            gloss: String.raw`$x_{\mathrm{eq}}$ is the hover height.
$K_p$ is the local gain.
$m$ is mass.`,
          },
          {
            kind: "contrast",
            tex: String.raw`\ddot x = F(x)/m - g`,
            stake: "The nonlinear trace keeps the full thrust law and its limits. Move too far from hover and the shortcut starts to show.",
            gloss: String.raw`$F(x)$ is the full thrust law.
$g$ is gravity.`,
          },
          {
            kind: "trace",
            text: "When the two traces lie on top of each other, the approximation is earning trust. When they separate, the full model is speaking again.",
          },
          {
            kind: "trace",
            text: "The offset knob tests how far the local story can be stretched.",
          },
        ],
      },
    ],
  },
  loopHover: {
    threads: [
      plain(
        "Balanced, and still restless",
        ["A hover can accelerate away from itself", "Feedback can hold a restless plant onto a landing"],
        [
          "A hover can be perfectly balanced and still be unsafe. Balance means the forces cancel at that one height. [Stability means a small mistake brings the vehicle back](https://en.wikipedia.org/wiki/Stability_theory). Here a small rise makes the acceleration point farther up, so waiting makes the miss larger.",
          "[Feedback watches the gap to a reference and pushes back](https://en.wikipedia.org/wiki/Feedback). The reference can walk down toward the pad while that push keeps collecting the vehicle. The $\\mu$ knob sets the open-loop growth rate: how hard the restless plant runs away on its own.",
        ],
      ),
      {
        id: "unstable",
        title: "An equilibrium that runs away",
        focus: ["Wrong-sign acceleration makes hover unstable", "Feedback can stabilize and land"],
        segments: [
          {
            kind: "look",
            text: "Hover can be a real equilibrium and still be unsafe. If a small offset accelerates away from the hover point, waiting makes it worse.",
          },
          {
            kind: "look",
            text: "Balance and stability are different. [A pencil can balance on its tip](https://en.wikipedia.org/wiki/Inverted_pendulum), and a small mistake sends it off.",
          },
          {
            kind: "relation",
            tex: String.raw`\ddot x = \mu^2 x`,
            stake: "The sign is the danger. Positive height creates positive acceleration, so the open-loop trace runs from the point it was supposed to hold.",
            gloss: String.raw`$\mu$ is the open-loop growth rate, in 1/s.
$x$ is height from the hover point.`,
          },
          {
            kind: "contrast",
            tex: String.raw`\ddot x = \mu^2 x + \frac{K_p}{m}(x_{\mathrm{ref}} - x) - \frac{K_d}{m}\dot x`,
            stake: "Feedback adds a term that watches the gap to the reference, and a rate term that damps the landing. The loop can hold only while Kp/m is larger than μ².",
            gloss: String.raw`$x_{\mathrm{ref}}$ is the moving landing reference.
$K_p$ and $K_d$ are the feedback gains.
$m$ is mass.`,
          },
          {
            kind: "trace",
            text: "The open-loop trace is the path the plant takes on its own. The closed-loop trace is the path the controller holds.",
          },
          {
            kind: "trace",
            text: "Landing here is a moving reference plus feedback that keeps correcting a plant that wants to leave.",
          },
        ],
      },
    ],
  },
  loopMode: {
    threads: [
      plain(
        "A motion the height sensor skips",
        ["Measured height is one part of the vehicle", "A hidden motion can still shove that height"],
        [
          "A height sensor reports one number, and [the controller reacts to that number, or to an estimate built from it](https://en.wikipedia.org/wiki/Observability). The vehicle can also be bending or ringing in a motion of its own.",
          "That hidden motion has its own growth. Coupling is the path by which it shoves the height anyway, so a calm sensor can sit on a vehicle that is already being pushed. The coupling knob sets how strong that shove is.",
        ],
      ),
      {
        id: "hidden",
        title: "The motion beside the height sensor",
        focus: ["Measured height is one part of the vehicle", "Hidden modes can couple back into motion"],
        segments: [
          {
            kind: "look",
            text: "The measured height can look reasonable while another motion grows inside the vehicle. A sensor is a choice about what counts as visible.",
          },
          {
            kind: "look",
            text: "A calm height trace can sit on a vehicle that is still in trouble. The controller reacts to the state it can see or estimate.",
          },
          {
            kind: "relation",
            tex: String.raw`\ddot q = \lambda^2 q`,
            stake: "The internal coordinate $q$ has its own dynamics. The controller reacts to $q$ once something measures it.",
            gloss: String.raw`$q$ is the hidden mode.
$\lambda$ is its growth rate.`,
          },
          {
            kind: "contrast",
            tex: String.raw`\ddot x = F/m - g + \epsilon q`,
            stake: "Coupling lets the hidden motion push on height anyway. The unseen state can return through the plant.",
            gloss: String.raw`$x$ is measured height.
$\epsilon$ is the coupling knob.
$F$ is thrust.`,
          },
          {
            kind: "trace",
            text: "The hidden-mode trace draws that internal motion beside the measured height.",
          },
          {
            kind: "trace",
            text: "Once the hidden motion pushes on measured height, the height trace bends with it.",
          },
        ],
      },
    ],
  },
  loopSample: {
    threads: [
      plain(
        "Look, decide, hold",
        ["A digital controller holds its thrust between samples", "A delayed sample decides about a vehicle that has moved"],
        [
          "A controller that updates all the time changes the thrust as the error changes. A digital one [looks, decides, and holds that decision until the next look](https://en.wikipedia.org/wiki/Zero-order_hold). Between looks the vehicle keeps moving under a thrust that has stopped listening.",
          "[If the look itself is old, the decision is about a height the vehicle has already left](https://en.wikipedia.org/wiki/Digital_control). A short sample and a short delay stay close to the always-updating loop. A long one spends more of the flight acting on a stale fact. The Delay knob sets how old that reading is. The Sample knob sets how long each hold lasts.",
        ],
      ),
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
            text: "Digital control puts gates in time: measure, compute, hold, and measure again.",
          },
          {
            kind: "relation",
            tex: String.raw`F = F_0 + K_p e + K_i\!\int e\,dt - K_d \dot x`,
            stake: "Continuous feedback updates as the error changes. The rate term keeps the always-updating loop from ringing away.",
            gloss: String.raw`$F_0$ is hover thrust.
$K_p$, $K_i$, $K_d$ are the PID gains.
$e$ is the height error.`,
          },
          {
            kind: "contrast",
            tex: String.raw`e_k = x_{\mathrm{ref}}(t) - x(t-\tau)`,
            stake: "The sampled controller can act on old height. A correction sized for the height now meets a vehicle that has already moved.",
            gloss: String.raw`$e_k$ is the sampled error.
$\tau$ is the measurement delay.
$x_{\mathrm{ref}}$ is the target height.`,
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
  turnFrames: {
    threads: [
      plain(
        "One arrow, two descriptions",
        ["A body direction is fixed on the rocket", "Yaw changes the world numbers that describe it"],
        [
          "A direction painted on the rocket is one arrow, and it turns when the rocket yaws because it is part of the rocket. [The world describes that same arrow with two numbers, how much of it points to the side and how much of it points forward](https://en.wikipedia.org/wiki/Rotation_matrix).",
          "Yaw changes those two numbers and leaves the arrow on the body the same length. At zero yaw the arrow is entirely the world's forward axis. The yaw knob is how far the body has turned.",
        ],
      ),
      {
        id: "frames",
        title: "Yaw changes the world numbers",
        focus: ["A body axis is fixed on the rocket", "Its world components change with yaw"],
        segments: [
          { kind: "look", text: "The gold arrow is painted on the rocket. Yaw turns the rocket, and that arrow turns with it." },
          { kind: "look", text: "The pale arrow is the world's side axis. It stays put while the rocket yaws." },
          {
            kind: "relation",
            tex: String.raw`\mathbf{r}_{\mathrm{world}} = R(\psi)\,\mathbf{r}_{\mathrm{body}}`,
            stake: "The arrow itself is unchanged. The rotation is what makes its world components move.",
          },
          {
            kind: "trace",
            text: "World X and world Z are the two numbers that describe that one body arrow after the yaw. When the yaw is zero they are 1 and 0.",
          },
        ],
      },
    ],
  },
  turnLock: {
    threads: [
      plain(
        "Two hinges become one",
        ["Pitch lifts the nose toward straight up", "At straight up, roll and yaw are the same turn"],
        [
          "[Pitch, yaw, and roll are three hinges](https://en.wikipedia.org/wiki/Euler_angles). Pitch lifts the nose up from the horizon toward straight up. Yaw spins the vehicle about the vertical. Roll spins it about the nose.",
          "While the nose is out on the horizon, those last two hinges point different ways, and a roll is a different turn from a yaw. When the nose reaches straight up, the nose and the vertical are the same line, so a roll and a yaw have become [the same command](https://en.wikipedia.org/wiki/Gimbal_lock). The pitch knob carries the nose along that rise.",
        ],
      ),
      {
        id: "lock",
        title: "Two axes become one",
        focus: ["Pitch carries the nose up from the horizon", "At 90 degrees roll and yaw share an axis"],
        segments: [
          { kind: "look", text: "The pale arrow is vertical. That is yaw. The gold arrow is the nose. That is roll." },
          { kind: "look", text: "Pitch is the angle that lifts the nose off the horizon toward the vertical arrow." },
          {
            kind: "relation",
            tex: String.raw`\gamma = 90^\circ - \theta`,
            stake: "The gap between the nose and the vertical axis shrinks one degree for every degree of pitch.",
          },
          {
            kind: "trace",
            text: "At 90 degrees of pitch the gap is zero. A roll about the nose and a yaw about vertical are the same turn.",
          },
        ],
      },
    ],
  },
  turnQuat: {
    threads: [
      plain(
        "A pitch that stays finite past vertical",
        ["Half the angle sets a cosine and a sine", "Those numbers stay ordinary through straight up"],
        [
          "The nose can take the same rise as in the lock scene and keep going past straight up. Stored as three hinges, that passage is the moment two hinges become one and a small extra pitch is hard to name.",
          "Stored as [the cosine and the sine of half the pitch](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation), the same passage is two ordinary numbers that trade off smoothly. The body still has a direction on the other side of vertical. The pitch knob sets how far past that point the nose goes.",
        ],
      ),
      {
        id: "quaternion",
        title: "The same pitch, past vertical",
        focus: ["A pitch is half-angle cosine and sine", "The numbers stay finite through vertical"],
        segments: [
          { kind: "look", text: "The nose takes the same path as in the lock scene, and it is allowed to keep going past vertical." },
          {
            kind: "relation",
            tex: String.raw`q = \cos(\theta/2) + \sin(\theta/2)\,\mathbf{k}`,
            stake: "One pitch about a fixed axis is two numbers. They are the cosine and sine of half the pitch.",
          },
          {
            kind: "contrast",
            tex: String.raw`\theta = 90^\circ`,
            stake: "Euler yaw and roll become the same command here. The quaternion components keep their roles and stay finite.",
          },
          {
            kind: "trace",
            text: "Watch quat w fall and quat z rise as the nose passes vertical. The body still has a direction on the other side.",
          },
        ],
      },
    ],
  },
  turnSpin: {
    threads: [
      plain(
        "A spin that nobody is changing",
        ["Zero torque leaves the spin rate constant", "Angle is that rate times time"],
        [
          "[Torque is zero, so the rocket keeps the spin it already has](https://en.wikipedia.org/wiki/Angular_momentum). The stripe walks around the body at that one pace, and the pace itself stays put.",
          "[The angle is the pace times how long the rocket has been turning](https://en.wikipedia.org/wiki/Angular_velocity). A faster spin draws a steeper climb in angle and leaves the spin as a flat line at a new height. The spin knob is that pace.",
        ],
      ),
      {
        id: "spin",
        title: "A rate that stays put",
        focus: ["No torque leaves the spin rate constant", "Angle is rate times time"],
        segments: [
          { kind: "look", text: "The stripe walks around the body at one pace. The spin stays at that pace." },
          {
            kind: "relation",
            tex: String.raw`\theta = \omega t`,
            stake: "A constant rate draws a straight ramp in angle. The slope of that ramp is the spin.",
          },
          {
            kind: "trace",
            text: "Drag the rocket and the ramp gets steeper. The spin trace stays flat, at a new height.",
          },
        ],
      },
    ],
  },
  turnInertia: {
    threads: [
      plain(
        "The same twist, a heavier reluctance",
        ["Torque changes the spin", "Inertia is how hard that change is"],
        [
          "[Torque is a twist](https://en.wikipedia.org/wiki/Torque). It changes how fast the rocket is spinning. [Inertia is the rocket's reluctance to that change](https://en.wikipedia.org/wiki/Moment_of_inertia): the same twist produces a smaller change in spin when the inertia is larger.",
          "Here the twist stays at a steady 2 newton-meters, so the spin climbs in a straight line and the slope of that line is the twist divided by the inertia. A larger inertia reaches the same angle later. The inertia knob is that reluctance.",
        ],
      ),
      {
        id: "inertia",
        title: "The same torque, a different inertia",
        focus: ["Torque equals inertia times angular acceleration", "More inertia climbs more slowly"],
        segments: [
          { kind: "look", text: "The torque stays at 2 N·m. The rocket speeds up, and how quickly it speeds up is the inertia." },
          {
            kind: "relation",
            tex: String.raw`\tau = I \alpha`,
            stake: "Angular acceleration is the torque divided by the inertia. The spin rate is that acceleration times time.",
          },
          {
            kind: "trace",
            text: "A larger inertia makes the spin trace shallower and the angle arrive later.",
          },
        ],
      },
    ],
  },
  turnGimbal: {
    threads: [
      plain(
        "Two engines, one allocation",
        ["Commands are side force and moment", "An allocation matrix picks two signed gimbal angles"],
        [
          "A flight computer asks for a sideways shove and a twist. [Two nozzles share that ask](https://en.wikipedia.org/wiki/Control_allocation): each can tilt left or right, and each stops at an actuator limit.",
          "The solve maps the two commands onto δ₁ and δ₂. Common tilt builds side force. Differential tilt builds moment. When a command needs more tilt than ±15°, that gimbal clips and the delivered force or moment falls short.",
        ],
      ),
      {
        id: "gimbal",
        title: "Allocate, then clip",
        focus: ["Fy and M are the commands", "δ₁ and δ₂ come from an allocation matrix", "Each actuator stops at ±15°"],
        segments: [
          { kind: "look", text: "Side force and moment on the chart are what the clipped gimbals deliver. Pitch is δ₁ and angle is δ₂. Drag Side Fy; set Moment M on the second slider." },
          {
            kind: "relation",
            tex: String.raw`\begin{bmatrix} F_y \\ M \end{bmatrix} = \begin{bmatrix} T & T \\ T\ell & -T\ell \end{bmatrix}\begin{bmatrix} \delta_1 \\ \delta_2 \end{bmatrix}`,
            stake: "In the small-angle model, the sum of the gimbals sets side force and the difference sets moment. T is thrust per engine and ℓ is the lever to the center of mass. The scene then uses sin δ for the delivered numbers.",
            gloss: String.raw`$T$ is thrust on each engine.
$\ell$ is the moment arm.
$\delta_1$ and $\delta_2$ are the gimbal angles.`,
          },
          {
            kind: "contrast",
            tex: String.raw`|\delta_i| \le 15^\circ`,
            stake: "Each actuator is signed and hard-limited. A large Side Fy with a large Moment M can demand more than ±15° on one engine; that engine clips and the traces sit below the unsaturated ask.",
            gloss: String.raw`$\delta_i$ is either gimbal angle.`,
          },
          {
            kind: "trace",
            text: "A pure side command drives δ₁ ≈ δ₂. A pure moment command drives δ₁ ≈ −δ₂. Mix both and watch one angle hit the wall first while the other still has room.",
          },
        ],
      },
    ],
  },
  turnAir: {
    threads: [
      plain(
        "Where the air pushes",
        ["Aerodynamic force acts at the center of pressure", "The moment restores when that point is behind the mass"],
        [
          "Air pushes on the rocket at the [center of pressure](https://en.wikipedia.org/wiki/Center_of_pressure_(fluid_mechanics)), which is its own point, separate from the center of mass. The rocket's lean against the air is the [angle of attack](https://en.wikipedia.org/wiki/Angle_of_attack).",
          "If the center of pressure sits behind the center of mass, that push [twists the nose back toward the wind](https://en.wikipedia.org/wiki/Static_margin), and the lean swings back through zero. If the center of pressure sits ahead of the mass, the same push twists the nose farther out, and the lean grows. The margin knob moves the center of pressure from one side of the mass to the other.",
        ],
      ),
      {
        id: "pressure",
        title: "The moment and the margin",
        focus: ["Aerodynamic force acts at the center of pressure", "The moment restores when that point is behind the mass"],
        segments: [
          { kind: "look", text: "The gold mark is the center of mass. The pale blue mark is the center of pressure. The rocket's lean is the angle of attack." },
          {
            kind: "relation",
            tex: String.raw`M = -q S d\, C_{N\alpha}\, \alpha\, \sigma`,
            stake: "The moment is proportional to the angle of attack $\\alpha$. Dynamic pressure $q$ grows with speed squared, $S$ is a reference area, and $d$ is a length, so together they scale how hard the air pushes. $C_{N\\alpha}$ is how steeply that push grows with angle. The minus sign and a positive margin $\\sigma$ make the moment oppose the lean.",
            gloss: String.raw`$\sigma$ is the margin.
Positive $\sigma$ puts the center of pressure behind the center of mass.`,
          },
          {
            kind: "contrast",
            tex: String.raw`\sigma < 0`,
            stake: "Move the center of pressure ahead of the mass and the moment has the same sign as the angle. The lean grows.",
          },
          {
            kind: "trace",
            text: "With a positive margin the angle of attack swings back through zero. With a negative margin the lean keeps growing.",
          },
        ],
      },
    ],
  },
  turnSlosh: {
    threads: [
      plain(
        "Fuel that can still move",
        ["The tank drains from the bottom", "The fuel that remains shifts the center of mass"],
        [
          "[Fuel in a tank is free to swing](https://en.wikipedia.org/wiki/Slosh_dynamics). The vehicle's [center of mass moves sideways with that swing](https://en.wikipedia.org/wiki/Center_of_mass), by the fuel's share of the whole mass. A full tank can shift the vehicle more than a nearly empty one.",
          "The tank drains from the bottom, so the fuel that remains rides higher as the burn goes on, and there is less of it. The swing can stay large while the shift of the center of mass shrinks. The swing knob is how far that remaining fuel rocks.",
        ],
      ),
      {
        id: "slosh",
        title: "Fuel that is still free to move",
        focus: ["The tank drains from the bottom", "The fuel that remains can swing and shift the center of mass"],
        segments: [
          { kind: "look", text: "The gold slug is the fuel still in the tank. It swings, and it rides higher as the tank empties." },
          {
            kind: "relation",
            tex: String.raw`x_{\mathrm{cm}} = \frac{m_{\mathrm{fuel}}}{m}\, x_{\mathrm{slug}}`,
            stake: "The vehicle center of mass moves sideways by the fuel's share of the mass.",
          },
          {
            kind: "trace",
            text: "The fuel trace falls as the burn goes on. The offset trace keeps the swing, and its size shrinks because there is less fuel left to move.",
          },
        ],
      },
    ],
  },
  vehAscent: {
    threads: [
      plain(
        "Push against weight",
        ["Thrust above weight speeds the vehicle upward", "T/W below 1 stays on the pad"],
        [
          "Thrust is a push along the climb. Weight is $mg$: here $m=2\\,\\mathrm{kg}$ and $g=9.81\\,\\mathrm{m/s^2}$, so weight is $19.6\\,\\mathrm{N}$. [When the push per kilogram is stronger than gravity, the vehicle speeds upward](https://en.wikipedia.org/wiki/Newton%27s_laws_of_motion). The readout $T/W$ is thrust over that weight.",
          "The mass stays fixed in this climb, so a steady thrust makes a steady acceleration. Speed then climbs in a straight line, and [height is that speed added up](https://en.wikipedia.org/wiki/Equations_of_motion). Drop thrust to or below weight and the vehicle never leaves the pad.",
        ],
      ),
      {
        id: "ascent",
        title: "Off the pad",
        focus: ["Thrust per mass minus gravity is the acceleration", "T/W = 1 is the break-even"],
        segments: [
          { kind: "look", text: "The flame stays on. Mass is 2 kg, so the thrust knob alone sets acceleration. The Thrust aside shows T/W and the 19.6 N weight." },
          {
            kind: "relation",
            tex: String.raw`a = T/m - g`,
            stake: "$T/m$ is the push per kilogram. Subtracting $g$ takes the weight off that push. Upward is positive. When $T \\le mg$, $a \\le 0$ and the pad hold keeps height and rate at zero.",
            gloss: String.raw`$T$ is thrust.
$m$ is $2\,\mathrm{kg}$.
$g$ is $9.81\,\mathrm{m/s^2}$.
Weight is $mg = 19.6\,\mathrm{N}$.`,
          },
          {
            kind: "contrast",
            tex: String.raw`v = at \qquad h = \tfrac{1}{2}at^{2}`,
            stake: "When $a>0$, speed is that acceleration times time, so the rate trace is a straight line whose slope is $a$. Height adds the speed up, which for a constant acceleration is half of $a$ times time squared.",
            gloss: String.raw`$v$ is the climb rate.
$h$ is height.
$t$ is the clock.`,
          },
          {
            kind: "relation",
            tex: String.raw`T/W = T/(mg)`,
            stake: "Thrust-to-weight above 1 climbs. Equal to 1 holds. Below 1 never leaves the pad from rest.",
            gloss: String.raw`$W$ is weight $mg$.
$T/W$ is the live readout.`,
          },
          {
            kind: "trace",
            text: "Raise thrust and the rate slope steepens. Lower it through 19.6 N and the climb flattens to a sit on the pad.",
          },
        ],
      },
    ],
  },
  vehNose: {
    threads: [
      plain(
        "The push follows the nose",
        ["Thrust points along the body", "Past ~66.9° the climb accel hits zero"],
        [
          "A rocket pushes along its nose. Point the nose straight up and the whole push fights weight. Lean the nose and [the push splits: the part along the vertical still fights weight, and the part across the vertical starts the vehicle moving downrange](https://en.wikipedia.org/wiki/Vector_decomposition). The gold arrow is thrust; the pale arrows are those two shares.",
          "One angle sets both parts. Raise the tilt toward $\\theta^\\star=\\arccos(mg/T)\\approx 66.9^\\circ$ and the climb acceleration falls to zero. Past that break-even the vehicle stays on the pad.",
        ],
      ),
      {
        id: "nose",
        title: "Cosine lifts, sine goes downrange",
        focus: ["Thrust is along the body axis", "Break-even when a_y = 0"],
        segments: [
          { kind: "look", text: "The rocket leans, and the flame leans with it. Thrust is 50 N on 2 kg (T/m = 25 m/s²). The Tilt aside shows a_y, a_x, and θ*." },
          {
            kind: "relation",
            tex: String.raw`a_y = (T/m)\cos\theta - g`,
            stake: "Tilt $\\theta$ is from vertical. Cosine is the share of the nose that still points up. When $(T/m)\\cos\\theta = g$, climb accel is zero.",
            gloss: String.raw`$T$ is $50\,\mathrm{N}$.
$m$ is $2\,\mathrm{kg}$.
$\theta$ is the tilt from vertical.
$g$ is $9.81\,\mathrm{m/s^2}$.`,
          },
          {
            kind: "contrast",
            tex: String.raw`a_x = (T/m)\sin\theta`,
            stake: "Sine is the share that points sideways. It grows as the nose leans, so downrange acceleration appears as climb acceleration gives way.",
          },
          {
            kind: "relation",
            tex: String.raw`\theta^\star = \arccos\!\left(\frac{mg}{T}\right)`,
            stake: "That is the break-even tilt, about $66.9^\\circ$ here. Beyond it $a_y \\le 0$ and the pad hold keeps height and downrange at zero from rest.",
            gloss: String.raw`$\theta^\star$ is the break-even tilt.
$mg/T$ is weight over thrust.`,
          },
          {
            kind: "trace",
            text: "While $a_y>0$, height and downrange both grow as the square of time. Scrub tilt through θ* and watch a_y cross zero on the readout.",
          },
        ],
      },
    ],
  },
  vehWind: {
    threads: [
      plain(
        "The angle the air arrives at",
        ["Attack starts at 90° when climb speed is zero", "A teaching side load drifts the vehicle downrange"],
        [
          "[Angle of attack is the angle between where the nose points and where the air is coming from](https://en.wikipedia.org/wiki/Angle_of_attack). Here the nose stays vertical and the wind comes from the side, so at liftoff ($v=0$) the air is pure crosswind and $\\alpha=90^\\circ$.",
          "[The climb also makes its own wind along the path](https://en.wikipedia.org/wiki/Relative_wind). As $v$ grows, the same $W$ is a smaller share of the relative wind, so $\\alpha$ falls. A teaching side load $a_x \\propto W$ also pushes the vehicle downrange so the wind is not only an angle on a chart.",
        ],
      ),
      {
        id: "wind",
        title: "Air across the nose",
        focus: ["α = arctan(W/v) with no silent speed floor", "Drift grows with wind"],
        segments: [
          { kind: "look", text: "The pale arrow is the wind. The nose stays vertical. At t = 0 the attack readout is 90° whenever W > 0." },
          {
            kind: "relation",
            tex: String.raw`\alpha = \arctan(W / v)`,
            stake: "Implemented as $\\mathrm{atan2}(W,v)$. When $v=0$ and $W>0$, $\\alpha=90^\\circ$. A larger $W$ raises the whole attack curve. A larger $v$ lowers $\\alpha$.",
            gloss: String.raw`$W$ is the wind.
$v$ is the climb rate.
$\alpha$ is the angle of attack.`,
          },
          {
            kind: "relation",
            tex: String.raw`a_x = 0.1\,W`,
            stake: "Teaching side load only: enough that downrange grows with wind. Real weathercocking and aero loads are richer; this scene keeps the nose fixed and shows that a crosswind can move the vehicle.",
            gloss: String.raw`$a_x$ is sideways acceleration.
$W$ is wind in m/s.`,
          },
          {
            kind: "trace",
            text: "Attack starts near 90° and falls as rate climbs. Raise the wind and both the attack curve and the downrange drift grow.",
          },
        ],
      },
    ],
  },
  vehEstimate: {
    threads: [
      plain(
        "Predict, then pull toward the sensor",
        ["This is a fixed-gain filter, not a Kalman filter", "The chart shows errors from truth so the wave is visible"],
        [
          "A sensor reports a height with a wave on it. You can also predict the next height from the thrust, using the speed you already believe and the acceleration that thrust should produce.",
          "The estimate starts from that prediction, then moves partway toward the sensor. [The gain is how hard that pull is: a fixed number you set, not a Kalman gain from variances](https://en.wikipedia.org/wiki/State_observer). High gain follows the wave. Low gain (or zero) trusts the model. The pale mark is the estimate; the rocket is truth.",
        ],
      ),
      {
        id: "estimate",
        title: "Predict, then correct",
        focus: ["The predictor uses the thrust model", "The corrector pulls toward the sensor", "Noise sets the wave amplitude"],
        segments: [
          { kind: "look", text: "The chart plots sensor − truth and estimate − truth, so the wave is visible. Raise Noise to grow the wave. Raise Gain and the estimate error starts to track the sensor error." },
          {
            kind: "relation",
            tex: String.raw`z = h + A\sin(2.2\,t)`,
            stake: "The sensor reports the true height $h$ with a wave of amplitude $A$ (the Noise knob). The wave is what a hard correction will start to follow.",
            gloss: String.raw`$z$ is the sensor.
$h$ is the true height.
$A$ is the noise amplitude.
$t$ is the clock.`,
          },
          {
            kind: "contrast",
            tex: String.raw`\hat{x}^{-} = \hat{x} + \hat{v}\,\Delta t + \tfrac{1}{2}a\Delta t^{2}`,
            stake: "The prediction carries the last estimate forward by its believed speed, and adds the acceleration from the thrust model over the step. $a$ is that thrust per mass, with gravity taken off.",
            gloss: String.raw`$\hat{x}$ is the estimate.
$\hat{v}$ is the estimated rate.
$\Delta t$ is the step.`,
          },
          {
            kind: "relation",
            tex: String.raw`\hat{x} = \hat{x}^{-} + K(z-\hat{x}^{-})\Delta t`,
            stake: "The gap is the sensor minus the prediction. The fixed gain $K$ pulls the estimate across a fraction of that gap on this step. The rate is pulled by half of the same correction. This is not $K = P/(P+R)$ from a Kalman filter.",
          },
          {
            kind: "trace",
            text: "At Gain 0 the estimate error stays near zero while the sensor error is the full wave. Raise the gain and the estimate error starts to hug that wave. Raise Noise and both error traces grow.",
          },
        ],
      },
    ],
  },
  vehGuide: {
    threads: [
      plain(
        "Aim at a target from an estimate",
        ["Guidance commands a pitch from filtered downrange and height", "The body follows that live command"],
        [
          "[Guidance is the direction you want the nose to point](https://en.wikipedia.org/wiki/Guidance_system), built here from a filtered estimate of where the vehicle is. The target is a point on the ground a chosen downrange away.",
          "Each step, noisy measurements of downrange and height are filtered, then the command is the angle from that estimate toward the target. The body closes on the command with $\\tau = 0.7\\,\\mathrm{s}$. Thrust stays on the nose, so the path bends as the nose tips.",
        ],
      ),
      {
        id: "guide",
        title: "Closed-loop aim",
        focus: ["The command comes from the estimate and the target", "The body lags the command", "Downrange and height show the flown path"],
        segments: [
          { kind: "look", text: "The chart is downrange and height. The pale arrow is the live pitch command. Drag Target to move the ground aim point." },
          {
            kind: "relation",
            tex: String.raw`\theta_{\mathrm{cmd}} = \mathrm{atan2}(x_t - \hat{x},\, \hat{z})`,
            stake: "The command is the pitch from vertical that points from the estimated position $(\\hat{x},\\hat{z})$ toward the ground target $x_t$. The estimate is a fixed-gain filter on noisy measurements, the same idea as the previous scene.",
            gloss: String.raw`$x_t$ is the target downrange.
$\hat{x}$ is the estimated downrange.
$\hat{z}$ is the estimated height.`,
          },
          {
            kind: "relation",
            tex: String.raw`\dot\theta = (\theta_{\mathrm{cmd}} - \theta)/\tau`,
            stake: "The rate of pitch is the miss divided by $\\tau = 0.7\\,\\mathrm{s}$. A larger miss turns the nose faster. The command itself moves every step as the estimate and geometry change.",
            gloss: String.raw`$\theta$ is the nose pitch.
$\theta_{\mathrm{cmd}}$ is the command.
$\tau$ is the follow time.`,
          },
          {
            kind: "contrast",
            tex: String.raw`a_x = (T/m)\sin\theta`,
            stake: "Thrust stays on the nose, so the sideways acceleration uses the body's pitch, which lags the command. A farther target asks for a larger tip and builds more downrange.",
          },
          {
            kind: "trace",
            text: "A farther target asks for a larger tip early, so the path builds more downrange by the end. The pitch follows the live aim as the estimate updates; there is no scripted step at two seconds.",
          },
        ],
      },
    ],
  },
  vehAuto: {
    threads: [
      plain(
        "Schedule the gain as the vehicle changes",
        ["Fixed K stays put while mass and air change", "Scheduled K scales with mass so the twist matches inertia"],
        [
          "The autopilot twists the nose toward a 12° command that arrives at 0.4 s. While that happens the vehicle burns fuel and speeds up, so inertia falls and [air loads grow with speed squared](https://en.wikipedia.org/wiki/Dynamic_pressure).",
          "[Gain scheduling changes K as the vehicle changes](https://en.wikipedia.org/wiki/Gain_scheduling). Here Schedule sets K ∝ m, so K/I stays nearer a constant. Fixed holds one K for the whole burn. The air term is written κ and is restoring in this scene (a teaching choice); many real rockets are aerodynamically unstable.",
        ],
      ),
      {
        id: "auto",
        title: "Fixed K or K(m)",
        focus: ["K twists the nose toward the command", "κ = c v² is air stiffness", "Schedule scales K with mass"],
        segments: [
          { kind: "look", text: "Pitch is the nose. Gain used is the live K (right axis). Switch Schedule between fixed and K∝m. Ask for speed v or air stiffness κ on the chart." },
          {
            kind: "relation",
            tex: String.raw`m = 3 - 1.6\,t/8`,
            stake: "Mass falls in a straight line over eight seconds. Inertia is I = m/4, so the vehicle gets easier to twist as it lightens.",
            gloss: String.raw`$m$ is mass in kilograms.
$t$ is the clock in seconds.
$I$ is inertia.`,
          },
          {
            kind: "contrast",
            tex: String.raw`I\dot\omega = K(\theta_{\mathrm{cmd}} - \theta) - b\omega - \kappa\theta`,
            stake: "K times the miss twists toward the command. b damps pitch rate. κθ is the air stiffness term, restoring here (positive κ pulls toward straight up). The sign is a teaching choice.",
            gloss: String.raw`$K$ is the gain used.
$\omega$ is pitch rate.
$b$ damps that rate.
$\kappa$ is air stiffness.`,
          },
          {
            kind: "relation",
            tex: String.raw`\kappa = c\,v^{2}`,
            stake: "Air stiffness grows with speed squared. Early on, K sets the pitch. Later, with the vehicle fast, κ eases the nose back from the command.",
            gloss: String.raw`$v$ is speed.
$c$ is the coefficient on that speed squared.`,
          },
          {
            kind: "relation",
            tex: String.raw`K = K_0\,m/m_0`,
            stake: "With Schedule on, the live gain tracks mass, so K/I changes less as the tank empties. With Schedule off, K stays at the Gain knob the whole burn.",
            gloss: String.raw`$K_0$ is the Gain knob.
$m_0$ is the start mass, 3 kg.`,
          },
          {
            kind: "trace",
            text: "With Schedule on, Gain used falls with mass. Pitch still rises toward 12°, then sags a little as κ grows. Fixed K keeps the right-axis gain flat.",
          },
        ],
      },
    ],
  },
  desCost: {
    threads: [
      plain(
        "A score that chooses the pull",
        ["The weight on height sets the gain", "The cost does not contain the ground"],
        [
          "A landing can be scored. One part of the score is how far the height sits from the pad, and another is how much thrust you spend getting there. [Those weights choose the gain that pulls the vehicle home](https://en.wikipedia.org/wiki/Linear-quadratic_regulator).",
          "A heavier weight on height makes a stronger gain, so the fall is checked sooner and the thrust peaks later and higher. Thrust cannot go below zero, so a strong early pull that would want negative thrust just sits on that floor. The score has no ground in it, so a hard pull can pass through the pad and come back.",
        ],
      ),
      {
        id: "gain",
        title: "The weight sets both gains",
        focus: ["kp grows with the square root of q/r", "kd brakes the rate", "thrust floors at zero"],
        segments: [
          {
            kind: "look",
            text: "The vehicle starts 18 m up, already falling at 6 m/s. The gain is the only thing steering it, on a mass of 2 kg.",
          },
          {
            kind: "relation",
            tex: String.raw`u = -k_p h - k_d v`,
            stake: "u is the acceleration left after gravity. It pulls height toward the pad and brakes a downward rate. Thrust is that acceleration plus gravity, times the mass, then floored at zero.",
            gloss: String.raw`$u$ is the extra acceleration.
$h$ is height.
$v$ is the climb rate.
$k_p$ acts on height.
$k_d$ acts on rate.`,
          },
          {
            kind: "relation",
            tex: String.raw`k_p = \sqrt{q/r},\quad k_d = \sqrt{2 k_p}`,
            stake: "q is the weight on height and r is the weight on thrust. A larger q, or a smaller r, raises the height gain. The rate gain is the square root of twice that, so a fast fall gets a brake that matches the height gain.",
            gloss: String.raw`$q$ is the weight on height.
$r$ is the weight on thrust.`,
          },
          {
            kind: "trace",
            text: "At two seconds a weight of 1 has the height near 3 m, and a weight of 4 has it under 1 m. Both traces can cross the pad. Raising q can keep thrust on the zero floor at the start, then peak later and higher once the demand rises.",
          },
        ],
      },
    ],
  },
  desPath: {
    threads: [
      plain(
        "Walls on the way down",
        ["Thrust has a floor and a ceiling", "The cone keeps the path over the pad"],
        [
          "The way down has walls. Thrust can only be so small and so large, and the vehicle has to stay inside [a cone that opens upward from the pad](http://www.larsblackmore.com/iee_tcst13.pdf). Outside that cone the pad is no longer under the vehicle in the way the landing allows.",
          "A tight cone meets the vehicle early and pushes the path onto the wall. A wide cone leaves room to come in on the guidance, and the wall stays pale and unused.",
        ],
      ),
      {
        id: "cone",
        title: "The cone and the thrust ring",
        focus: ["Downrange is limited by height and the cone angle", "Thrust magnitude stays between two bounds"],
        segments: [
          {
            kind: "look",
            text: "The vehicle starts 4 m downrange and 36 m up, inside an 8° cone. The pale wall on the chart is the allowed |x| at the current height.",
          },
          {
            kind: "relation",
            tex: String.raw`|x| \le z \tan\theta`,
            stake: "x is downrange and z is height. θ is the cone angle from vertical. At a given height the vehicle may sit no farther from the pad than the height times the tangent of that angle. Past the wall, a sideways acceleration pushes it back in, and the chart marks outside cone.",
            gloss: String.raw`$\theta$ is the cone angle.
$x$ is downrange.
$z$ is height.`,
          },
          {
            kind: "relation",
            tex: String.raw`6 \le \|T\| \le 55`,
            stake: "T is the thrust vector in newtons. The floor keeps a push on the engine, and the ceiling is as hard as this engine pushes. The cone is the knob. The thrust bounds stay put (see the Cone aside).",
            gloss: String.raw`$\|T\|$ is the size of the thrust.`,
          },
          {
            kind: "trace",
            text: "A 10° cone pulls the vehicle toward the wall early. A 35° cone leaves more room, so downrange stays well inside the pale wall. Opening past about 35° does not change the path further.",
          },
        ],
      },
    ],
  },
  desLoss: {
    threads: [
      plain(
        "A hole the answer does not use",
        ["Allowed thrust is a ring", "The slack stays equal to the thrust you fly"],
        [
          "The thrusts you may use form a ring. The hole in the middle is every push smaller than the floor, and the outside of the ring is every push larger than the engine. [A relaxed problem fills that hole and still charges you for a slack at least as large as the floor](http://www.larsblackmore.com/iee_tcst13.pdf).",
          "On this descent the thrust you see stays on that charge. It matches the command while the command sits outside the hole. Raise the floor until it crosses the command, and the early push is lifted onto the rim. Lift it far enough and the vehicle leaves the pad.",
        ],
      ),
      {
        id: "ring",
        title: "The slack and the rim",
        focus: ["The cost pays for the slack", "The flown thrust stays at least as large as the floor"],
        segments: [
          {
            kind: "look",
            text: "Command is the push the guidance asked for. Thrust is the push after the floor and the ceiling. They share a line while the floor sits under the whole burn.",
          },
          {
            kind: "relation",
            tex: String.raw`\|T\| \le \sigma,\quad T_{\min} \le \sigma \le T_{\max}`,
            stake: "σ is the slack, the push you pay for. T is the thrust vector. The first inequality lets the real push sit inside the circle of radius σ, which fills the hole. The second keeps the slack between the floor and the ceiling. The cost adds σ up over the flight, so a larger slack costs more fuel.",
            gloss: String.raw`$\sigma$ is the slack.
$T_{\min}$ is the floor.
$T_{\max}$ is the ceiling.`,
          },
          {
            kind: "relation",
            tex: String.raw`\|T\| = \sigma`,
            stake: "The landing you see puts the thrust on the slack. A command inside the hole is raised to the floor, and then the slack is that floor. A command between the floor and the ceiling is flown as itself, and the slack equals it. For minimum fuel the optimum never wastes slack above |T|, so the relaxation is exact: the ring and the filled disc share the same cheapest path.",
          },
          {
            kind: "trace",
            text: "With the floor near 12 N the thrust matches the command and the vehicle meets the pad. Raise the floor to 20 N and thrust rides the rim while command can climb past the 55 N ceiling. The height and rate readouts show whether that still lands.",
          },
        ],
      },
    ],
  },
  desScvx: {
    threads: [
      plain(
        "Fly it, solve, step, fly again",
        ["Each pass builds a new thrust near the last one", "A trust region limits how far that step may go"],
        [
          "The first thrust is only a weak upward push, smaller than the weight, so the rocket falls through the pad. [Successive convexification flies that profile, solves a pad-aiming subproblem (stand-in for the convex program), and steps toward it](https://arxiv.org/abs/1608.05133).",
          "The new thrust is not free to jump to the full solve. A trust region caps how far each sample may move, and a small virtual vertical slack can close leftover pad error once the step is already near the pad. If the trial flight is worse, the step is rejected and the region shrinks.",
        ],
      ),
      {
        id: "pass",
        title: "Solve, trust, slack, accept",
        focus: ["The subproblem aims at the pad", "The trust region limits the thrust step", "Virtual slack closes leftover miss"],
        segments: [
          {
            kind: "look",
            text: "The solid height is the pass you are on. The dashed height is the pass before it. On the first pass the two lines are the same weak push.",
          },
          {
            kind: "relation",
            tex: String.raw`\|T_{k+1} - T_k\| \le \Delta_k`,
            stake: "$T_k$ is the thrust profile on this pass. The solved pad-aiming thrust is pulled back so the step stays inside a trust radius $\\Delta_k$. An accepted trial grows $\\Delta$. A rejected trial shrinks it.",
            gloss: String.raw`$k$ is the pass.
$\Delta_k$ is the trust radius.`,
          },
          {
            kind: "relation",
            tex: String.raw`a = -6r/t_f^{2} - 4v/t_f`,
            stake: "The pad-aiming subproblem uses this acceleration from the state and the time left. Virtual slack then adds a small vertical push when the trust-limited trial still ends a little off the pad.",
            gloss: String.raw`$t_f$ is the time left.
$r$ is the distance still to the pad.
$v$ is the speed.`,
          },
          {
            kind: "trace",
            text: "Pass 1 falls well under the pad. Later passes cut that miss until the height meets the pad. The aside reports end-height error, the trust radius, and whether the step out of that pass was accepted.",
          },
        ],
      },
    ],
  },
  desGusto: {
    threads: [
      plain(
        "Hard trust, merit accept",
        ["Each iterate stays inside a hard trust region", "A bad trial shrinks Δ; a good one can grow it"],
        [
          "GuSTO starts from a weak first push and iterates. Each step linearizes about the path just flown, takes a hard trust-limited correction, and keeps the step only when a merit test (end-height error) improves. [There is no virtual control; feasibility is enforced by the trust region and the accept/reject rule](https://arxiv.org/abs/1903.00155).",
          "A modest starting Δ can walk toward the pad. A huge starting Δ trusts a local model that is wrong far from the path, so the trial overshoots and is rejected. The dashed height is the weak first push.",
        ],
      ),
      {
        id: "region",
        title: "Why a huge region can fail",
        focus: ["Δ is a hard cap on the thrust step", "Merit accepts or shrinks", "Too large a step trusts a bad local model"],
        segments: [
          {
            kind: "look",
            text: "The first push is 14 N straight up, under the weight of the 2 kg vehicle. Drag the trust knob. The solid height is after a few GuSTO iterates. The dashed height is that first push.",
          },
          {
            kind: "relation",
            tex: String.raw`\|T_{\mathrm{new}} - T_{\mathrm{old}}\| \le \Delta`,
            stake: "Δ is the trust region in newtons. The local pad-aim about the flown path proposes T_new. If the trial end-height error is worse than the current flight, GuSTO rejects the step and shrinks Δ.",
            gloss: String.raw`$\Delta$ is the trust region.
$T_{\mathrm{old}}$ is the thrust on this iterate.
$T_{\mathrm{new}}$ is the trial thrust.`,
          },
          {
            kind: "trace",
            text: "A starting region near 2–3 N improves the fall toward the pad. A starting region near 6 N takes a huge step on a bad local model, overshoots, and the merit test rejects it. The aside counts accepts and rejects.",
          },
        ],
      },
    ],
  },
  desReplan: {
    threads: [
      plain(
        "The plan is for the state you have",
        ["A gust changes the sideways speed", "Limits can make a recovery impossible"],
        [
          "A plan is a thrust that would reach the pad from the state you have now. [Recomputing that plan as the clock runs is the idea behind model predictive control](https://en.wikipedia.org/wiki/Model_predictive_control).",
          "At 2.2 seconds a sideways gust changes the speed. The live plan turns to cancel it, but thrust stays at or under 40 N and the tilt stays inside a 12° cone from vertical. The dashed downrange is the plan that never felt the gust. A large gust saturates those limits and misses the pad.",
        ],
      ),
      {
        id: "gust",
        title: "Replan inside the limits",
        focus: ["The gust is added to sideways speed", "Thrust and cone caps bind on large gusts", "A miss means the limits were not enough"],
        segments: [
          {
            kind: "look",
            text: "Downrange is the live plan after the gust. The dashed line is the old plan. Markers call out the gust, an outside-cone moment, and a miss when the end is off the pad.",
          },
          {
            kind: "relation",
            tex: String.raw`a = -6r/t_f^{2} - 4v/t_f`,
            stake: "The acceleration is rebuilt from the distance and the speed the vehicle has now, and from the time left. After the gust, v includes the new sideways speed. The commanded thrust is then clipped to 40 N and to the 12° cone.",
            gloss: String.raw`$r$ is the distance still to the pad.
$v$ is the speed, gust included.
$t_f$ is the time left.`,
          },
          {
            kind: "trace",
            text: "A gust of 6 m/s recovers under the limits. A gust of 14 m/s drives the vehicle outside the cone, saturates thrust, and ends off the pad. Replanning helps only while the limits still have room.",
          },
        ],
      },
    ],
  },
  desMiss: {
    threads: [
      plain(
        "When no soft landing exists",
        ["Soft landing needs |rate| ≤ 0.5 m/s at the pad", "Fuel-out and a hard ceiling are two ways to miss"],
        [
          "A soft landing here means the vehicle reaches the pad with $|\\dot z| \\le 0.5\\,\\mathrm{m/s}$. Faster than that is a miss. Fuel is one way to fail: [each newton of thrust spends budget](https://en.wikipedia.org/wiki/Tsiolkovsky_rocket_equation), and when the tank is empty thrust is gone.",
          "The other way is [infeasibility](https://en.wikipedia.org/wiki/Feasible_region): the Cause knob sets a ceiling of 18 N that cannot kill this fall from 45 m / −3 m/s. Status reads INFEASIBLE because no thrust history under that bound meets the soft-landing speed.",
        ],
      ),
      {
        id: "budget",
        title: "Thrust spends the budget",
        focus: ["Fuel-out is a miss when impact speed exceeds 0.5 m/s", "A hard ceiling is INFEASIBLE"],
        segments: [
          {
            kind: "look",
            text: "Start 45 m up at −3 m/s. Cause = fuel uses the Fuel knob. Cause = ceiling locks T ≤ 18 N with ample fuel.",
          },
          {
            kind: "relation",
            tex: String.raw`\dot m_f = -T / v_e`,
            stake: "In fuel mode, $m_f$ is the budget left. At zero fuel the thrust is zero for the rest of the flight. Soft landing still needs $|\\dot z| \\le 0.5\\,\\mathrm{m/s}$ at the pad.",
            gloss: String.raw`$m_f$ is fuel left.
$T$ is thrust.
$v_e$ is exhaust speed, $280\,\mathrm{m/s}$.`,
          },
          {
            kind: "contrast",
            tex: String.raw`| \dot z(t_f) | \le 0.5`,
            stake: "That is the touchdown-speed criterion. Status soft / miss / INFEASIBLE is judged from the rate when height first hits the pad.",
            gloss: String.raw`$t_f$ is touchdown.
$0.5\,\mathrm{m/s}$ is the soft limit in this scene.`,
          },
          {
            kind: "trace",
            text: "Fuel 0.4 kg → miss. Fuel near 0.7 kg → soft. Flip Cause to ceiling → INFEASIBLE with impact still fast.",
          },
        ],
      },
    ],
  },
  propEq: {
    threads: [
      plain(
        "Throw mass, keep the speed",
        ["Final speed follows the exhaust speed and the mass ratio", "The tank gets lighter, so the same push accelerates more"],
        [
          "A rocket gets its speed by throwing mass out the back. The faster that mass leaves, and the larger the share of the rocket that is propellant, the more speed the rocket keeps. [The rocket equation](https://en.wikipedia.org/wiki/Tsiolkovsky_rocket_equation) is that trade, with gravity off and no air in this scene.",
          "Drag the rocket to change the mass ratio. At 4, a 4000 kg rocket finishes at 1000 kg. The Isp knob is [how long one kilogram of propellant can produce one newton](https://en.wikipedia.org/wiki/Specific_impulse): 300 s here, which is an exhaust speed of 2942 m/s. The burn takes 10 s and the final speed is 4078 m/s.",
        ],
      ),
      {
        id: "log",
        title: "Speed is a logarithm",
        focus: ["Final speed is ve times ln of the mass ratio", "Acceleration rises as the mass falls", "Doubling the ratio adds ve ln 2"],
        segments: [
          {
            kind: "look",
            text: "Speed climbs as a curve, and acceleration climbs with it. The rocket is lightest at the end, so the last seconds are the hardest push.",
          },
          {
            kind: "relation",
            tex: String.raw`\Delta v = v_e \ln\frac{m_0}{m_f} \qquad v_e = I_{sp}\, g_0`,
            stake: "The final speed is the exhaust speed times the natural log of the mass ratio. Specific impulse turns into an exhaust speed through standard gravity, 9.80665 m/s². In this scene that g0 only defines ve. The rocket is in free space.",
            gloss: String.raw`$v_e$ is the exhaust speed.
$I_{sp}$ is the specific impulse.
$g_0 = 9.80665\,\mathrm{m/s^2}$.
$m_0$ is 4000 kg. $m_f = m_0$ divided by the ratio.`,
          },
          {
            kind: "relation",
            tex: String.raw`a = \frac{v_e \dot m}{m}`,
            stake: "Acceleration is the exhaust speed times the mass-flow rate, divided by the mass that is left. At ratio 4 and 300 s the burn starts at 221 m/s² and ends at 883 m/s². While gravity is off, the burn time drops out of the final speed: the same ratio and the same exhaust speed finish together.",
            gloss: String.raw`$\dot m$ is how fast the tank empties.
The default tank empties 3000 kg in 10 s, so $\dot m = 300\,\mathrm{kg/s}$.`,
          },
          {
            kind: "trace",
            text: "Doubling Isp doubles the final speed, because $v_e$ is proportional to $I_{sp}$. Raising the ratio from 2 to 4 adds $v_e \\ln 2 = 2040\\,\\mathrm{m/s}$, and the speed goes from 2039 m/s to 4078 m/s.",
          },
        ],
      },
    ],
  },
  propNozzle: {
    threads: [
      plain(
        "The bell and the air",
        ["Thrust is exhaust momentum plus a pressure push on the bell", "The best push is where the exit pressure matches the air"],
        [
          "The exhaust leaves through a bell. A wider bell lets the gas expand further, which raises the speed of the exhaust and lowers the pressure at the lip. [That lip pressure, set against the air outside, pushes on the bell](https://en.wikipedia.org/wiki/Rocket_engine_nozzle).",
          "Drag sets the expansion, the exit area divided by the throat area. This rocket is on a clock from the pad to 60 km in 12 s, and the air thins as it climbs. At expansion 16 the sea-level specific impulse is 295 s and the vacuum value on the same engine is 326 s. The pressure push is zero at 4.2 km, where the exit pressure equals the air.",
        ],
      ),
      {
        id: "bell",
        title: "Momentum plus pressure",
        focus: ["Thrust has a momentum term and a pressure term", "Isp is thrust per unit of weight flow", "A separated plume leaves the ideal term behind"],
        segments: [
          {
            kind: "look",
            text: "Isp starts at the sea-level value and climbs toward the vacuum value as the air thins. The pressure term crosses zero at the matched mark.",
          },
          {
            kind: "relation",
            tex: String.raw`T = \dot m v_e + (p_e - p_a) A_e \qquad I_{sp} = \frac{T}{\dot m g_0}`,
            stake: "The first term is the momentum leaving every second. The second term is the pressure difference across the exit area. When $p_e = p_a$ that term is zero and the bell is matched. A wide bell at the pad has $p_e < p_a$, so the pressure term is negative and the sea-level Isp sits below the vacuum Isp.",
            gloss: String.raw`$\dot m = 25\,\mathrm{kg/s}$.
$p_e$ is the pressure at the lip.
$p_a = 101325\,\mathrm{e}^{-h/8500}$ pascals, with $h$ in metres.
$A_e$ is the exit area.
The chamber is 9.7 MPa and 3600 K, with $\gamma = 1.22$ and $R = 380\,\mathrm{J/(kg\cdot K)}$.
The matched altitude at expansion 16 is 4.2 km.`,
          },
          {
            kind: "trace",
            text: "Open the bell and the vacuum Isp rises, while the sea-level number falls. At expansion 80 the ideal sea-level Isp is 197 s and the vacuum Isp is 350 s. The exit pressure there is already under 0.4 of the sea-level air, which is where a real plume separates. The curve stays on the ideal term past that point, and the aside says so.",
          },
        ],
      },
    ],
  },
  propStage: {
    threads: [
      plain(
        "Drop the empty shell",
        ["Two stages leave the lower structure behind", "The speeds match until the drop"],
        [
          "One tank has to carry its empty shell for the whole burn. Two stages burn, drop the lower shell, and the upper burn pushes a lighter vehicle. [That is staging](https://en.wikipedia.org/wiki/Multistage_rocket).",
          "Both vehicles carry 1000 kg of payload and 8000 kg of propellant, and both throw the exhaust at 2942 m/s. Structure is 0.08 kg of dry mass for each kilogram of propellant in that stage. At a split of 0.75 the first burn is 2865 m/s and the second is 2948 m/s, 5814 m/s together. The pale vehicle keeps the shell and ends at 5211 m/s. Until the drop, both have burned the same propellant from the same mass, so the traces lie on each other.",
        ],
      ),
      {
        id: "split",
        title: "Two logs, then add",
        focus: ["Each burn is its own rocket equation", "Zero structure makes the split irrelevant", "The fastest split sits near 0.76"],
        segments: [
          {
            kind: "look",
            text: "The lower stage peels off at the marker and stays there. The upper stage keeps climbing. The pale vehicle is the one tank that never drops anything.",
          },
          {
            kind: "relation",
            tex: String.raw`\Delta v = v_e \ln\frac{m_0}{m_b} + v_e \ln\frac{m_b'}{m_f}`,
            stake: "Each stage spends its own propellant, and the speeds add. $m_b$ is the stack at the end of the first burn, still holding the lower shell. $m_b'$ is the upper stage after that shell is gone. The single-stage vehicle uses one log from the full stack down to the payload plus every shell.",
            gloss: String.raw`$v_e = 2942\,\mathrm{m/s}$.
The split is the share of the 8000 kg that rides in the first stage.
Structure $k$ is kilograms of dry mass per kilogram of propellant in that stage.`,
          },
          {
            kind: "trace",
            text: "Set structure to 0 and both vehicles end at 6464 m/s, which is $2942 \\ln 9$. There is nothing to drop, so the split stops mattering. At 0.08 the split that finishes fastest is 0.76.",
          },
        ],
      },
    ],
  },
  propLoss: {
    threads: [
      plain(
        "Gravity takes a share",
        ["The free-space speed is 4078 m/s", "A longer burn hands more of it to gravity"],
        [
          "The rocket equation's 4078 m/s is the speed this tank would have in free space. Point the same burn straight up and [gravity pulls for the whole time the engine is on](https://en.wikipedia.org/wiki/Gravity_drag). A longer burn gives gravity longer.",
          "Full thrust is 80 kN. The 3000 kg of propellant is gone in 110 s, gravity has taken 1082 m/s, and the speed at burnout is 2996 m/s. Half thrust, 40 kN, takes 221 s and finishes at 1914 m/s. Quarter thrust, 20 kN, sits under the 39.2 kN starting weight. The vehicle stays on the pad, engine burning, through the 240 s clock. It lifts once the mass falls to 2039 kg, at 289 s. The picture is a log of altitude so the pad stays in frame, and the readout is the true height.",
        ],
      ),
      {
        id: "gap",
        title: "The burn length, times g",
        focus: ["The subtraction applies while the vehicle is flying", "The ideal line counts propellant burned on the pad", "After burnout the vehicle coasts"],
        segments: [
          {
            kind: "look",
            text: "The ideal line is the free-space speed of the propellant already burned. The speed line is what the vehicle actually has. The gap at the burnout marker is g times the burn.",
          },
          {
            kind: "relation",
            tex: String.raw`v = v_e \ln\frac{m_0}{m_f} - g\, t_b`,
            stake: "This is the speed at burnout when the vehicle flew the whole burn. $g$ is 9.81 m/s² and $t_b$ is how long the engine was on in flight. Hold the vehicle on the pad while thrust is at or under its weight: the mass still falls, the speed stays zero, and the ideal line still climbs. The vehicle lifts when the mass drops below thrust divided by g.",
            gloss: String.raw`$v_e \ln(m_0/m_f) = 4078\,\mathrm{m/s}$ for this tank.
Full thrust burns for 110 s. Half thrust burns for 221 s.
Quarter thrust reaches liftoff mass at 289 s.`,
          },
          {
            kind: "trace",
            text: "After the tank is empty the ideal line holds still and the vehicle coasts, so gravity keeps taking speed. The 2996 m/s and the 1914 m/s are the speeds at the burnout marker. The clock runs 240 s, and the chart is how you walk through a burn that long.",
          },
        ],
      },
    ],
  },
  freqMargin: {
    threads: [
      plain(
        "Spare lag, spare gain",
        ["The chart is how strongly the loop answers, and how late", "45° of spare lateness and 6 dB of spare gain"],
        [
          "The chart shows this hover at every speed of wiggle. One curve is how strongly the rocket answers. The other is how late that answer is. [That pair is a Bode plot](https://en.wikipedia.org/wiki/Bode_plot). Drag the rocket to add delay. The hover and the chart move together.",
          "The usual aim is about 45° of spare lateness and 6 dB of spare gain. Spare gain here is room above the gain that just holds this hover. Lower that gain through the floor and the rocket runs away. Spend the spare lateness, with the delay or by taking Kd down to 0, and the rocket rings.",
        ],
      ),
      {
        id: "margins",
        title: "Where the loop crosses 0 dB",
        focus: ["Phase margin is spare lag at the gain crossover", "Gain margin is room above the hold", "Kd at 0 rings, and Kp under the floor runs away"],
        segments: [
          {
            kind: "look",
            text: "The gold line is magnitude in decibels. The teal line is phase in degrees. The mark is the gain crossover, where the magnitude crosses 0 dB. The clock plays the hover. A click on the chart does not move that clock.",
          },
          {
            kind: "relation",
            tex: String.raw`L(s)=\frac{K_p+K_d s}{m(s^2-\mu^2)}`,
            stake: "This is the open-loop gain of the same hover as The loop. $m$ is 2.5 kg and $\\mu$ is 1.5 per second, so $m\\mu^2$ is 5.625 N/m. The controller is $K_p$ plus $K_d$ times $s$. At the default, $K_p$ is 11.2 and $K_d$ is 5.5.",
            gloss: String.raw`Phase margin is $\arctan(\omega_c K_d/K_p)$, in degrees, then minus the delay $\omega_c\tau$.
Gain margin is $20\log_{10}(K_p/(m\mu^2))$. It is how far $K_p$ sits above the floor. Raising $K_p$ adds margin. Lowering it through 5.625 lets the hover run away.`,
          },
          {
            kind: "trace",
            text: "At these gains the crossover is 2.01 rad/s, the phase margin is 45°, and the gain margin is 6.0 dB. A delay of 0.39 s spends that phase margin. Kd at 0 leaves the phase on −180° and the hover rings without settling. Kp at 4 sits 3.0 dB under the floor, there is no crossover, and the height runs away without ringing. The hover scene in The loop uses Kp 22 and Kd 15 on this same rocket, which is 11.8 dB and about 76°.",
          },
        ],
      },
    ],
  },
  freqDelay: {
    threads: [
      plain(
        "Late on purpose",
        ["A delay costs phase equal to frequency times the wait", "A sample-and-hold costs about half of that"],
        [
          "A height reading that arrives late is answering a rocket that has already moved. [That lateness is a phase lag of frequency times the delay](https://en.wikipedia.org/wiki/Time_delay). It spends the spare lateness from the margins scene.",
          "A sample-and-hold is a second, smaller lag. The loop waits for the next sample and then holds the thrust. [That hold costs about half the phase of a pure delay of the same length](https://en.wikipedia.org/wiki/Zero-order_hold). Drag the rocket to change the sample time. The delay knob is the other lag.",
        ],
      ),
      {
        id: "lag",
        title: "Two different lags",
        focus: ["Delay keeps the magnitude and spends ωτ", "The hold spends about ωT/2 and trims the magnitude", "Together they spend the phase margin"],
        segments: [
          {
            kind: "look",
            text: "The gains stay the ones from The margins. The delay knob moves only the phase. The sample knob moves the phase a smaller amount and also trims the magnitude.",
          },
          {
            kind: "relation",
            tex: String.raw`e^{-s\tau},\qquad \frac{1-e^{-sT}}{sT}`,
            stake: "The first factor is a pure delay of $\\tau$ seconds. Its magnitude is 1, and its phase is $-\\omega\\tau$ radians. The second is a zero-order hold of $T$ seconds. Its phase is about $-\\omega T/2$, and its magnitude is $\\mathrm{sinc}(\\omega T/2)$.",
            gloss: String.raw`$\omega$ is the frequency in rad/s.
At the gain crossover those two phases come straight off the phase margin.
Equal times do not cost equal phase: the hold is about half the delay.`,
          },
          {
            kind: "trace",
            text: "Set the delay and the sample time both to 0.2 s. At the crossover near 2.0 rad/s the delay takes 23° and the hold takes 11°, and the phase margin falls to 10°. Raise the delay toward 0.39 s, with the sample time small, and the margin reaches 0°. The hover starts to grow.",
          },
        ],
      },
    ],
  },
  freqNotch: {
    threads: [
      plain(
        "One frequency, then take it out",
        ["A bend rings at one frequency", "A notch tuned to that frequency removes the peak"],
        [
          "A bend in the airframe rings at one frequency. On the Bode plot that ring is a peak. The hidden mode in The loop was a different thing: it grew on its own, with no frequency to point at.",
          "[A notch is a filter tuned to one frequency](https://en.wikipedia.org/wiki/Band-stop_filter). Turn it on and the peak at 8 rad/s leaves, and the ring that was growing in the height settles. Drag the rocket to move the bend off that frequency and the peak comes back.",
        ],
      ),
      {
        id: "peak",
        title: "The peak, then the notch",
        focus: ["The bend is a light resonance on the thrust", "The notch cancels that resonance only at 8 rad/s", "Off the notch, the peak and the runaway come back"],
        segments: [
          {
            kind: "look",
            text: "Notch off, the gold line has a spike and the rocket's height grows. Notch on, with the bend at 8 rad/s, the spike is gone and the hover settles. Drag the bend away from 8 and the spike returns.",
          },
          {
            kind: "relation",
            tex: String.raw`\frac{\omega_b^2}{s^2+2\zeta\omega_b s+\omega_b^2}`,
            stake: "The bend multiplies the hover plant. $\\omega_b$ is the bend frequency and $\\zeta$ is 0.045, light enough that the spike is tall. The notch is tuned to 8 rad/s. It cancels that spike and leaves a wider, calmer pair of poles. It does not cancel a bend that has moved off 8.",
            gloss: String.raw`The notch zeros use the same light damping, at 8 rad/s.
Its poles use damping 0.55, so the spike is replaced by a low hump.
This is not $\ddot q=\lambda^2 q$ from The loop. That mode grew on its own, with no frequency to point at. This one rings at one frequency.`,
          },
          {
            kind: "trace",
            text: "Notch off, bend at 8 rad/s: the magnitude there is +10 dB, the phase margin is −78°, and the height grows until it hits the stop. Notch on at that same bend: the peak is gone, the phase margin is 29°, and the hover settles at 1.2 m. Move the bend to 5 rad/s with the notch still on and the peak is back near +10 dB, and the hover runs away again.",
          },
        ],
      },
    ],
  },
};

export function readingFor(id: SceneId) {
  return READING[id];
}
