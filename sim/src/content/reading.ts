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
          "Height says where the mass is. Rate says how fast that height is changing. [Those two numbers are enough to know what the mass does next](https://www.fbswiki.org/wiki/index.php/System_Modeling): the same height with an upward rate climbs, and the same height with a downward rate falls.",
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
            stake: "The left side is mass times acceleration. The right side is a force proportional to height, and the minus sign points that force back toward the rest point. With the damper open, the force depends on the height alone.",
            gloss: String.raw`$m$ is the mass.
$k$ is the spring stiffness.
$x$ is height from the rest point.`,
          },
          {
            kind: "contrast",
            tex: String.raw`\ddot{x} = -x`,
            stake: "In this scene the mass and the stiffness are both one, so acceleration equals the height with the sign flipped. Positive height gives negative acceleration. Negative height gives positive acceleration.",
          },
          {
            kind: "trace",
            text: "On the live chart, height and rate trade places: one is largest near the moments when the other crosses zero.",
          },
          {
            kind: "trace",
            text: "Height and rate [trade roles through the cycle](https://en.wikipedia.org/wiki/Simple_harmonic_motion). Together they say what the next moment does.",
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
            tex: String.raw`x = \cos t \qquad v = -\sin t`,
            stake: "The clock chooses both numbers at once. If you know only height, you can miss whether the mass is climbing or falling.",
            gloss: String.raw`$x$ is height from rest.
$v$ is rate in meters per second.
$t$ is the slider time.`,
          },
          {
            kind: "trace",
            text: "A phase plot turns those two numbers into one point. As time moves, that point loops.",
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
      plain(
        "A short promise, then a jump",
        ["A step holds the current height and rate", "A longer step trusts a stale rate for longer"],
        [
          "A computer follows a smooth swing by making a short promise. It takes the height and the speed it has right now, [holds them for a moment, and jumps to a new height](https://en.wikipedia.org/wiki/Euler_method). The true spring keeps curving during that moment.",
          "A short moment stays close to the true swing, because the speed is still the speed of that moment. A long moment keeps a stale speed, and the computed mass walks away from the real one. The step length is the knob that sets how long each promise lasts.",
        ],
      ),
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
            text: "Each update takes the current state as a short promise and keeps that promise until the next sample.",
          },
          {
            kind: "relation",
            tex: String.raw`v_{n+1} = v_n - \Delta t\,x_n`,
            stake: "The new rate is the old rate plus acceleration times the step. For this spring the acceleration is the negative of the height, so the update subtracts $\\Delta t$ times the height it had at the start.",
            gloss: String.raw`$n$ is the current step.
$\Delta t$ is the step length.
$x_n$ is height at the start of the step.
$v_n$ is rate at the start of the step.`,
          },
          {
            kind: "relation",
            tex: String.raw`x_{n+1} = x_n + \Delta t\,v_n`,
            stake: "The new height trusts the old rate for the whole step. A larger $\\Delta t$ makes that trust last longer, and the gold mass drifts from the wire box, which keeps the true spring.",
          },
          {
            kind: "trace",
            text: "The stepped trace leaves the true spring when the held value has left the motion it is approximating.",
          },
          {
            kind: "trace",
            text: "A shorter step makes each guess shorter, so the force law gets more chances to correct the path.",
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
        ["A command can jump", "The vehicle feels the thrust the engine has made"],
        [
          "You can ask an engine for a new thrust in an instant. [The engine reaches that thrust on its own time, closing the gap faster when it is farther behind, and slower when its lag is longer](https://en.wikipedia.org/wiki/Time_constant). It also has a limit, so a request past that limit stays at the limit.",
          "The vehicle feels the thrust the engine has already made. Height and speed follow that thrust, so the climb starts when the engine arrives.",
        ],
      ),
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
            stake: "The command can ask for more than the engine can supply. The vehicle feels the clamped value.",
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
            text: "A slow engine changes when acceleration arrives, so the whole trajectory changes.",
          },
          {
            kind: "look",
            text: "This is the burn's motion chain with an actuator in front of it. The command sits upstream of the state.",
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
          "[One pushes in proportion to the gap right now. One pushes for the gap that has been sitting there, so a small miss that lasts can still grow into a real push. One pushes against the speed, so it can start easing off while the height is still on the way](https://en.wikipedia.org/wiki/PID_controller). The gain knob scales the first of those.",
        ],
      ),
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
            text: "Each term listens to a different part of the error.",
          },
          {
            kind: "relation",
            tex: String.raw`e = x_{\mathrm{ref}} - x`,
            stake: "Error is signed. It gives the distance to the target and the direction the correction needs to push.",
          },
          {
            kind: "contrast",
            tex: String.raw`F_0 + K_p e \qquad F_0 + K_i\!\int e\,dt \qquad F_0 - K_d v`,
            stake: "Proportional reacts to the gap now. Integral reacts to error that has lingered. Derivative pushes against motion before height alone tells the whole story.",
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
          "[Feedback watches the gap to a reference and pushes back](https://en.wikipedia.org/wiki/Feedback). The reference can walk down toward the pad while that push keeps collecting the vehicle. The open-loop knob sets how hard the restless plant runs away on its own.",
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
          },
          {
            kind: "contrast",
            tex: String.raw`\ddot x = \mu^2 x + \frac{K_p}{m}(x_{\mathrm{ref}} - x) - \frac{K_d}{m}\dot x`,
            stake: "Feedback adds a term that watches the gap to the reference, and a rate term that damps the landing. The loop can hold only while Kp/m is larger than μ².",
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
          },
          {
            kind: "contrast",
            tex: String.raw`\ddot x = F/m - g + \epsilon q`,
            stake: "Coupling lets the hidden motion push on height anyway. The unseen state can return through the plant.",
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
          "[If the look itself is old, the decision is about a height the vehicle has already left](https://en.wikipedia.org/wiki/Digital_control). A short sample and a short delay stay close to the always-updating loop. A long one spends more of the flight acting on a stale fact. Drag sets the delay. The sample knob sets how long each hold lasts.",
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
          },
          {
            kind: "contrast",
            tex: String.raw`e_k = x_{\mathrm{ref}}(t) - x(t-\tau)`,
            stake: "The sampled controller can act on old height. A correction sized for the height now meets a vehicle that has already moved.",
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
        "One tilt, a shove and a twist",
        ["A tilted nozzle gives the thrust a sideways part", "That sideways part also makes a moment"],
        [
          "Point the nozzle straight along the body and the thrust pushes along the body. Tilt it, and [part of that push points sideways](https://en.wikipedia.org/wiki/Thrust_vectoring). The sideways part shoves the rocket off to the side.",
          "The nozzle sits below the center of mass, so the same sideways push also [twists the rocket](https://en.wikipedia.org/wiki/Torque). One tilt sets both the shove and the twist, and a larger tilt sets both larger, until the nozzle holds. The gimbal knob is that tilt.",
        ],
      ),
      {
        id: "gimbal",
        title: "One angle, two effects",
        focus: ["A tilted nozzle makes side force", "The same tilt makes a moment about the center of mass"],
        segments: [
          { kind: "look", text: "The gold arrow leaves the nozzle off the centerline. That tilt is the only command." },
          {
            kind: "relation",
            tex: String.raw`F_y = T\sin\delta`,
            stake: "Side force is the thrust times the sine of the gimbal angle. At zero tilt the sine is zero and the shove vanishes. A larger tilt raises the sine, and both traces rise, until the angle holds.",
            gloss: String.raw`$T$ is thrust.
$\delta$ is the gimbal angle.`,
          },
          {
            kind: "contrast",
            tex: String.raw`M = \ell T\sin\delta`,
            stake: "The moment is that same side force times $\\ell$, the distance from the nozzle up to the center of mass. One sine sets the shove and the twist, and the lever arm turns the shove into the twist.",
            gloss: String.raw`$\ell$ is the moment arm.
$M$ is the moment about the center of mass.`,
          },
          {
            kind: "trace",
            text: "The two traces rise together and then hold. One gimbal sets both, so the moment stays with the side force.",
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
        ["Thrust above weight speeds the vehicle upward", "Height is that speed accumulated"],
        [
          "Thrust is a push along the climb. Weight is gravity pulling the vehicle down. [When the push per kilogram is stronger than gravity, the vehicle speeds upward](https://en.wikipedia.org/wiki/Newton%27s_laws_of_motion). When they match, the speed holds. When the push is weaker, the vehicle slows.",
          "The mass stays fixed in this climb, so a steady thrust makes a steady acceleration. Speed then climbs in a straight line, and [height is that speed added up](https://en.wikipedia.org/wiki/Equations_of_motion), which is why the height bends upward: later seconds add a speed the vehicle has already gained. The thrust knob is the push.",
        ],
      ),
      {
        id: "ascent",
        title: "Off the pad",
        focus: ["Thrust per mass minus gravity is the acceleration", "Height is half of acceleration times time squared"],
        segments: [
          { kind: "look", text: "The flame stays on and the rocket leaves the pad. The mass stays at 2 kg, so the thrust knob is the only thing that changes the acceleration." },
          {
            kind: "relation",
            tex: String.raw`a = T/m - g`,
            stake: "$T/m$ is the push per kilogram. Subtracting $g$ takes the weight off that push. What remains is a constant acceleration for as long as the thrust holds.",
            gloss: String.raw`$T$ is thrust.
$m$ is $2\,\mathrm{kg}$.
$g$ is $9.81\,\mathrm{m/s^2}$.`,
          },
          {
            kind: "contrast",
            tex: String.raw`v = at \qquad h = \tfrac{1}{2}at^{2}`,
            stake: "Speed is that acceleration times time, so the rate trace is a straight line whose slope is $a$. Height adds the speed up, which for a constant acceleration is half of $a$ times time squared, and that is the upward bend.",
            gloss: String.raw`$v$ is the climb rate.
$h$ is height.
$t$ is the clock.`,
          },
          {
            kind: "trace",
            text: "Raise the thrust and both the slope of the rate and the bend of the height get steeper. Lower it toward the weight and the climb flattens.",
          },
        ],
      },
    ],
  },
  vehNose: {
    threads: [
      plain(
        "The push follows the nose",
        ["Thrust points along the body", "Tilt splits it into lift and downrange"],
        [
          "A rocket pushes along its nose. Point the nose straight up and the whole push fights weight. Lean the nose and [the push splits: the part along the vertical still fights weight, and the part across the vertical starts the vehicle moving downrange](https://en.wikipedia.org/wiki/Vector_decomposition).",
          "One angle sets both parts. A small lean keeps most of the push in the climb. A steeper lean spends more of it downrange and leaves less to fight weight, so the climb gives some of itself up. The tilt knob is that angle. The thrust stays fixed, and the flame leans with the nose.",
        ],
      ),
      {
        id: "nose",
        title: "Cosine lifts, sine goes downrange",
        focus: ["Thrust is along the body axis", "Tilt splits it into lift and downrange"],
        segments: [
          { kind: "look", text: "The rocket leans, and the flame leans with it. The pad stays behind as the path moves sideways. Thrust is 50 N on the same 2 kg vehicle, and the tilt knob spends that push between the two directions." },
          {
            kind: "relation",
            tex: String.raw`a_y = (T/m)\cos\theta - g`,
            stake: "Cosine is the share of the nose that still points up. At zero tilt it is 1, so the whole push fights gravity. As the tilt grows, cosine shrinks, less acceleration remains in the climb, and gravity takes a larger share of what is left.",
            gloss: String.raw`$T$ is thrust.
$m$ is mass.
$\theta$ is the tilt from vertical.
$g$ is gravity.`,
          },
          {
            kind: "contrast",
            tex: String.raw`a_x = (T/m)\sin\theta`,
            stake: "Sine is the share that points sideways. It is zero when the nose is vertical, and it grows as the nose leans, so downrange acceleration appears as climb acceleration gives way.",
          },
          {
            kind: "trace",
            text: "Height and downrange both grow as the square of time, each with its own acceleration. A steeper tilt raises the downrange trace and softens the height trace.",
          },
        ],
      },
    ],
  },
  vehWind: {
    threads: [
      plain(
        "The angle the air arrives at",
        ["Attack is the angle between the nose and the oncoming air", "The same crosswind is a smaller angle on a faster climb"],
        [
          "[Angle of attack is the angle between where the nose points and where the air is coming from](https://en.wikipedia.org/wiki/Angle_of_attack). Here the nose stays vertical and the wind comes from the side, so the air meets the nose at an angle.",
          "[The climb also makes its own wind, straight down the path](https://en.wikipedia.org/wiki/Relative_wind), and that part grows as the vehicle goes faster. The sideways wind stays the size you set. A faster climb makes the sideways wind a smaller share of the air the nose meets, so the angle shrinks. The wind knob is that sideways speed.",
        ],
      ),
      {
        id: "wind",
        title: "Air across the nose",
        focus: ["Wind is sideways air", "Attack falls as climb speed rises"],
        segments: [
          { kind: "look", text: "The pale arrow is the wind. The nose stays vertical, so the air meets it from the side." },
          {
            kind: "relation",
            tex: String.raw`\alpha = \arctan(W / v)`,
            stake: "The tangent of the attack angle is the sideways wind divided by the climb speed. Arctangent turns that ratio back into an angle. A larger $W$ raises the angle. A larger $v$ lowers it, because the same sideways wind is a smaller share of a faster climb.",
            gloss: String.raw`$W$ is the wind.
$v$ is the climb rate.
$\alpha$ is the angle of attack.`,
          },
          {
            kind: "trace",
            text: "The climb uses 45 N on 2 kg, so the rate trace is a straight rise. The attack trace starts wide, while the climb is slow, and falls as that rate grows. Raise the wind and the whole attack trace lifts.",
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
        "One gain, a vehicle that is changing",
        ["The gain twists the nose toward the command", "Mass falls and the air pushes the nose back"],
        [
          "The autopilot is one gain. It twists the nose toward a commanded angle, harder when the miss is larger. After four tenths of a second the command is twelve degrees, and it stays there.",
          "While that happens the vehicle burns fuel. It gets lighter, so [the same gain is a quicker twist](https://en.wikipedia.org/wiki/Time-variant_system), and it goes faster, so the air pushes harder. [That air push grows with the square of the speed](https://en.wikipedia.org/wiki/Dynamic_pressure) and eases the nose back toward straight up. The gain and the air are both acting on the same angle. The gain knob is how hard the autopilot twists.",
        ],
      ),
      {
        id: "auto",
        title: "The same gain, a changing vehicle",
        focus: ["Gain pitches the nose toward the command", "Mass falls and the air moment grows"],
        segments: [
          { kind: "look", text: "The nose tips toward a 12° command that arrives at 0.4 s. The tank is emptying the whole time, from 3 kg down to 1.4 kg, and the climb uses 40 N." },
          {
            kind: "relation",
            tex: String.raw`m = 3 - 1.6\,t/8`,
            stake: "Mass falls in a straight line over the eight seconds. Inertia is a quarter of the mass, so the vehicle gets easier to twist as it lightens, and the same gain produces a quicker angular acceleration.",
            gloss: String.raw`$m$ is mass in kilograms.
$t$ is the clock in seconds.`,
          },
          {
            kind: "contrast",
            tex: String.raw`I\dot\omega = K(\theta_{\mathrm{cmd}} - \theta) - b\omega - q\theta`,
            stake: "The first term is the gain times the miss, and it twists the nose toward the command. The middle term damps the pitch rate so the nose settles on the way there. The last term is the air, and it opposes the pitch itself.",
            gloss: String.raw`$I$ is inertia.
$K$ is the gain.
$\omega$ is pitch rate.
$b$ damps that rate.
$q$ scales the air.`,
          },
          {
            kind: "relation",
            tex: String.raw`q = c\,v^{2}`,
            stake: "The air coefficient grows with speed squared. Early in the burn the gain sets the pitch. Later, with the vehicle fast and light, the air eases the nose back from the 12° command.",
            gloss: String.raw`$v$ is speed.
$c$ is the coefficient on that speed squared.`,
          },
          {
            kind: "trace",
            text: "Mass falls in a straight line. Pitch rises toward the command and then sags a little as the air term grows. A higher gain makes the rise quicker, and the air is still there at the end.",
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
            stake: "T_k is the thrust profile on this pass. The solved pad-aiming thrust is pulled back so the step stays inside a trust radius Δ_k. An accepted trial grows Δ. A rejected trial shrinks it.",
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
            text: "A starting region near 2–3 N improves the fall toward the pad. A starting region of 8 N takes a huge step on a bad local model, overshoots, and the merit test rejects it. The aside counts accepts and rejects.",
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
        "The fuel runs out on the way",
        ["Thrust stops when the budget is gone", "Speed left at the pad is a miss"],
        [
          "Fuel is a budget, not a force you can keep spending. The landing wants thrust the whole way down. [Each newton of thrust spends a little of that budget](https://en.wikipedia.org/wiki/Tsiolkovsky_rocket_equation).",
          "When the budget hits zero the thrust stops and gravity keeps the fall. Reaching the pad with speed still on the vehicle is a miss. Enough fuel lets the speed die as the height arrives.",
        ],
      ),
      {
        id: "budget",
        title: "Thrust spends the budget",
        focus: ["Fuel falls in proportion to thrust", "An empty budget sets thrust to zero"],
        segments: [
          {
            kind: "look",
            text: "The vehicle starts 45 m up, falling at 3 m/s. The rate trace freezes at the speed it had when the height reached the pad.",
          },
          {
            kind: "relation",
            tex: String.raw`\dot m_f = -T / v_e`,
            stake: "m_f is the fuel still left, in kilograms. T is thrust. v_e is the exhaust speed, 280 m/s. A larger thrust empties the budget faster. At zero fuel the thrust is zero for the rest of the flight.",
            gloss: String.raw`$m_f$ is fuel left.
$T$ is thrust.
$v_e$ is exhaust speed.`,
          },
          {
            kind: "trace",
            text: "A budget of 0.4 kg reaches the pad still moving fast. A budget near 0.7 kg arrives with the speed nearly gone, and what is left in the tank stays unused.",
          },
        ],
      },
    ],
  },
};

export function readingFor(id: SceneId) {
  return READING[id];
}
