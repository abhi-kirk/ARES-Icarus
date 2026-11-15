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
            text: "Start above the rest point. The mass is high, and the acceleration points back down.",
          },
          {
            kind: "look",
            text: "Position is one number. A rocket, a spring, or any moving body also carries a direction of change and a force.",
          },
          {
            kind: "relation",
            tex: String.raw`m\ddot{x} = -kx`,
            stake: "Displacement creates a restoring force. With the damper open, the force depends on displacement alone.",
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
            text: "Height and rate trade roles through the cycle. Together they say what the next moment does.",
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
            gloss: String.raw`$x$ is height from rest. $v$ is rate in meters per second. $t$ is the slider time.`,
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
            stake: "The update uses the height at the start of the step.",
            gloss: String.raw`$n$ is the current step. $\Delta t$ is the step length. $x_n$ and $v_n$ are the state at the start.`,
          },
          {
            kind: "relation",
            tex: String.raw`x_{n+1} = x_n + \Delta t\,v_n`,
            stake: "The height update also trusts the old rate. When $\\Delta t$ grows, that trust lasts longer, and the gold trace drifts away from the truth.",
          },
          {
            kind: "trace",
            text: "The stepped trace goes wrong when the held value has left the motion it is approximating.",
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
            stake: "Thrust is the input. Acceleration is what the vehicle feels. The same $F$ produces less acceleration when $m$ is large.",
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
            gloss: String.raw`$x$ is height from the pad. $v$ is vertical rate.`,
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
            text: "The plan holds while the world matches it. A disturbance, a modeling error, or a changing vehicle turns the planned thrust into the wrong thrust.",
          },
          {
            kind: "relation",
            tex: String.raw`F = F_0 + \Delta F`,
            stake: "After one second, the extra thrust is held for the rest of the run.",
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
            text: "The drift grows because the law holds the extra thrust and leaves it there.",
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
            text: "Each term notices something useful and misses something else.",
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
            text: "Balance and stability are different. A pencil can balance on its tip, and a small mistake sends it off.",
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
            text: "The hidden-mode trace is for you, so the missing state has somewhere to show itself.",
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
            text: "Digital control puts gates in time: measure, compute, hold, and measure again.",
          },
          {
            kind: "relation",
            tex: String.raw`F = F_0 + K_p e + K_i\!\int e\,dt`,
            stake: "Continuous feedback updates as the error changes.",
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
  turnFrames: {
    threads: [
      {
        id: "frames",
        title: "One arrow, two descriptions",
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
      {
        id: "gimbal",
        title: "One angle, two effects",
        focus: ["A tilted nozzle makes side force", "The same tilt makes a moment about the center of mass"],
        segments: [
          { kind: "look", text: "The gold arrow leaves the nozzle off the centerline. That tilt is the only command." },
          {
            kind: "relation",
            tex: String.raw`F_y = T\sin\delta`,
            stake: "Side force is the thrust across the body.",
          },
          {
            kind: "contrast",
            tex: String.raw`M = \ell T\sin\delta`,
            stake: "The moment is that same side force times the distance from the nozzle to the center of mass. One sine sets both.",
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
      {
        id: "pressure",
        title: "Where the air pushes",
        focus: ["Aerodynamic force acts at the center of pressure", "The moment restores when that point is behind the mass"],
        segments: [
          { kind: "look", text: "The gold mark is the center of mass. The pale blue mark is the center of pressure. The rocket's lean is the angle of attack." },
          {
            kind: "relation",
            tex: String.raw`M = -q S d\, C_{N\alpha}\, \alpha\, \sigma`,
            stake: "Margin $\\sigma$ positive means the center of pressure is behind the center of mass, so the moment opposes the angle.",
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
      {
        id: "slosh",
        title: "Fuel that is still free to move",
        focus: ["The tank drains from the bottom", "The fuel that remains can swing and shift the center of mass"],
        segments: [
          { kind: "look", text: "The gold slug is the fuel still in the tank. It swings, and it rides higher as the tank empties." },
          {
            kind: "relation",
            tex: String.raw`x_{\mathrm{cm}} = \frac{m_{\mathrm{fuel}}}{m}\, x_{\mathrm{slug}}`,
            stake: "The vehicle center of mass moves sideways only by the fuel's share of the mass.",
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
      {
        id: "ascent",
        title: "Off the pad",
        focus: ["Thrust above weight makes an upward acceleration", "Height is accumulated rate"],
        segments: [
          { kind: "look", text: "The flame stays on and the rocket leaves the pad. This climb is the vehicle the later scenes ride on." },
          {
            kind: "relation",
            tex: String.raw`a = T/m - g`,
            stake: "Acceleration is thrust per mass, with gravity taken off. The mass stays fixed in this scene.",
            gloss: String.raw`$T$ is thrust. $m$ is $2\,\mathrm{kg}$. $g$ is $9.81\,\mathrm{m/s^2}$.`,
          },
          {
            kind: "trace",
            text: "Rate climbs in a straight line. Height bends upward because it keeps the rate it has already gained.",
          },
        ],
      },
    ],
  },
  vehNose: {
    threads: [
      {
        id: "nose",
        title: "The push follows the nose",
        focus: ["Thrust is along the body axis", "Tilt splits it into lift and downrange"],
        segments: [
          { kind: "look", text: "The rocket leans, and the flame leans with it. The pad stays behind as the path moves sideways." },
          {
            kind: "relation",
            tex: String.raw`a_y = (T/m)\cos\theta - g`,
            stake: "The vertical part is the thrust along the nose, times the cosine of the tilt, with gravity taken off.",
          },
          {
            kind: "contrast",
            tex: String.raw`a_x = (T/m)\sin\theta`,
            stake: "The sideways part is the same thrust times the sine of the tilt. One angle sets both.",
          },
          {
            kind: "trace",
            text: "Height and downrange both grow. A steeper tilt trades climb for downrange.",
          },
        ],
      },
    ],
  },
  vehWind: {
    threads: [
      {
        id: "wind",
        title: "Air across the nose",
        focus: ["Wind is sideways air", "Attack falls as climb speed rises"],
        segments: [
          { kind: "look", text: "The pale arrow is the wind. The nose stays vertical, so the air meets it from the side." },
          {
            kind: "relation",
            tex: String.raw`\alpha = \arctan(W / v)`,
            stake: "Attack is the angle whose tangent is wind speed over climb speed. A faster climb makes that angle smaller.",
            gloss: String.raw`$W$ is the wind. $v$ is the climb rate.`,
          },
          {
            kind: "trace",
            text: "The rate trace climbs. The attack trace falls, because the same wind is a smaller share of a faster vehicle.",
          },
        ],
      },
    ],
  },
  vehEstimate: {
    threads: [
      {
        id: "estimate",
        title: "Predict, then correct",
        focus: ["The predictor uses the thrust model", "The corrector pulls toward the sensor"],
        segments: [
          { kind: "look", text: "The rocket is the true height. The pale mark is the estimate, and it rides beside the body." },
          {
            kind: "relation",
            tex: String.raw`\hat{x}^{-} = \hat{x} + \hat{v}\,\Delta t + \tfrac{1}{2}a\Delta t^{2}`,
            stake: "The prediction carries the last estimate forward with the thrust model.",
          },
          {
            kind: "contrast",
            tex: String.raw`\hat{x} = \hat{x}^{-} + K(z - \hat{x}^{-})`,
            stake: "The correction adds a fraction of the gap between the sensor and that prediction. $z$ is the sensor.",
          },
          {
            kind: "trace",
            text: "The truth is smooth. The estimate stays near it. A larger gain follows the sensor's wave more closely.",
          },
        ],
      },
    ],
  },
  vehGuide: {
    threads: [
      {
        id: "guide",
        title: "A direction to hold",
        focus: ["Guidance commands a pitch", "The body follows that command"],
        segments: [
          { kind: "look", text: "The pale arrow is the direction guidance wants. The nose starts vertical, then tips toward that arrow." },
          {
            kind: "relation",
            tex: String.raw`\dot\theta = (\theta_{\mathrm{cmd}} - \theta)/\tau`,
            stake: "The nose closes the gap to the command at a rate set by that gap. $\\tau$ is $0.7\\,\\mathrm{s}$.",
          },
          {
            kind: "trace",
            text: "The command stays at zero, then steps into the kickover. The pitch trace follows it, and the path bends with the nose.",
          },
        ],
      },
    ],
  },
  vehAuto: {
    threads: [
      {
        id: "auto",
        title: "The same gain, a changing vehicle",
        focus: ["Gain pitches the nose toward the command", "Mass falls and the air moment grows"],
        segments: [
          { kind: "look", text: "The nose tips toward a fixed command. The tank is emptying while it does." },
          {
            kind: "relation",
            tex: String.raw`I\dot\omega = K(\theta_{\mathrm{cmd}} - \theta) - b\omega - qc\,\theta`,
            stake: "The gain $K$ pulls toward the command. The middle term damps the rate. The last term grows with speed squared and eases the nose back.",
          },
          {
            kind: "trace",
            text: "Mass falls in a straight line. Pitch rises to the command, then the growing air eases it back a little.",
          },
        ],
      },
    ],
  },
};

export function readingFor(id: SceneId) {
  return READING[id];
}
