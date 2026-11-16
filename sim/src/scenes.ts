// Rocket Dynamics. One clock per scene. Each scene has a claim, a knob you can drag, and chart columns.

import {
  loopRowAt,
  loopRowsFor,
  LOOP_HIDDEN_LAMBDA,
  LOOP_HOVER_KD,
  LOOP_HOVER_KP,
  LOOP_LIMIT,
  LOOP_M,
  LOOP_SAMPLE_DELAY_MARK,
  LOOP_T0,
} from "./loopPlant";
import { turnAt, turnBrief, turnDuration, turnRows, type TurnId, type TurnSample } from "./turnPlant";
import {
  ascentTw,
  ascentWeight,
  noseBreakEvenDeg,
  noseClimbAccel,
  noseSideAccel,
  vehicleAt,
  vehicleBrief,
  vehicleDuration,
  vehicleRows,
  type VehicleId,
  type VehicleSample,
} from "./vehiclePlant";
import {
  costGains,
  descentAt,
  descentBrief,
  descentDuration,
  descentRows,
  gustoInfo,
  missOutcome,
  MISS_HARD_CEILING,
  scvxPassInfo,
  TOUCH_SPEED,
  type DescentId,
  type DescentSample,
} from "./descentPlant";


export type SceneId =
  | "state"
  | "step"
  | "mass"
  | "engine"
  | "loopOpen"
  | "loopError"
  | "loopEquil"
  | "loopHover"
  | "loopMode"
  | "loopSample"
  | "turnFrames"
  | "turnLock"
  | "turnQuat"
  | "turnSpin"
  | "turnInertia"
  | "turnGimbal"
  | "turnAir"
  | "turnSlosh"
  | "vehAscent"
  | "vehNose"
  | "vehWind"
  | "vehEstimate"
  | "vehGuide"
  | "vehAuto"
  | "desCost"
  | "desPath"
  | "desLoss"
  | "desScvx"
  | "desGusto"
  | "desReplan"
  | "desMiss";

export type Column =
  | "height"
  | "rate"
  | "stepped"
  | "truth"
  | "accel"
  | "mass"
  | "command"
  | "actual"
  | "reference"
  | "pHeight"
  | "iHeight"
  | "dHeight"
  | "nonlinear"
  | "linear"
  | "openHeight"
  | "closedHeight"
  | "measured"
  | "hiddenMode"
  | "continuous"
  | "discrete"
  | "thrust"
  | "worldX"
  | "worldZ"
  | "pitch"
  | "axisGap"
  | "quatW"
  | "quatZ"
  | "angle"
  | "spin"
  | "side"
  | "moment"
  | "attack"
  | "fill"
  | "offset"
  | "downrange"
  | "estimate"
  | "aim";

export type Row = {
  t: number;
  height: number;
  rate: number;
  stepped: number;
  truth: number;
  accel: number;
  mass: number;
  command: number;
  actual: number;
  reference: number;
  pHeight: number;
  iHeight: number;
  dHeight: number;
  nonlinear: number;
  linear: number;
  openHeight: number;
  closedHeight: number;
  measured: number;
  hiddenMode: number;
  continuous: number;
  discrete: number;
  thrust: number;
  worldX: number;
  worldZ: number;
  pitch: number;
  axisGap: number;
  quatW: number;
  quatZ: number;
  angle: number;
  spin: number;
  side: number;
  moment: number;
  attack: number;
  fill: number;
  offset: number;
  downrange: number;
  estimate: number;
  aim: number;
};

export type Knobs = {
  dt: number;
  thrust: number;
  tau: number;
  ask: number;
  kp: number;
  ki: number;
  kd: number;
  perturb: number;
  unstable: number;
  coupling: number;
  extraThrust: number;
  yaw: number;
  pitch: number;
  tumble: number;
  rate: number;
  inertia: number;
  gimbal: number;
  margin: number;
  slosh: number;
  lift: number;
  lean: number;
  wind: number;
  trust: number;
  kick: number;
  hold: number;
  care: number;
  spend: number;
  cone: number;
  floor: number;
  pass: number;
  region: number;
  gust: number;
  fuel: number;
  noise: number;
  schedule: number;
  twist: number;
  // Step scene: 0 = explicit Euler, 1 = semi-implicit Euler.
  method: number;
  // Spring scene: initial height and rate at t = 0.
  x0: number;
  v0: number;
  // Engine scene: 0 = step at need time, 1 = ask one τ early (lead the lag).
  lead: number;
  // Miss scene: 0 = fuel budget miss, 1 = low-ceiling INFEASIBLE.
  cause: number;
};

export const KNOBS: Knobs = {
  dt: 0.4,
  thrust: 40,
  tau: 0.35,
  ask: 90,
  extraThrust: 15,
  kp: 12,
  ki: 8,
  kd: 15,
  perturb: 0.35,
  unstable: 1.5,
  coupling: 0.35,
  yaw: 90,
  pitch: 90,
  tumble: 180,
  rate: 45,
  inertia: 1,
  gimbal: 8,
  margin: 0.6,
  slosh: 16,
  lift: 40,
  lean: 15,
  wind: 8,
  trust: 2,
  kick: 90,
  hold: 6,
  care: 1,
  spend: 1,
  cone: 15,
  floor: 12,
  pass: 3,
  region: 3,
  gust: 6,
  fuel: 0.4,
  noise: 12,
  schedule: 1,
  twist: 10,
  method: 0,
  x0: 1,
  v0: 0,
  lead: 0,
  cause: 0,
};

// Undamped spring. Mass 1 kg, stiffness 1 N/m, so one swing takes 2π seconds.
// Default release from rest 1 m above rest: height = cos(t), rate = -sin(t).
// General start (x0, v0): height = x0 cos(t) + v0 sin(t).
const SPRING_M = 1;
const SPRING_K = 1;

const G = 9.81;
const DRY = 1.5;
const FUEL = 1.5;
const BURN = 4;
const LIMIT = 50;
const ASK_AT = 0.4;

export type KnobKey =
  | "dt"
  | "thrust"
  | "tau"
  | "ask"
  | "kp"
  | "ki"
  | "kd"
  | "perturb"
  | "unstable"
  | "coupling"
  | "extraThrust"
  | "yaw"
  | "pitch"
  | "tumble"
  | "rate"
  | "inertia"
  | "gimbal"
  | "margin"
  | "slosh"
  | "lift"
  | "lean"
  | "wind"
  | "trust"
  | "kick"
  | "hold"
  | "care"
  | "spend"
  | "cone"
  | "floor"
  | "pass"
  | "region"
  | "gust"
  | "fuel"
  | "noise"
  | "schedule"
  | "twist"
  | "method"
  | "x0"
  | "v0"
  | "lead"
  | "cause";

const LIMITS: Record<KnobKey, { min: number; max: number }> = {
  dt: { min: 0.05, max: 0.5 },
  thrust: { min: 20, max: 80 },
  tau: { min: 0.05, max: 0.8 },
  ask: { min: 0, max: 120 },
  extraThrust: { min: 0, max: 35 },
  kp: { min: 4, max: 24 },
  ki: { min: 2, max: 16 },
  kd: { min: 4, max: 28 },
  perturb: { min: 0.08, max: 0.8 },
  // μ in the lesson ẍ = μ² x. Keep μ² under Kp/m so closed loop can land.
  unstable: { min: 0.8, max: 2.5 },
  coupling: { min: 0.1, max: 0.7 },
  yaw: { min: 30, max: 180 },
  pitch: { min: 30, max: 90 },
  tumble: { min: 90, max: 180 },
  rate: { min: 15, max: 120 },
  inertia: { min: 0.4, max: 4 },
  // turnGimbal: signed side-force command (N). Other scenes do not use this range.
  gimbal: { min: -25, max: 25 },
  twist: { min: -45, max: 45 },
  margin: { min: -1, max: 1.5 },
  slosh: { min: 4, max: 35 },
  // Below weight (~19.6 N on 2 kg) the ascent stays on the pad; keep that case reachable.
  lift: { min: 10, max: 70 },
  // Past θ* ≈ 66.9° vertical accel is ≤ 0 for T=50 N on 2 kg; keep that case on the slider.
  lean: { min: 0, max: 75 },
  wind: { min: 0, max: 25 },
  trust: { min: 0, max: 6 },
  noise: { min: 0, max: 20 },
  // Target downrange for closed-loop guidance (meters).
  kick: { min: 40, max: 160 },
  hold: { min: 2, max: 14 },
  schedule: { min: 0, max: 1 },
  care: { min: 0.05, max: 4 },
  spend: { min: 0.25, max: 4 },
  // Above ~35° the wall is wider than the path needs, so further opening does nothing.
  cone: { min: 8, max: 35 },
  // Below ~12 N the guidance command already sits above the floor, so those settings match.
  floor: { min: 12, max: 28 },
  pass: { min: 1, max: 5 },
  // Above ~6 N the first GuSTO trial tends to reject the same way; keep the interesting band.
  region: { min: 1, max: 6 },
  gust: { min: 0, max: 14 },
  // Near 0.7 kg arrival speed is already nearly gone; more fuel only sits unused.
  fuel: { min: 0.2, max: 0.75 },
  method: { min: 0, max: 1 },
  x0: { min: -1.5, max: 1.5 },
  v0: { min: -1.5, max: 1.5 },
  lead: { min: 0, max: 1 },
  cause: { min: 0, max: 1 },
};

export function clampNamed(key: KnobKey, value: number): number {
  const limit = LIMITS[key];
  return clamp(value, limit.min, limit.max);
}

export const SCENES: {
  id: SceneId;
  name: string;
  title: string;
  lede: string;
  opener: string;
  live: Column[];
  askable: Column[];
  chartTitle: string;
  visual: string;
}[] = [
  {
    id: "state",
    name: "The spring",
    title: "The spring",
    lede: "A mass hangs from a spring with the damper open. Drag sets the start height; the Rate knob sets the start rate. Same height with two rates leaves two ways.",
    opener: "Height and rate together are the state. Drag the mass to set start height, or set the Rate knob. Ask me about a symbol, or tell me a time.",
    live: ["height", "rate"],
    askable: ["height", "rate", "accel"],
    chartTitle: "Rate against height",
    visual: "Drag the mass to set start height x0. The Rate knob sets start rate v0. The phase loop is that pair, with a gold dot at the current state.",
  },
  {
    id: "step",
    name: "The step",
    title: "The step",
    lede: "Explicit Euler holds the old height and rate for a whole Δt, then jumps. That freezes a stale force, so mechanical energy grows and the gold orbit spirals out. Semi-implicit updates the rate first, then the height, and stays closer to the wire box.",
    opener: "The gold mass is one step at a time. The wire box is the true height. Ask me what a symbol means, or set a step.",
    live: ["stepped", "truth"],
    askable: ["stepped", "truth", "height", "rate"],
    chartTitle: "Phase orbit: true and stepped",
    visual: "Phase: gold is the numerical orbit, pale is the true ellipse. Explicit Euler spirals out as energy grows. Semi-implicit stays nearer the wire box.",
  },
  {
    id: "mass",
    name: "The burn",
    title: "The burn",
    lede: "Thrust stays on while the tank empties, and at four seconds the tank is empty so the thrust stops. Height and rate come from integrating acceleration. Drag the rocket to change the thrust input.",
    opener: "Height and rate are the state. Thrust is an input until the tank is empty. Ask me what a symbol means, or set a thrust.",
    live: ["accel", "rate"],
    askable: ["height", "rate", "accel", "mass", "actual"],
    chartTitle: "Acceleration and rate",
    visual: "The rocket rises or falls from the integrated motion. The live chart shows acceleration and rate so the burn story is visible before height finishes changing.",
  },
  {
    id: "engine",
    name: "The engine",
    title: "The engine",
    lede: "The pale line is the command; the gold line is the thrust the engine makes, capped at 50 N. Lead asks one τ early so actual thrust is closer when you need it. Drag the rocket to change the lag.",
    opener: "The command is an input. Lead asks early so the lagging engine is nearer the need. Height and rate follow actual thrust on a 2.5 kg vehicle.",
    live: ["actual", "command"],
    askable: ["actual", "command", "height", "rate", "accel"],
    chartTitle: "Command and actual",
    visual: "Pale command vs gold actual thrust (ceiling 50 N). Lead mode steps the ask one lag-time early so F is farther along by the need mark.",
  },
  {
    id: "loopOpen",
    name: "Open loop",
    title: "Open loop",
    lede: "The planner picks a thrust schedule and the rocket follows it. The command stays on that schedule, so a fixed extra thrust after one second lets the rocket drift above the target.",
    opener: "Reference height is the target. Height is what the fixed thrust produces. Drag the rocket to change the extra thrust.",
    live: ["reference", "height"],
    askable: ["reference", "height", "rate", "thrust"],
    chartTitle: "Reference and height",
    visual: "The reference is the desired height. Thrust follows the schedule alone, so a small extra push keeps accumulating motion.",
  },
  {
    id: "loopError",
    name: "Three reactions to an error",
    title: "Three reactions to an error",
    lede: "The target steps at two seconds. P, I, and D each fly alone; PID flies all three together. I alone can run far, so it sits on its own axis. Kp, Ki, and Kd each have a slider.",
    opener: "P, D, and PID share the left axis. I is on the right. Drag the rocket to change Kp.",
    live: ["pHeight", "dHeight", "closedHeight", "iHeight"],
    askable: ["reference", "pHeight", "iHeight", "dHeight", "closedHeight", "height", "rate", "thrust"],
    chartTitle: "P, D, PID, and I",
    visual: "Closed loop is the combined PID. I alone is on the right axis so P and D stay readable.",
  },
  {
    id: "loopEquil",
    name: "Equilibrium and linearization",
    title: "Equilibrium and linearization",
    lede: "Near hover, the same closed-loop law is shown twice: the full thrust-height relation and the linearized one. For a small starting offset they stay on top of each other.",
    opener: "Nonlinear is the real plant. Linear is the small-signal model. Drag the rocket to change the starting offset.",
    live: ["nonlinear", "linear"],
    askable: ["nonlinear", "linear", "height", "rate", "thrust"],
    chartTitle: "Nonlinear and linear",
    visual: "The nonlinear trace uses the full thrust law. The linear trace uses the local small-signal model near hover.",
  },
  {
    id: "loopHover",
    name: "Hover that is unstable and still lands",
    title: "Hover that is unstable and still lands",
    lede: "Open loop, a wrong-sign term makes height run away from equilibrium. With feedback on height, the same plant is driven to a landing profile and settles on the pad.",
    opener: "Open loop diverges. Closed loop lands. Drag the rocket to change how unstable the open loop is.",
    live: ["openHeight", "closedHeight"],
    askable: ["reference", "openHeight", "closedHeight", "height", "rate", "thrust"],
    chartTitle: "Open loop and closed loop",
    visual: "The open-loop trace runs away from hover. The closed-loop trace follows a landing reference and settles on the pad.",
  },
  {
    id: "loopMode",
    name: "A mode the loop misses",
    title: "A mode the loop misses",
    lede: "Height is what you measure. A hidden flexible mode grows on its own while feedback acts on height, and the chart draws both.",
    opener: "Measured height is what the loop sees. Hidden mode is internal. Drag the rocket to change how strongly the mode pushes on motion.",
    live: ["measured", "hiddenMode"],
    askable: ["reference", "measured", "hiddenMode", "height", "rate", "thrust"],
    chartTitle: "Measured and hidden mode",
    visual: "Measured height is what the controller sees. The hidden mode is internal motion that can grow while the sensor looks calm.",
  },
  {
    id: "loopSample",
    name: "Sample time and delay",
    title: "Sample time and delay",
    lede: "Continuous feedback updates every instant. Discrete feedback holds thrust between samples, and delay uses an old height reading. The same gains feel different on the chart.",
    opener: "Continuous is the ideal loop. Discrete uses sample time and delay. Drag the rocket to change the lag on the delayed height.",
    live: ["continuous", "discrete"],
    askable: ["reference", "continuous", "discrete", "height", "rate", "command", "actual"],
    chartTitle: "Continuous and discrete",
    visual: "The continuous trace updates every instant. The discrete trace holds thrust between samples and may act on delayed height.",
  },
  {
    id: "turnFrames",
    name: "Frames",
    title: "Frames",
    lede: "The gold arrow is fixed to the rocket. The pale arrow is the same direction drawn in the world, and its two components change as the rocket yaws.",
    opener: "The body arrow stays fixed on the rocket. worldX and worldZ are that arrow in the world. Drag the rocket to change how far it yaws.",
    live: ["worldX", "worldZ"],
    askable: ["worldX", "worldZ"],
    chartTitle: "World components",
    visual: "The gold arrow stays on the body. The pale arrow stays on the world axis. Yaw turns one relative to the other.",
  },
  {
    id: "turnLock",
    name: "Gimbal lock",
    title: "Gimbal lock",
    lede: "Pitch tips the nose up from the horizon. At 90 degrees the nose lies on the vertical yaw axis, so a roll about the nose is the same turn as a yaw.",
    opener: "Pitch is how far the nose has risen. The gap is the angle left between the nose and the vertical axis. Drag the rocket to change the final pitch.",
    live: ["pitch", "axisGap"],
    askable: ["pitch", "axisGap"],
    chartTitle: "Pitch and axis gap",
    visual: "The pale arrow is the vertical yaw axis. The gold arrow is the nose. They meet when pitch reaches 90 degrees.",
  },
  {
    id: "turnQuat",
    name: "Quaternions",
    title: "Quaternions",
    lede: "The same pitch, carried past vertical, stays a smooth pair of numbers. The cosine and sine of half the angle keep their roles and stay finite when the nose crosses the yaw axis.",
    opener: "quatW and quatZ are one rotation about the pitch axis. Drag the rocket to change how far past vertical the nose goes.",
    live: ["quatW", "quatZ"],
    askable: ["quatW", "quatZ", "pitch"],
    chartTitle: "Quaternion components",
    visual: "The nose keeps a direction through and past the vertical. The chart holds the four-number rotation as two components of a unit quaternion.",
  },
  {
    id: "turnSpin",
    name: "Spin",
    title: "Spin",
    lede: "Torque is zero, so the spin rate stays put and the angle is that rate times time. The stripe on the body walks around at a steady pace.",
    opener: "Spin is the rate. Angle is how far that rate has carried the body. Drag the rocket to change the rate.",
    live: ["angle", "spin"],
    askable: ["angle", "spin"],
    chartTitle: "Angle and spin",
    visual: "The rocket yaws at a constant rate. The angle on the chart is a straight ramp.",
  },
  {
    id: "turnInertia",
    name: "Inertia and torque",
    title: "Inertia and torque",
    lede: "A steady torque makes the spin rate climb in a straight line. More inertia, same torque, and the line is shallower.",
    opener: "The torque is fixed at 2 N·m. Drag the rocket to change the inertia.",
    live: ["angle", "spin"],
    askable: ["angle", "spin", "moment"],
    chartTitle: "Angle and spin",
    visual: "The rocket speeds up as it turns. A larger inertia reaches the same angle later.",
  },
  {
    id: "turnGimbal",
    name: "Gimbal and allocation",
    title: "Gimbal and allocation",
    lede: "Two engines share side-force and moment commands through an allocation matrix. Each gimbal is signed and clipped to ±15°. When the solve hits a limit, the delivered force and moment fall short of the ask.",
    opener: "Side force and moment are what the allocated gimbals deliver. Drag the rocket to change the side-force command.",
    live: ["side", "moment"],
    askable: ["side", "moment", "pitch", "angle"],
    chartTitle: "Side force and moment",
    visual: "Pitch is δ₁. Angle is δ₂. The gold arrow uses the mean tilt. Actuators stop at ±15°.",
  },
  {
    id: "turnAir",
    name: "Air and center of pressure",
    title: "Air and center of pressure",
    lede: "Air pushes at the center of pressure. When that point sits behind the center of mass, the moment turns the nose back toward the wind.",
    opener: "Positive margin means the center of pressure is behind the center of mass. Drag the rocket to move that margin.",
    live: ["attack", "moment"],
    askable: ["attack", "moment"],
    chartTitle: "Angle and moment",
    visual: "The gold mark is the center of mass. The pale blue mark is the center of pressure. The rocket's tilt is the angle of attack.",
  },
  {
    id: "turnSlosh",
    name: "Tank that drains and sloshes",
    title: "Tank that drains and sloshes",
    lede: "The tank empties from the bottom while the fuel still in it swings. That swing shifts the center of mass sideways, and the shift shrinks as the fuel runs out.",
    opener: "Fill is the fuel still aboard. Offset is how far the swinging fuel has moved the center of mass. Drag the rocket to change the swing.",
    live: ["fill", "offset"],
    askable: ["fill", "offset"],
    chartTitle: "Fuel and offset",
    visual: "The gold slug is the fuel that is still free to swing. It rides up the tank as the propellant burns, and its sideways travel moves the center of mass.",
  },
  {
    id: "vehAscent",
    name: "Ascent",
    title: "Ascent",
    lede: "Mass is 2 kg, so weight is 19.6 N. Thrust above that weight lifts off; at or below it the vehicle stays on the pad. T/W is thrust over weight.",
    opener: "Height and rate come from one thrust on a fixed mass. Watch T/W. Drag the rocket to change the thrust.",
    live: ["height", "rate"],
    askable: ["height", "rate", "accel", "thrust"],
    chartTitle: "Height and rate",
    visual: "The flame is the thrust. T/W above 1 climbs; T/W at or below 1 stays on the pad.",
  },
  {
    id: "vehNose",
    name: "Thrust along the nose",
    title: "Thrust along the nose",
    lede: "Thrust points along the nose. Cosine fights weight; sine goes downrange. At about 66.9° the climb accel hits zero, and the vehicle stays on the pad.",
    opener: "Gold is thrust along the nose; pale arrows are the vertical and side shares. Drag the rocket to change the tilt.",
    live: ["height", "downrange"],
    askable: ["height", "downrange", "rate", "pitch"],
    chartTitle: "Height and downrange",
    visual: "Thrust along the nose splits into climb and downrange. Past the break-even tilt the pad holds.",
  },
  {
    id: "vehWind",
    name: "Wind",
    title: "Wind",
    lede: "The nose stays vertical. Attack is arctan(W/v): 90° at liftoff, then falling as climb speed grows. A teaching side load also drifts the vehicle downrange.",
    opener: "Attack is the air angle on the nose. Drift is the teaching side load. Drag the rocket to change the wind.",
    live: ["attack", "downrange"],
    askable: ["rate", "attack", "height", "downrange"],
    chartTitle: "Attack and downrange",
    visual: "Pale wind arrow. Attack starts near 90° and falls. Downrange grows from the side load.",
  },
  {
    id: "vehEstimate",
    name: "Sensors and an estimate",
    title: "Sensors and an estimate",
    lede: "A fixed-gain filter predicts height from thrust, then pulls toward the sensor. The chart shows sensor error and estimate error from truth. High gain follows the wave. Low gain trusts the model.",
    opener: "The rocket is truth. The pale mark is the estimate. Drag the rocket to change the gain.",
    live: ["measured", "estimate"],
    askable: ["height", "estimate", "measured", "rate"],
    chartTitle: "Sensor error and estimate error",
    visual: "Chart values are errors from truth. Raise Noise to grow the wave. Gain 0 trusts only the model.",
  },
  {
    id: "vehGuide",
    name: "Guidance",
    title: "Guidance",
    lede: "Closed-loop guidance aims the nose at a ground target from a filtered estimate of downrange and height. The body follows that command, and the path bends as the nose tips.",
    opener: "Downrange and height are the flown path. Drag the rocket to change the target downrange.",
    live: ["downrange", "height"],
    askable: ["downrange", "height", "aim", "pitch", "estimate"],
    chartTitle: "Downrange and height",
    visual: "The pale arrow is the live pitch command toward the target. Estimate is the filtered downrange used for aiming.",
  },
  {
    id: "vehAuto",
    name: "Autopilot as mass and air change",
    title: "Autopilot as mass and air change",
    lede: "The autopilot pitches toward a command while mass falls and air stiffness grows with speed squared. Schedule scales the gain with mass so the twist stays matched to inertia. Fixed holds one K the whole burn.",
    opener: "Pitch is the nose. Gain used is the live K. Drag the rocket to change the base gain.",
    live: ["pitch", "estimate"],
    askable: ["pitch", "estimate", "mass", "rate", "measured", "aim", "height"],
    chartTitle: "Pitch and gain used",
    visual: "Estimate is the live gain K. Measured is air stiffness κ = c v². Rate is speed.",
  },
  {
    id: "desCost",
    name: "A cost that makes a gain",
    title: "A cost that makes a gain",
    lede: "The weight on height chooses the gain. A heavier weight checks the fall sooner and peaks the thrust later and higher. Thrust floors at zero. The cost has no ground in it, so the path can pass through the pad.",
    opener: "Height is the state. Thrust is what the gain asks for, floored at zero. Drag the rocket to change the weight on height.",
    live: ["height", "thrust"],
    askable: ["height", "thrust", "rate", "command"],
    chartTitle: "Height and thrust",
    visual: "The rocket falls toward the pad. A heavier weight on height brakes sooner. Thrust can sit at zero until the demand rises.",
  },
  {
    id: "desPath",
    name: "A path under constraints",
    title: "A path under constraints",
    lede: "Thrust stays between 6 N and 55 N, and the vehicle has to remain inside a cone that opens from the pad. A tight cone puts the path on the wall.",
    opener: "Downrange is the path. The pale wall is the cone limit. Drag the rocket to change the cone.",
    live: ["downrange", "reference"],
    askable: ["height", "downrange", "reference", "thrust", "rate"],
    chartTitle: "Downrange and cone wall",
    visual: "Downrange is the vehicle. The pale wall is |x| = z tan θ. Outside that wall is a cone violation.",
  },
  {
    id: "desLoss",
    name: "Lossless convexification",
    title: "Lossless convexification",
    lede: "Allowed thrust is a ring, with a hole of pushes that are too small. The landing charges for a slack at least as large as the inner rim, and the thrust you see stays on that charge.",
    opener: "Thrust is the push that flies. Command is the push the guidance asked for before the floor. Drag the rocket to raise the floor.",
    live: ["thrust", "command"],
    askable: ["thrust", "command", "reference", "height", "rate", "downrange"],
    chartTitle: "Thrust and command",
    visual: "Thrust is flown (|T| = σ). Command is the uncapped ask. The ceiling is 55 N.",
  },
  {
    id: "desScvx",
    name: "Successive convexification",
    title: "Successive convexification",
    lede: "Each pass flies the current thrust, solves a pad-aiming subproblem, limits the step with a trust region, and may add virtual slack. Early passes miss the pad. Later ones close in.",
    opener: "Height is this pass. The dashed line is the pass before it. Drag the rocket to change the pass.",
    live: ["height", "reference"],
    askable: ["height", "reference", "downrange", "thrust", "rate"],
    chartTitle: "This pass and the one before",
    visual: "The rocket flies the pass you picked. Early passes fall through the pad. The aside shows end error, trust radius, and whether the next step was accepted.",
  },
  {
    id: "desGusto",
    name: "GuSTO",
    title: "GuSTO",
    lede: "GuSTO iterates with a hard trust region and a merit test. A modest Δ improves the fall. A huge Δ trusts a bad local model and can overshoot or be rejected.",
    opener: "Height is after the GuSTO iterations. The dashed line is the weak first push. Drag the rocket to change the starting trust region.",
    live: ["height", "reference"],
    askable: ["height", "reference", "downrange", "thrust"],
    chartTitle: "Corrected path and the first push",
    visual: "The rocket flies the accepted iterates. The aside shows end error, final Δ, and accept/reject counts.",
  },
  {
    id: "desReplan",
    name: "Replanning",
    title: "Replanning",
    lede: "A gust changes the sideways speed. Guidance recomputes from the state you have now (MPC-style), inside a thrust ceiling and a glide cone. A modest gust recovers. A large one saturates the limits and misses.",
    opener: "Downrange is the live plan. The dashed line is the plan from before the gust. Drag the rocket to change the gust.",
    live: ["downrange", "reference"],
    askable: ["downrange", "reference", "height", "rate", "thrust"],
    chartTitle: "New plan and the old one",
    visual: "Gust at 2.2 s. Thrust ≤ 40 N, cone 12° from vertical. Large gusts can miss the pad.",
  },
  {
    id: "desMiss",
    name: "Runs that do not land",
    title: "Runs that do not land",
    lede: "A soft landing needs |rate| ≤ 0.5 m/s at the pad. Fuel-out or a ceiling too low both miss; Cause switches those two failure modes.",
    opener: "Status is soft, miss, or INFEASIBLE. Drag the rocket to change the fuel, or flip Cause to a hard ceiling.",
    live: ["height", "rate"],
    askable: ["height", "rate", "thrust"],
    chartTitle: "Height and rate",
    visual: "Fuel-out leaves speed at the pad. A low ceiling is INFEASIBLE: no soft landing exists under that thrust bound.",
  },
];

export function sceneById(id: SceneId) {
  const found = SCENES.find((scene) => scene.id === id);
  if (!found) return SCENES[0];
  return found;
}

export function durationOf(id: SceneId): number {
  if (id === "state" || id === "step") return 2 * Math.PI;
  if (id === "mass") return 8;
  if (id === "engine") return 5;
  if (id === "loopOpen") return 10;
  if (id === "loopError") return 14;
  if (id === "loopEquil") return 10;
  if (id === "loopHover") return 14;
  if (id === "loopMode") return 5;
  if (id === "loopSample") return 12;
  if (isTurnScene(id)) return turnDuration(id);
  if (isVehicleScene(id)) return vehicleDuration(id);
  if (isDescentScene(id)) return descentDuration(id);
  return 5;
}

export function isLoopScene(id: SceneId): boolean {
  return id.startsWith("loop");
}

export function isTurnScene(id: SceneId): id is TurnId {
  return id.startsWith("turn");
}

export function isVehicleScene(id: SceneId): id is VehicleId {
  return id.startsWith("veh");
}

export function isDescentScene(id: SceneId): id is DescentId {
  return id.startsWith("des");
}

export function clampKnob(id: SceneId, value: number): number {
  if (id === "step") return clampNamed("dt", value);
  if (id === "mass") return clampNamed("thrust", value);
  if (id === "engine") return clampNamed("tau", value);
  if (id === "loopSample") return clampNamed("tau", value);
  if (id === "loopOpen") return clampNamed("extraThrust", value);
  if (id === "loopError") return clampNamed("kp", value);
  if (id === "loopEquil") return clampNamed("perturb", value);
  if (id === "loopHover") return clampNamed("unstable", value);
  if (id === "loopMode") return clampNamed("coupling", value);
  if (id === "turnFrames") return clampNamed("yaw", value);
  if (id === "turnLock") return clampNamed("pitch", value);
  if (id === "turnQuat") return clampNamed("tumble", value);
  if (id === "turnSpin") return clampNamed("rate", value);
  if (id === "turnInertia") return clampNamed("inertia", value);
  if (id === "turnGimbal") return clampNamed("gimbal", value);
  if (id === "turnAir") return clampNamed("margin", value);
  if (id === "turnSlosh") return clampNamed("slosh", value);
  if (id === "vehAscent") return clampNamed("lift", value);
  if (id === "vehNose") return clampNamed("lean", value);
  if (id === "vehWind") return clampNamed("wind", value);
  if (id === "vehEstimate") return clampNamed("trust", value);
  if (id === "vehGuide") return clampNamed("kick", value);
  if (id === "vehAuto") return clampNamed("hold", value);
  if (id === "desCost") return clampNamed("care", value);
  if (id === "desPath") return clampNamed("cone", value);
  if (id === "desLoss") return clampNamed("floor", value);
  if (id === "desScvx") return clampNamed("pass", value);
  if (id === "desGusto") return clampNamed("region", value);
  if (id === "desReplan") return clampNamed("gust", value);
  if (id === "desMiss") return clampNamed("fuel", value);
  return value;
}

function clamp(value: number, lo: number, hi: number): number {
  return Math.min(hi, Math.max(lo, value));
}

function blank(t: number): Row {
  return {
    t,
    height: 0,
    rate: 0,
    stepped: 0,
    truth: 0,
    accel: 0,
    mass: 0,
    command: 0,
    actual: 0,
    reference: 0,
    pHeight: 0,
    iHeight: 0,
    dHeight: 0,
    nonlinear: 0,
    linear: 0,
    openHeight: 0,
    closedHeight: 0,
    measured: 0,
    hiddenMode: 0,
    continuous: 0,
    discrete: 0,
    thrust: 0,
    worldX: 0,
    worldZ: 0,
    pitch: 0,
    axisGap: 0,
    quatW: 0,
    quatZ: 0,
    angle: 0,
    spin: 0,
    side: 0,
    moment: 0,
    attack: 0,
    fill: 0,
    offset: 0,
    downrange: 0,
    estimate: 0,
    aim: 0,
  };
}

export function springHeight(t: number, x0 = 1, v0 = 0): number {
  return x0 * Math.cos(t) + v0 * Math.sin(t);
}

export function springRate(t: number, x0 = 1, v0 = 0): number {
  return -x0 * Math.sin(t) + v0 * Math.cos(t);
}

function springRows(x0: number, v0: number): Row[] {
  const rows: Row[] = [];
  const end = durationOf("state");
  const startHeight = clampNamed("x0", x0);
  const startRate = clampNamed("v0", v0);
  for (let t = 0; t <= end + 1e-9; t += 0.05) {
    const row = blank(t);
    row.height = springHeight(t, startHeight, startRate);
    row.rate = springRate(t, startHeight, startRate);
    row.accel = (-SPRING_K / SPRING_M) * row.height;
    row.truth = row.height;
    rows.push(row);
  }
  return rows;
}

// Explicit Euler: both updates use the old state, so energy grows.
function eulerStep(height: number, rate: number, step: number): { height: number; rate: number } {
  const accel = -SPRING_K * height;
  return {
    height: height + step * rate,
    rate: rate + step * accel,
  };
}

// Semi-implicit (symplectic) Euler: rate first, then height with the new rate.
function semiImplicitStep(height: number, rate: number, step: number): { height: number; rate: number } {
  const accel = -SPRING_K * height;
  const nextRate = rate + step * accel;
  return {
    height: height + step * nextRate,
    rate: nextRate,
  };
}

function integratorStep(
  height: number,
  rate: number,
  step: number,
  method: number,
): { height: number; rate: number } {
  if (method >= 0.5) return semiImplicitStep(height, rate, step);
  return eulerStep(height, rate, step);
}

function eulerHold(t: number, dt: number, method = 0): { height: number; rate: number } {
  let height = 1;
  let rate = 0;
  let tau = 0;
  const step = clampKnob("step", dt);
  while (tau + step <= t + 1e-9) {
    const next = integratorStep(height, rate, step, method);
    height = next.height;
    rate = next.rate;
    tau += step;
  }
  return { height, rate };
}

export function steppedHeight(t: number, dt: number, method = 0): number {
  return eulerHold(t, dt, method).height;
}

export function steppedRate(t: number, dt: number, method = 0): number {
  return eulerHold(t, dt, method).rate;
}

function springEnergy(height: number, rate: number): number {
  return 0.5 * SPRING_M * rate * rate + 0.5 * SPRING_K * height * height;
}

function stairPoints(dt: number, until: number, method = 0): [number, number][] {
  const step = clampKnob("step", dt);
  const end = Math.min(until, durationOf("step"));
  const points: [number, number][] = [[0, 1]];
  let height = 1;
  let rate = 0;
  let tau = 0;
  while (tau + step <= end + 1e-9) {
    const nextTime = tau + step;
    points.push([nextTime, height]);
    const next = integratorStep(height, rate, step, method);
    points.push([nextTime, next.height]);
    height = next.height;
    rate = next.rate;
    tau = nextTime;
  }
  if (end > tau + 1e-6) points.push([end, height]);
  return points;
}

function stepRows(dt: number, method: number): Row[] {
  const rows: Row[] = [];
  const sample = 0.02;
  for (let t = 0; t <= durationOf("step") + 1e-9; t += sample) {
    const held = eulerHold(t, dt, method);
    const row = blank(t);
    row.height = held.height;
    row.rate = held.rate;
    row.truth = springHeight(t);
    row.stepped = held.height;
    row.accel = springEnergy(held.height, held.rate);
    rows.push(row);
  }
  return rows;
}

/** Pad contact: the rocket sits on the ground instead of falling through it. */
function holdPad(height: number, rate: number): { height: number; rate: number } {
  if (height >= 0) return { height, rate };
  return { height: 0, rate: Math.max(rate, 0) };
}

function massRows(thrust: number): Row[] {
  const rows: Row[] = [];
  const force = clampKnob("mass", thrust);
  let velocity = 0;
  let height = 0;
  const step = 0.02;
  for (let t = 0; t <= durationOf("mass") + 1e-9; t += step) {
    const fuelLeft = Math.max(0, FUEL * (1 - t / BURN));
    const mass = DRY + fuelLeft;
    const pushing = t < BURN ? force : 0;
    const accel = pushing / mass - G;
    const row = blank(t);
    row.height = height;
    row.rate = velocity;
    row.accel = accel;
    row.mass = mass;
    row.actual = pushing;
    rows.push(row);
    velocity += accel * step;
    height += velocity * step;
    const held = holdPad(height, velocity);
    height = held.height;
    velocity = held.rate;
  }
  return rows;
}

function engineStepAt(tau: number, lead: number): number {
  const lag = clampKnob("engine", tau);
  if (lead >= 0.5) return Math.max(0, ASK_AT - lag);
  return ASK_AT;
}

function engineRows(tau: number, ask: number, lead: number): Row[] {
  const rows: Row[] = [];
  const lag = clampKnob("engine", tau);
  const asked = clampNamed("ask", ask);
  const stepAt = engineStepAt(tau, lead);
  const mass = 2.5;
  let thrust = 0;
  let velocity = 0;
  let height = 0;
  const step = 0.02;
  for (let t = 0; t <= durationOf("engine") + 1e-9; t += step) {
    const command = t < stepAt ? 0 : asked;
    thrust += ((command - thrust) / lag) * step;
    if (thrust > LIMIT) thrust = LIMIT;
    if (thrust < 0) thrust = 0;
    const accel = thrust / mass - G;
    const row = blank(t);
    row.height = height;
    row.rate = velocity;
    row.accel = accel;
    row.command = command;
    row.actual = thrust;
    rows.push(row);
    velocity += accel * step;
    height += velocity * step;
    const held = holdPad(height, velocity);
    height = held.height;
    velocity = held.rate;
  }
  return rows;
}

let rowCache: { key: string; rows: Row[] } | null = null;

function fromLoopRow(row: ReturnType<typeof loopRowAt>): Row {
  const next = blank(row.t);
  next.height = row.height;
  next.rate = row.rate;
  next.command = row.command;
  next.actual = row.actual;
  next.reference = row.reference;
  next.pHeight = row.pHeight;
  next.iHeight = row.iHeight;
  next.dHeight = row.dHeight;
  next.nonlinear = row.nonlinear;
  next.linear = row.linear;
  next.openHeight = row.openHeight;
  next.closedHeight = row.closedHeight;
  next.measured = row.measured;
  next.hiddenMode = row.hiddenMode;
  next.continuous = row.continuous;
  next.discrete = row.discrete;
  next.thrust = row.thrust;
  return next;
}

function fromTurnSample(row: TurnSample): Row {
  const next = blank(row.t);
  next.worldX = row.worldX;
  next.worldZ = row.worldZ;
  next.pitch = row.pitch;
  next.axisGap = row.axisGap;
  next.quatW = row.quatW;
  next.quatZ = row.quatZ;
  next.angle = row.angle;
  next.spin = row.spin;
  next.side = row.side;
  next.moment = row.moment;
  next.attack = row.attack;
  next.fill = row.fill;
  next.offset = row.offset;
  return next;
}

function fromVehicleSample(row: VehicleSample): Row {
  const next = blank(row.t);
  next.height = row.height;
  next.rate = row.rate;
  next.downrange = row.downrange;
  next.attack = row.attack;
  next.estimate = row.estimate;
  next.measured = row.measured;
  next.pitch = row.pitch;
  next.aim = row.aim;
  next.mass = row.mass;
  next.accel = row.accel;
  next.thrust = row.thrust;
  return next;
}

function fromDescentSample(row: DescentSample): Row {
  const next = blank(row.t);
  next.height = row.height;
  next.rate = row.rate;
  next.downrange = row.downrange;
  next.thrust = row.thrust;
  next.command = row.command;
  next.reference = row.reference;
  return next;
}

export function rowsFor(id: SceneId, knobs: Knobs): Row[] {
  const key = `${id}|${knobs.dt}|${knobs.thrust}|${knobs.tau}|${knobs.ask}|${knobs.kp}|${knobs.ki}|${knobs.kd}|${knobs.perturb}|${knobs.unstable}|${knobs.coupling}|${knobs.extraThrust}|${knobs.yaw}|${knobs.pitch}|${knobs.tumble}|${knobs.rate}|${knobs.inertia}|${knobs.gimbal}|${knobs.margin}|${knobs.slosh}|${knobs.lift}|${knobs.lean}|${knobs.wind}|${knobs.trust}|${knobs.kick}|${knobs.hold}|${knobs.care}|${knobs.spend}|${knobs.cone}|${knobs.floor}|${knobs.pass}|${knobs.region}|${knobs.gust}|${knobs.fuel}|${knobs.noise}|${knobs.schedule}|${knobs.twist}|${knobs.method}|${knobs.x0}|${knobs.v0}|${knobs.lead}|${knobs.cause}`;
  if (rowCache && rowCache.key === key) return rowCache.rows;

  let rows: Row[];
  if (id === "state") rows = springRows(knobs.x0, knobs.v0);
  else if (id === "step") rows = stepRows(knobs.dt, knobs.method);
  else if (id === "mass") rows = massRows(knobs.thrust);
  else if (id === "engine") rows = engineRows(knobs.tau, knobs.ask, knobs.lead);
  else if (isTurnScene(id)) rows = turnRows(id, knobs).map(fromTurnSample);
  else if (isVehicleScene(id)) rows = vehicleRows(id, knobs).map(fromVehicleSample);
  else if (isDescentScene(id)) rows = descentRows(id, knobs).map(fromDescentSample);
  else rows = loopRowsFor(id, knobs).map(fromLoopRow);

  rowCache = { key, rows };
  return rows;
}

export function rowAt(rows: Row[], t: number): Row {
  let chosen = rows[0];
  for (const row of rows) {
    if (row.t <= t + 1e-9) chosen = row;
    else break;
  }
  return chosen;
}

export type BodyDraw = {
  height: number;
  truth: number;
  flame: number;
  yaw: number;
  tilt: number;
  gimbal: number;
  slosh: number;
  cp: number;
  fill: number;
  drift: number;
  aim: number;
  estimate: number;
};

function still(height: number, truth: number, flame: number): BodyDraw {
  return { height, truth, flame, yaw: 0, tilt: 0, gimbal: 0, slosh: 0, cp: 0, fill: 1, drift: 0, aim: 0, estimate: height };
}

export function bodyAt(id: SceneId, t: number, knobs: Knobs): BodyDraw {
  if (id === "state") {
    const height = springHeight(t, knobs.x0, knobs.v0);
    return still(height, height, 0);
  }
  if (id === "step") {
    return still(steppedHeight(t, knobs.dt, knobs.method), springHeight(t), 0);
  }
  if (isTurnScene(id)) {
    const sample = turnAt(id, t, knobs);
    return {
      height: 0,
      truth: 0,
      flame: sample.flame,
      yaw: sample.yaw,
      tilt: sample.tilt,
      gimbal: sample.gimbal,
      slosh: sample.slosh,
      cp: sample.cp,
      fill: id === "turnSlosh" ? sample.fill / 1.5 : 1,
      drift: 0,
      aim: 0,
      estimate: 0,
    };
  }
  if (isVehicleScene(id)) {
    const sample = vehicleAt(id, t, knobs);
    return {
      height: sample.height,
      truth: sample.height,
      flame: sample.flame,
      yaw: 0,
      tilt: sample.tilt,
      gimbal: 0,
      slosh: 0,
      cp: 0,
      fill: sample.mass > 0 ? sample.mass / 3 : 1,
      drift: sample.drift,
      aim: sample.aim / (180 / Math.PI),
      estimate: sample.estimate,
    };
  }
  if (isDescentScene(id)) {
    const sample = descentAt(id, t, knobs);
    return {
      height: sample.height,
      truth: sample.height,
      flame: sample.flame,
      yaw: 0,
      tilt: sample.tilt,
      gimbal: 0,
      slosh: 0,
      cp: 0,
      fill: 1,
      drift: sample.drift,
      aim: 0,
      estimate: sample.height,
    };
  }
  const row = rowAt(rowsFor(id, knobs), t);
  if (id === "mass") {
    const burning = t < BURN && knobs.thrust > 0;
    return still(row.height, row.height, burning ? knobs.thrust / LIMITS.thrust.max : 0);
  }
  if (isLoopScene(id)) {
    const thrust = row.actual || row.thrust || LOOP_T0;
    return still(row.height, row.height, Math.min(1, Math.max(0, (thrust - LOOP_T0 * 0.85) / (LOOP_LIMIT - LOOP_T0 * 0.85))));
  }
  return still(row.height, row.height, row.actual / LIMIT);
}

export type PlotSeries = {
  name: string;
  column: Column;
  gold: boolean;
  points: [number, number][];
  axis: 0 | 1;
};

export type ColumnUnit = "m" | "m/s" | "m/s2" | "kg" | "N" | "Nm" | "deg" | "deg/s" | "unitless";

export const COLUMNS: Record<Column, { name: string; label: string; unit: ColumnUnit; words: string[] }> = {
  height: { name: "Height", label: "Height (m)", unit: "m", words: ["height", "state", "position"] },
  rate: { name: "Rate", label: "Rate (m/s)", unit: "m/s", words: ["rate", "slope", "speed"] },
  stepped: { name: "Step", label: "Step (m)", unit: "m", words: ["step", "stair", "euler"] },
  truth: { name: "True height", label: "True height (m)", unit: "m", words: ["true", "truth", "exact"] },
  accel: { name: "Acceleration", label: "Acceleration (m/s²)", unit: "m/s2", words: ["accel", "acceleration"] },
  mass: { name: "Mass", label: "Mass (kg)", unit: "kg", words: ["mass", "weight", "tank"] },
  command: { name: "Command", label: "Command (N)", unit: "N", words: ["command", "ask"] },
  actual: { name: "Actual", label: "Actual (N)", unit: "N", words: ["actual", "thrust", "engine"] },
  reference: { name: "Reference", label: "Reference (m)", unit: "m", words: ["reference", "target", "setpoint"] },
  pHeight: { name: "P response", label: "P (m)", unit: "m", words: ["proportional", "p"] },
  iHeight: { name: "I response", label: "I (m)", unit: "m", words: ["integral", "i"] },
  dHeight: { name: "D response", label: "D (m)", unit: "m", words: ["derivative", "d"] },
  nonlinear: { name: "Nonlinear", label: "Nonlinear (m)", unit: "m", words: ["nonlinear", "full"] },
  linear: { name: "Linear", label: "Linear (m)", unit: "m", words: ["linear", "linearized"] },
  openHeight: { name: "Open loop", label: "Open loop (m)", unit: "m", words: ["open", "unstable"] },
  closedHeight: { name: "Closed loop", label: "Closed loop (m)", unit: "m", words: ["closed", "feedback"] },
  measured: { name: "Measured", label: "Measured (m)", unit: "m", words: ["measured", "sensor"] },
  hiddenMode: { name: "Hidden mode", label: "Hidden mode", unit: "unitless", words: ["hidden", "mode", "internal"] },
  continuous: { name: "Continuous", label: "Continuous (m)", unit: "m", words: ["continuous", "analog"] },
  discrete: { name: "Discrete", label: "Discrete (m)", unit: "m", words: ["discrete", "sample", "digital"] },
  thrust: { name: "Thrust", label: "Thrust (N)", unit: "N", words: ["thrust", "force"] },
  worldX: { name: "World X", label: "World X", unit: "unitless", words: ["world", "x", "component"] },
  worldZ: { name: "World Z", label: "World Z", unit: "unitless", words: ["world", "z", "component"] },
  pitch: { name: "Pitch", label: "Pitch (deg)", unit: "deg", words: ["pitch", "tip"] },
  axisGap: { name: "Axis gap", label: "Axis gap (deg)", unit: "deg", words: ["gap", "lock", "axis"] },
  quatW: { name: "quat w", label: "quat w", unit: "unitless", words: ["quaternion", "w"] },
  quatZ: { name: "quat z", label: "quat z", unit: "unitless", words: ["quaternion", "z"] },
  angle: { name: "Angle", label: "Angle (deg)", unit: "deg", words: ["angle", "theta"] },
  spin: { name: "Spin", label: "Spin (deg/s)", unit: "deg/s", words: ["spin", "rate", "omega"] },
  side: { name: "Side force", label: "Side force (N)", unit: "N", words: ["side", "force"] },
  moment: { name: "Moment", label: "Moment (N·m)", unit: "Nm", words: ["moment", "torque"] },
  attack: { name: "Attack", label: "Attack (deg)", unit: "deg", words: ["attack", "alpha", "aoa"] },
  fill: { name: "Fuel", label: "Fuel (kg)", unit: "kg", words: ["fuel", "fill", "propellant"] },
  offset: { name: "Offset", label: "Offset (m)", unit: "m", words: ["offset", "slosh", "cg"] },
  downrange: { name: "Downrange", label: "Downrange (m)", unit: "m", words: ["downrange", "range", "sideways"] },
  estimate: { name: "Estimate", label: "Estimate (m)", unit: "m", words: ["estimate", "filter", "prediction"] },
  aim: { name: "Aim", label: "Aim (deg)", unit: "deg", words: ["aim", "command", "guidance", "kick"] },
};

export function columnName(column: Column): string {
  return COLUMNS[column].name;
}

export function columnLabel(column: Column): string {
  return COLUMNS[column].label;
}

export function columnUnit(column: Column): ColumnUnit {
  return COLUMNS[column].unit;
}

export function columnWords(column: Column): string[] {
  return COLUMNS[column].words;
}

export function chartColumns(id: SceneId): Column[] {
  return sceneById(id).askable;
}

export function visualContext(id: SceneId): string {
  return sceneById(id).visual;
}

export function seriesPoints(id: SceneId, column: Column, knobs: Knobs, until: number): [number, number][] {
  if (id === "step" && column === "stepped") return stairPoints(knobs.dt, until, knobs.method);
  const rows = rowsFor(id, knobs).filter((row) => row.t <= until + 1e-9);
  // Estimate scene: chart sensor and estimate as errors from truth so the wave is visible.
  const points = rows.map((row) => {
    let value = row[column];
    if (id === "vehEstimate" && (column === "measured" || column === "estimate")) {
      value = row[column] - row.height;
    }
    return [row.t, value] as [number, number];
  });
  const wantsTip = id === "state" || (id === "step" && column === "truth");
  if (wantsTip) {
    const tip = points[points.length - 1];
    if (!tip || Math.abs(tip[0] - until) > 1e-4) {
      // Step truth stays on the unit spring. The spring scene uses (x0, v0).
      const x0 = id === "state" ? knobs.x0 : 1;
      const v0 = id === "state" ? knobs.v0 : 0;
      const value = column === "rate" ? springRate(until, x0, v0) : springHeight(until, x0, v0);
      points.push([until, value]);
    }
  }
  return points;
}

export function plotOf(id: SceneId, columns: Column[], knobs: Knobs, until: number): PlotSeries[] {
  const primaryUnit = COLUMNS[columns[0] ?? "height"].unit;
  // Height and rate on the spring share one scale. Splitting them by unit
  // would park the rate on an axis that is not drawn.
  const shareScale = id === "state";
  return columns.map((column) => {
    // Unstable hover: open loop runs to huge heights. Put it on its own log axis
    // so the closed-loop landing stays readable. Hidden mode: keep measured height
    // on the left axis and the internal coordinate on the right.
    let axis: 0 | 1 = shareScale || COLUMNS[column].unit === primaryUnit ? 0 : 1;
    if (id === "loopHover" && column === "openHeight") axis = 1;
    if (id === "loopHover" && column === "closedHeight") axis = 0;
    if (id === "loopError" && column === "iHeight") axis = 1;
    if (id === "loopError" && (column === "pHeight" || column === "dHeight" || column === "closedHeight" || column === "reference")) axis = 0;
    if (id === "loopMode" && column === "hiddenMode") axis = 1;
    if (id === "loopMode" && column === "measured") axis = 0;
    if (id === "vehAuto" && column === "estimate") axis = 1;
    if (id === "vehAuto" && column === "pitch") axis = 0;
    let name = COLUMNS[column].label;
    if (id === "vehEstimate" && column === "measured") name = "Sensor − truth (m)";
    if (id === "vehEstimate" && column === "estimate") name = "Estimate − truth (m)";
    if (id === "vehAuto" && column === "estimate") name = "Gain used K";
    if (id === "vehAuto" && column === "measured") name = "Air stiffness κ";
    if (id === "vehAuto" && column === "rate") name = "Speed v (m/s)";
    if (id === "turnGimbal" && column === "pitch") name = "Gimbal δ₁ (°)";
    if (id === "turnGimbal" && column === "angle") name = "Gimbal δ₂ (°)";
    if (id === "loopError" && column === "closedHeight") name = "PID (m)";
    if (id === "loopError" && column === "iHeight") name = "I alone (m)";
    return {
      name,
      column,
      gold:
        column === "rate" ||
        column === "stepped" ||
        column === "accel" ||
        column === "actual" ||
        column === "height" ||
        column === "pHeight" ||
        column === "nonlinear" ||
        column === "openHeight" ||
        column === "measured" ||
        column === "discrete",
      points: seriesPoints(id, column, knobs, until),
      axis,
    };
  });
}

export function axisOf(id: SceneId, columns: Column[], knobs: Knobs): { y: [number, number]; y2?: [number, number]; y2Log?: boolean } {
  if (id === "state") {
    const reach = Math.max(1.25, Math.hypot(knobs.x0, knobs.v0) * 1.15);
    return { y: [-reach, reach] };
  }
  const full = plotOf(id, columns, knobs, durationOf(id));
  const primary = full.filter((series) => series.axis === 0).flatMap((series) => series.points.map((point) => point[1]));
  const secondary = full.filter((series) => series.axis === 1).flatMap((series) => series.points.map((point) => point[1]));
  if (id === "loopHover" && secondary.length) {
    const openValues = secondary.filter((value) => value > 0);
    const openHi = openValues.length ? Math.max(...openValues) : 1;
    const openLo = openValues.length ? Math.min(...openValues) : 0.01;
    return {
      y: pad(primary, true),
      y2: [Math.max(1e-3, openLo * 0.8), Math.max(openHi * 1.2, 1)],
      y2Log: true,
    };
  }
  if (id === "loopError" && secondary.length) {
    return {
      y: pad(primary, true),
      y2: pad(secondary, true),
    };
  }
  return {
    y: pad(primary, true),
    y2: secondary.length ? pad(secondary, false) : undefined,
  };
}

function pad(values: number[], includeZero: boolean): [number, number] {
  let lo = Math.min(...values);
  let hi = Math.max(...values);
  if (includeZero) {
    lo = Math.min(lo, 0);
    hi = Math.max(hi, 0);
  }
  const span = hi - lo || 1;
  let padLo = lo - span * 0.08;
  let padHi = hi + span * 0.08;
  // A trace that never crosses zero should not grow a large empty side.
  if (lo >= 0) padLo = 0;
  if (hi <= 0) padHi = 0;
  return nice(padLo, padHi);
}

function nice(lo: number, hi: number): [number, number] {
  const span = hi - lo || 1;
  const raw = span / 4;
  const exp = Math.pow(10, Math.floor(Math.log10(raw)));
  const base = raw / exp;
  let step = exp;
  if (base > 5) step = 10 * exp;
  else if (base > 2) step = 5 * exp;
  else if (base > 1) step = 2 * exp;
  const start = Math.floor(lo / step) * step;
  const end = Math.ceil(hi / step) * step;
  return [Math.round(start * 1000) / 1000, Math.round(end * 1000) / 1000];
}

function formatReadout(value: number, column: Column): string {
  const unit = COLUMNS[column].unit;
  const suffix =
    unit === "deg" ? "°" : unit === "deg/s" ? "°/s" : unit === "Nm" ? " N·m" : unit === "unitless" ? "" : ` ${unit.replace("m/s2", "m/s²")}`;
  const digits = Math.abs(value) >= 100 ? 0 : Math.abs(value) >= 10 ? 1 : 2;
  return `${value.toFixed(digits)}${suffix}`;
}

export type ReadoutLine = { name: string; text: string; column?: Column };

export function readout(id: SceneId, t: number, knobs: Knobs): ReadoutLine[] {
  if (id === "state") {
    return [
      { name: "Time", text: `${t.toFixed(2)} s` },
      { name: "Height", text: `${springHeight(t, knobs.x0, knobs.v0).toFixed(2)} m`, column: "height" },
      { name: "Rate", text: `${springRate(t, knobs.x0, knobs.v0).toFixed(2)} m/s`, column: "rate" },
    ];
  }
  if (id === "step") {
    const held = eulerHold(t, knobs.dt, knobs.method);
    return [
      { name: "Time", text: `${t.toFixed(2)} s` },
      { name: "Step", text: `${held.height.toFixed(2)} m`, column: "stepped" },
      { name: "True", text: `${springHeight(t).toFixed(2)} m`, column: "truth" },
      { name: "Energy", text: `${springEnergy(held.height, held.rate).toFixed(2)} J` },
    ];
  }
  const row = rowAt(rowsFor(id, knobs), t);
  if (id === "mass") {
    return [
      { name: "Time", text: `${t.toFixed(2)} s` },
      { name: "Height", text: `${row.height.toFixed(1)} m`, column: "height" },
      { name: "Rate", text: `${row.rate.toFixed(1)} m/s`, column: "rate" },
    ];
  }
  if (id === "desLoss") {
    return [
      { name: "Time", text: `${t.toFixed(2)} s` },
      { name: "Height", text: `${row.height.toFixed(1)} m`, column: "height" },
      { name: "Rate", text: `${row.rate.toFixed(1)} m/s`, column: "rate" },
      { name: "Thrust", text: `${row.thrust.toFixed(0)} N`, column: "thrust" },
      { name: "Command", text: `${row.command.toFixed(0)} N`, column: "command" },
    ];
  }
  if (id === "vehAscent") {
    const tw = ascentTw(knobs.lift);
    return [
      { name: "Time", text: `${t.toFixed(2)} s` },
      { name: "Height", text: `${row.height.toFixed(1)} m`, column: "height" },
      { name: "Rate", text: `${row.rate.toFixed(1)} m/s`, column: "rate" },
      { name: "T/W", text: tw.toFixed(2) },
    ];
  }
  if (id === "vehNose") {
    return [
      { name: "Time", text: `${t.toFixed(2)} s` },
      { name: "Height", text: `${row.height.toFixed(1)} m`, column: "height" },
      { name: "Downrange", text: `${row.downrange.toFixed(1)} m`, column: "downrange" },
      { name: "a_y", text: `${noseClimbAccel(knobs.lean).toFixed(2)} m/s²` },
    ];
  }
  if (id === "vehWind") {
    return [
      { name: "Time", text: `${t.toFixed(2)} s` },
      { name: "Attack", text: `${row.attack.toFixed(1)}°`, column: "attack" },
      { name: "Downrange", text: `${row.downrange.toFixed(1)} m`, column: "downrange" },
      { name: "Rate", text: `${row.rate.toFixed(1)} m/s`, column: "rate" },
    ];
  }
  if (id === "desMiss") {
    const outcome = missOutcome(knobs);
    const status =
      outcome.status === "infeasible" ? "INFEASIBLE" : outcome.status === "soft" ? "soft" : "miss";
    return [
      { name: "Time", text: `${t.toFixed(2)} s` },
      { name: "Height", text: `${row.height.toFixed(1)} m`, column: "height" },
      { name: "Rate", text: `${row.rate.toFixed(1)} m/s`, column: "rate" },
      { name: "Status", text: `${status} · |v|≤${TOUCH_SPEED}` },
    ];
  }
  if (isTurnScene(id) || isVehicleScene(id) || isDescentScene(id)) {
    const live = sceneById(id).live;
    return [
      { name: "Time", text: `${t.toFixed(2)} s` },
      { name: columnName(live[0]), text: formatReadout(row[live[0]], live[0]), column: live[0] },
      { name: columnName(live[1]), text: formatReadout(row[live[1]], live[1]), column: live[1] },
    ];
  }
  if (isLoopScene(id)) {
    return [
      { name: "Time", text: `${t.toFixed(2)} s` },
      { name: "Height", text: `${row.height.toFixed(2)} m`, column: "height" },
      { name: "Rate", text: `${row.rate.toFixed(2)} m/s`, column: "rate" },
    ];
  }
  return [
    { name: "Time", text: `${t.toFixed(2)} s` },
    { name: "Height", text: `${row.height.toFixed(1)} m`, column: "height" },
    { name: "Rate", text: `${row.rate.toFixed(1)} m/s`, column: "rate" },
    { name: "Actual", text: `${row.actual.toFixed(0)} N`, column: "actual" },
  ];
}

export type KnobControl = {
  key: KnobKey;
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  show: string;
  aside?: string;
};

function knob(
  key: KnobKey,
  label: string,
  value: number,
  min: number,
  max: number,
  step: number,
  show: string,
  aside?: string,
): KnobControl {
  return { key, label, value, min, max, step, show, aside };
}

export function knobsOf(id: SceneId, knobs: Knobs): KnobControl[] {
  if (id === "state") {
    return [
      knob("x0", "x0", knobs.x0, LIMITS.x0.min, LIMITS.x0.max, 0.05, `${knobs.x0.toFixed(2)} m`, "height at t=0"),
      knob("v0", "v0", knobs.v0, LIMITS.v0.min, LIMITS.v0.max, 0.05, `${knobs.v0.toFixed(2)} m/s`, "rate at t=0"),
    ];
  }
  if (id === "step") {
    const semi = knobs.method >= 0.5;
    return [
      knob("dt", "Step", knobs.dt, LIMITS.dt.min, LIMITS.dt.max, 0.05, `${knobs.dt.toFixed(2)} s`),
      knob("method", "Method", knobs.method, LIMITS.method.min, LIMITS.method.max, 1, semi ? "semi" : "Euler", semi ? "rate then height" : "explicit Euler"),
    ];
  }
  if (id === "mass") {
    return [knob("thrust", "Thrust", knobs.thrust, LIMITS.thrust.min, LIMITS.thrust.max, 1, `${knobs.thrust.toFixed(0)} N`)];
  }
  if (id === "engine") {
    const leading = knobs.lead >= 0.5;
    return [
      knob("ask", "Command", knobs.ask, LIMITS.ask.min, LIMITS.ask.max, 1, `${knobs.ask.toFixed(0)} N`, "ceiling 50 N"),
      knob("tau", "Lag", knobs.tau, LIMITS.tau.min, LIMITS.tau.max, 0.01, `${knobs.tau.toFixed(2)} s`),
      knob(
        "lead",
        "Lead",
        knobs.lead,
        LIMITS.lead.min,
        LIMITS.lead.max,
        1,
        leading ? "on" : "off",
        leading ? `ask at ${engineStepAt(knobs.tau, 1).toFixed(2)} s (need ${ASK_AT.toFixed(1)} s)` : `ask at need ${ASK_AT.toFixed(1)} s`,
      ),
    ];
  }
  if (id === "loopOpen") {
    return [knob("extraThrust", "Extra", knobs.extraThrust, LIMITS.extraThrust.min, LIMITS.extraThrust.max, 1, `${knobs.extraThrust.toFixed(0)} N`)];
  }
  if (id === "loopError") {
    return [
      knob("kp", "Kp", knobs.kp, LIMITS.kp.min, LIMITS.kp.max, 1, knobs.kp.toFixed(0), "proportional"),
      knob("ki", "Ki", knobs.ki, LIMITS.ki.min, LIMITS.ki.max, 1, knobs.ki.toFixed(0), "integral"),
      knob("kd", "Kd", knobs.kd, LIMITS.kd.min, LIMITS.kd.max, 1, knobs.kd.toFixed(0), "derivative"),
    ];
  }
  if (id === "loopEquil") {
    return [knob("perturb", "Offset", knobs.perturb, LIMITS.perturb.min, LIMITS.perturb.max, 0.02, `${knobs.perturb.toFixed(2)} m`)];
  }
  if (id === "loopHover") {
    const mu2 = knobs.unstable * knobs.unstable;
    const margin = LOOP_HOVER_KP / LOOP_M - mu2;
    return [
      knob(
        "unstable",
        "Open μ",
        knobs.unstable,
        LIMITS.unstable.min,
        LIMITS.unstable.max,
        0.1,
        `${knobs.unstable.toFixed(1)}`,
        `μ² ${mu2.toFixed(2)} · Kp/m ${((LOOP_HOVER_KP / LOOP_M).toFixed(2))} · margin ${margin.toFixed(2)}`,
      ),
    ];
  }
  if (id === "loopMode") {
    return [knob("coupling", "Coupling", knobs.coupling, LIMITS.coupling.min, LIMITS.coupling.max, 0.02, `${knobs.coupling.toFixed(2)}`)];
  }
  if (id === "loopSample") {
    return [
      knob("dt", "Sample", knobs.dt, LIMITS.dt.min, LIMITS.dt.max, 0.01, `${knobs.dt.toFixed(2)} s`),
      knob(
        "tau",
        "Delay",
        knobs.tau,
        LIMITS.tau.min,
        LIMITS.tau.max,
        0.01,
        `${knobs.tau.toFixed(2)} s`,
        `mark ~${LOOP_SAMPLE_DELAY_MARK.toFixed(2)} s`,
      ),
    ];
  }
  if (id === "turnFrames") return [knob("yaw", "Yaw", knobs.yaw, LIMITS.yaw.min, LIMITS.yaw.max, 1, `${knobs.yaw.toFixed(0)}°`)];
  if (id === "turnLock") return [knob("pitch", "Pitch", knobs.pitch, LIMITS.pitch.min, LIMITS.pitch.max, 1, `${knobs.pitch.toFixed(0)}°`)];
  if (id === "turnQuat") return [knob("tumble", "Pitch", knobs.tumble, LIMITS.tumble.min, LIMITS.tumble.max, 1, `${knobs.tumble.toFixed(0)}°`)];
  if (id === "turnSpin") return [knob("rate", "Spin", knobs.rate, LIMITS.rate.min, LIMITS.rate.max, 1, `${knobs.rate.toFixed(0)}°/s`)];
  if (id === "turnInertia") return [knob("inertia", "Inertia", knobs.inertia, LIMITS.inertia.min, LIMITS.inertia.max, 0.1, `${knobs.inertia.toFixed(1)} kg·m²`)];
  if (id === "turnGimbal") {
    const rows = turnRows("turnGimbal", knobs);
    const end = rows[rows.length - 1];
    const hit = Math.abs(end.pitch) >= 14.9 || Math.abs(end.angle) >= 14.9;
    const aside = `δ₁ ${end.pitch.toFixed(1)}° · δ₂ ${end.angle.toFixed(1)}°${hit ? " · limit" : ""}`;
    return [
      knob("gimbal", "Side Fy", knobs.gimbal, LIMITS.gimbal.min, LIMITS.gimbal.max, 1, `${knobs.gimbal.toFixed(0)} N`, aside),
      knob("twist", "Moment M", knobs.twist, LIMITS.twist.min, LIMITS.twist.max, 1, `${knobs.twist.toFixed(0)} N·m`, "±15° actuators"),
    ];
  }
  if (id === "turnAir") {
    return [
      knob(
        "margin",
        "Margin",
        knobs.margin,
        LIMITS.margin.min,
        LIMITS.margin.max,
        0.1,
        `${knobs.margin.toFixed(1)} cal`,
        knobs.margin < 0 ? "unstable" : "restoring",
      ),
    ];
  }
  if (id === "turnSlosh") return [knob("slosh", "Swing", knobs.slosh, LIMITS.slosh.min, LIMITS.slosh.max, 1, `${knobs.slosh.toFixed(0)}°`)];
  if (id === "vehAscent") {
    const weight = ascentWeight();
    const tw = ascentTw(knobs.lift);
    return [
      knob(
        "lift",
        "Thrust",
        knobs.lift,
        LIMITS.lift.min,
        LIMITS.lift.max,
        1,
        `${knobs.lift.toFixed(0)} N`,
        `T/W ${tw.toFixed(2)} · weight ${weight.toFixed(1)} N`,
      ),
    ];
  }
  if (id === "vehNose") {
    const ay = noseClimbAccel(knobs.lean);
    const breakEven = noseBreakEvenDeg();
    const aside =
      ay <= 0
        ? `a_y ${ay.toFixed(2)} · pad · θ* ${breakEven.toFixed(1)}°`
        : `a_y ${ay.toFixed(2)} · a_x ${noseSideAccel(knobs.lean).toFixed(2)} · θ* ${breakEven.toFixed(1)}°`;
    return [knob("lean", "Tilt", knobs.lean, LIMITS.lean.min, LIMITS.lean.max, 1, `${knobs.lean.toFixed(0)}°`, aside)];
  }
  if (id === "vehWind") {
    const side = 0.1 * knobs.wind;
    return [
      knob(
        "wind",
        "Wind",
        knobs.wind,
        LIMITS.wind.min,
        LIMITS.wind.max,
        1,
        `${knobs.wind.toFixed(0)} m/s`,
        `a_x ${side.toFixed(2)} m/s² · α→90° at liftoff`,
      ),
    ];
  }
  if (id === "vehEstimate") {
    return [
      knob("trust", "Gain", knobs.trust, LIMITS.trust.min, LIMITS.trust.max, 0.1, knobs.trust.toFixed(1), "fixed-gain filter"),
      knob("noise", "Noise", knobs.noise, LIMITS.noise.min, LIMITS.noise.max, 0.5, `${knobs.noise.toFixed(1)} m`, "sensor wave"),
    ];
  }
  if (id === "vehGuide") {
    return [knob("kick", "Target", knobs.kick, LIMITS.kick.min, LIMITS.kick.max, 5, `${knobs.kick.toFixed(0)} m`, "ground target x")];
  }
  if (id === "vehAuto") {
    const scheduled = knobs.schedule >= 0.5;
    return [
      knob("hold", "Gain", knobs.hold, LIMITS.hold.min, LIMITS.hold.max, 0.5, knobs.hold.toFixed(1), scheduled ? "base K₀" : "fixed K"),
      knob(
        "schedule",
        "Schedule",
        knobs.schedule,
        LIMITS.schedule.min,
        LIMITS.schedule.max,
        1,
        scheduled ? "K(m)" : "fixed",
        "0 fixed · 1 K∝m",
      ),
    ];
  }
  if (id === "desCost") {
    const gains = costGains(knobs.care, knobs.spend);
    return [
      knob(
        "care",
        "Weight q",
        knobs.care,
        LIMITS.care.min,
        LIMITS.care.max,
        0.05,
        knobs.care.toFixed(2),
        `kp ${gains.kp.toFixed(2)}  kd ${gains.kd.toFixed(2)}`,
      ),
      knob("spend", "Weight r", knobs.spend, LIMITS.spend.min, LIMITS.spend.max, 0.05, knobs.spend.toFixed(2), "thrust cost"),
    ];
  }
  if (id === "desPath") {
    return [
      knob(
        "cone",
        "Cone",
        knobs.cone,
        LIMITS.cone.min,
        LIMITS.cone.max,
        1,
        `${knobs.cone.toFixed(0)}°`,
        "T 6–55 N",
      ),
    ];
  }
  if (id === "desLoss") {
    return [
      knob(
        "floor",
        "Floor",
        knobs.floor,
        LIMITS.floor.min,
        LIMITS.floor.max,
        1,
        `${knobs.floor.toFixed(0)} N`,
        `σ = |T| · ceiling ${55} N`,
      ),
    ];
  }
  if (id === "desScvx") {
    const infos = scvxPassInfo();
    const index = Math.min(infos.length, Math.max(1, Math.round(knobs.pass))) - 1;
    const info = infos[index];
    const aside = info
      ? `err ${info.error.toFixed(2)} m · Δ ${info.radius.toFixed(1)} N · ${info.accepted ? "accept" : "reject"}`
      : undefined;
    return [knob("pass", "Pass", knobs.pass, LIMITS.pass.min, LIMITS.pass.max, 1, `${Math.round(knobs.pass)}`, aside)];
  }
  if (id === "desGusto") {
    // Force the GuSTO iterate so the aside matches the knob.
    descentRows("desGusto", knobs);
    const info = gustoInfo();
    const aside = info
      ? `err ${info.error.toFixed(2)} m · Δ ${info.radius.toFixed(1)} N · ${info.accepts}↑ ${info.rejects}↓`
      : undefined;
    return [knob("region", "Trust", knobs.region, LIMITS.region.min, LIMITS.region.max, 0.5, `${knobs.region.toFixed(1)} N`, aside)];
  }
  if (id === "desReplan") {
    const rows = descentRows("desReplan", knobs);
    const end = rows[rows.length - 1];
    const missed = Math.abs(end.downrange) > 1.2 || (end.height < 0.5 && Math.abs(end.rate) > 0.8);
    const aside = missed
      ? `miss · |x| ${Math.abs(end.downrange).toFixed(1)} m · T≤40 N · cone 12°`
      : `ok · T≤40 N · cone 12°`;
    return [knob("gust", "Gust", knobs.gust, LIMITS.gust.min, LIMITS.gust.max, 0.5, `${knobs.gust.toFixed(1)} m/s`, aside)];
  }
  if (id === "desMiss") {
    const hard = knobs.cause >= 0.5;
    const outcome = missOutcome(knobs);
    const status =
      outcome.status === "infeasible" ? "INFEASIBLE" : outcome.status === "soft" ? "soft" : "miss";
    return [
      knob(
        "cause",
        "Cause",
        knobs.cause,
        LIMITS.cause.min,
        LIMITS.cause.max,
        1,
        hard ? "ceiling" : "fuel",
        hard ? `T≤${MISS_HARD_CEILING} N · ${status}` : `${status} · |v|≤${TOUCH_SPEED} m/s`,
      ),
      ...(hard
        ? []
        : [knob("fuel", "Fuel", knobs.fuel, LIMITS.fuel.min, LIMITS.fuel.max, 0.05, `${knobs.fuel.toFixed(2)} kg`)]),
    ];
  }
  return [];
}

export function dragHint(id: SceneId): string {
  if (id === "state") return "Drag to set height";
  if (id === "step") return "Drag the mass";
  return "Drag the rocket";
}

export function viewSpan(id: SceneId, knobs: Knobs): { min: number; max: number } {
  if (id === "state") {
    const reach = Math.max(1, Math.hypot(knobs.x0, knobs.v0) * 1.1);
    return { min: -reach, max: reach };
  }
  if (id === "step") {
    let min = -1;
    let max = 1;
    for (const point of stairPoints(knobs.dt, durationOf("step"), knobs.method)) {
      min = Math.min(min, point[1]);
      max = Math.max(max, point[1]);
    }
    return { min, max };
  }
  if (isTurnScene(id)) return { min: -1, max: 1 };
  const rows = rowsFor(id, knobs);
  let min = 0;
  let max = 0;
  for (const row of rows) {
    min = Math.min(min, row.height);
    max = Math.max(max, row.height);
  }
  if (max - min < 4) max = min + 4;
  return { min, max };
}

export type ChartMarker = { t: number; label: string };

export function chartMarkers(id: SceneId, knobs: Knobs): ChartMarker[] {
  if (id === "mass") return [{ t: BURN, label: "tank empty" }];
  if (id === "engine") {
    if (knobs.lead >= 0.5) {
      return [
        { t: engineStepAt(knobs.tau, knobs.lead), label: "lead ask" },
        { t: ASK_AT, label: "need" },
      ];
    }
    return [{ t: ASK_AT, label: "command" }];
  }
  if (id === "loopOpen") return [{ t: 1, label: "extra thrust" }];
  if (id === "loopError") return [{ t: 2, label: "target step" }];
  if (id === "loopMode") return [{ t: 1.5, label: "target step" }];
  if (id === "turnGimbal") return [{ t: 1.5, label: "commands hold" }];
  if (id === "turnLock" && knobs.pitch >= 90) return [{ t: durationOf(id), label: "nose vertical" }];
  if (id === "turnQuat") return [{ t: durationOf(id) * (90 / knobs.tumble), label: "nose vertical" }];
  if (id === "turnAir" && knobs.margin < 0) {
    const rows = rowsFor(id, knobs);
    const tumble = rows.find((row) => Math.abs(row.attack) >= 179.5);
    return [{ t: tumble ? tumble.t : 0, label: "unstable" }];
  }
  if (id === "desCost") {
    const rows = rowsFor(id, knobs);
    if (rows[0].thrust > 0.05) return [];
    const lift = rows.find((row) => row.thrust > 0.05);
    return [{ t: lift ? lift.t : 0, label: "thrust floor" }];
  }
  if (id === "desPath") {
    const rows = rowsFor(id, knobs);
    const outside = rows.find((row) => Math.abs(row.downrange) > row.reference + 0.05);
    return outside ? [{ t: outside.t, label: "outside cone" }] : [];
  }
  if (id === "desLoss") {
    const rows = rowsFor(id, knobs);
    const over = rows.find((row) => row.command > 55 + 0.05);
    return over ? [{ t: over.t, label: "above ceiling" }] : [];
  }
  if (id === "vehGuide") return [{ t: 0, label: "target" }];
  if (id === "vehAuto") return [{ t: 0.4, label: "pitch command" }];
  if (id === "desReplan") {
    const markers: ChartMarker[] = [{ t: 2.2, label: "gust" }];
    const rows = rowsFor(id, knobs);
    const cone = (12 * Math.PI) / 180;
    const outside = rows.find((row) => Math.abs(row.downrange) > row.height * Math.tan(cone) + 0.05);
    if (outside) markers.push({ t: outside.t, label: "outside cone" });
    const end = rows[rows.length - 1];
    if (Math.abs(end.downrange) > 1.2 || (end.height < 0.5 && Math.abs(end.rate) > 0.8)) {
      markers.push({ t: end.t, label: "miss" });
    }
    return markers;
  }
  if (id === "desMiss") {
    const rows = rowsFor(id, knobs);
    const markers: ChartMarker[] = [];
    if (knobs.cause < 0.5) {
      const empty = rows.find((row, index) => index > 0 && row.reference <= 1e-6 && rows[index - 1].reference > 1e-6);
      if (empty) markers.push({ t: empty.t, label: "fuel out" });
    }
    const touch = rows.find((row) => row.height <= 1e-3);
    const outcome = missOutcome(knobs);
    if (touch) {
      const label =
        outcome.status === "infeasible" ? "INFEASIBLE" : outcome.status === "soft" ? "soft" : "miss";
      markers.push({ t: touch.t, label });
    }
    return markers;
  }
  if (id === "loopSample") {
    const markers: ChartMarker[] = [{ t: 1, label: "target step" }];
    for (let t = knobs.dt; t < durationOf(id) - 1e-9; t += knobs.dt) {
      markers.push({ t, label: "sample" });
      if (markers.length >= 9) break;
    }
    return markers;
  }
  return [];
}

function allowedList(id: SceneId): string {
  return chartColumns(id).join(", ");
}

export function chatBrief(id: SceneId, knobs: Knobs): string {
  const scene = sceneById(id);
  if (id === "state") {
    return `${scene.title}. Undamped spring, mass ${SPRING_M} kg, stiffness ${SPRING_K} N/m. Start height x0 = ${knobs.x0.toFixed(2)} m, start rate v0 = ${knobs.v0.toFixed(2)} m/s. Height is x0 cos(t) + v0 sin(t). Rate is −x0 sin(t) + v0 cos(t). Acceleration is −height. Drag sets x0 (keeps the current rate as v0 and resets time). The live chart is a phase loop of rate against height. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "step") {
    const method = knobs.method >= 0.5 ? "semi-implicit Euler (rate then height)" : "explicit Euler (both updates from the old state)";
    return `${scene.title}. True height is cos(t). Stepped height uses ${method} with dt = ${knobs.dt.toFixed(2)} s on a unit spring (m=k=1). Explicit Euler grows mechanical energy and the phase orbit spirals out; semi-implicit stays nearer the true ellipse. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "mass") {
    return `${scene.title}. Thrust input is ${knobs.thrust.toFixed(0)} N until t = ${BURN} s, then zero. Dry mass ${DRY} kg, fuel ${FUEL} kg. Height and rate integrate acceleration. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "engine") {
    const stepAt = engineStepAt(knobs.tau, knobs.lead);
    const leadNote =
      knobs.lead >= 0.5
        ? `Lead is on: command steps at ${stepAt.toFixed(2)} s (one τ early) so actual thrust is farther along by the need mark at ${ASK_AT} s.`
        : `Lead is off: command steps at the need time ${ASK_AT} s.`;
    return `${scene.title}. Command steps to ${knobs.ask.toFixed(0)} N. ${leadNote} Lag τ = ${knobs.tau.toFixed(2)} s. Thrust is clamped to ${LIMIT} N on a 2.5 kg vehicle. Height and rate integrate actual thrust. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "loopOpen") {
    return `${scene.title}. Hover thrust is ${LOOP_T0.toFixed(1)} N on a ${LOOP_M} kg rocket. Extra thrust ${knobs.extraThrust.toFixed(0)} N after 1 s with no feedback. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "loopError") {
    return `${scene.title}. Target steps to 2 m at 2 s. Gains Kp=${knobs.kp}, Ki=${knobs.ki}, Kd=${knobs.kd} for the three single-mode runs. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "loopEquil") {
    return `${scene.title}. Start offset ${knobs.perturb.toFixed(2)} m with closed-loop hover law. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "loopHover") {
    const mu2 = knobs.unstable * knobs.unstable;
    return `${scene.title}. Open loop ẍ = μ² x with μ = ${knobs.unstable.toFixed(1)} (μ² = ${mu2.toFixed(2)}). Closed loop adds (Kp e − Kd ẋ)/m with Kp = ${LOOP_HOVER_KP}, Kd = ${LOOP_HOVER_KD}; needs Kp/m > μ² (${(LOOP_HOVER_KP / LOOP_M).toFixed(2)} > ${mu2.toFixed(2)}). Pad clamp at height 0. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "loopMode") {
    return `${scene.title}. Hidden mode q̈ = λ² q with λ = ${LOOP_HIDDEN_LAMBDA}, coupling ε = ${knobs.coupling.toFixed(2)} into height. Measured height can look calm while q grows. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (id === "loopSample") {
    return `${scene.title}. Sample time ${knobs.dt.toFixed(2)} s and measurement delay ${knobs.tau.toFixed(2)} s versus continuous PID (Kp 12, Ki 4, Kd 15). Delay near ${LOOP_SAMPLE_DELAY_MARK.toFixed(2)} s starts to pull discrete away from continuous. Allowed chart columns: ${allowedList(id)}.`;
  }
  if (isTurnScene(id)) return turnBrief(id, knobs);
  if (isVehicleScene(id)) return vehicleBrief(id, knobs);
  if (isDescentScene(id)) return `${descentBrief(id, knobs)} Allowed chart columns: ${allowedList(id)}.`;
  return `${scene.title}. Allowed chart columns: ${allowedList(id)}.`;
}
