import * as Tooltip from "@radix-ui/react-tooltip";
import { Info, Pause, Play } from "lucide-react";
import { useEffect, useRef, useState, type CSSProperties, type KeyboardEvent as ReactKeyboardEvent, type PointerEvent as ReactPointerEvent, type ReactElement } from "react";
import { ChartView } from "../ChartView";
import { FlightView } from "../FlightView";
import {
  KNOBS,
  NOMINAL_CASE,
  formatKnob,
  readKnob,
  sameCase,
  withController,
  withDivert,
  withKnob,
  withWorld,
  type ControllerName,
  type FlightCase,
  type KnobKey,
  type KnobSpec,
  type WorldName,
} from "../flight/case";
import { FLIGHT_SIGNALS, flightTrace, signalSpec, type FlightSignal } from "../flight/chart";
import type { EChartsOption } from "echarts";
import { endTime, parseLines, parseSample, sampleAt, type Sample } from "../flight/samples";

const TAPES: { key: KnobKey; span: number; tick: number }[] = [
  { key: "height", span: 200, tick: 40 },
  { key: "descent", span: 40, tick: 5 },
];

const START: KnobKey[] = ["offset", "fuel"];
const CUSTOM_START: KnobKey[] = ["offset", "cross", "along", "across", "fuel"];
type StartMode = "drop" | "divert" | "custom";
const SEA: KnobKey[] = ["wind", "pad", "gust"];
const LIMITS: KnobKey[] = ["floor", "ceiling", "cone", "glide", "lag", "gimbal", "inertia"];

const WORLDS: { id: WorldName; label: string; tip: string }[] = [
  { id: "earth", label: "Earth", tip: "Air and wind, with a barge that can drift. Gravity is 9.81 m/s²." },
  { id: "moon", label: "Moon", tip: "No air and no wind. Gravity is 1.62 m/s², and the pad stays put." },
];

const CONTROLLERS: { id: ControllerName; label: string; line: string; tip: string }[] = [
  {
    id: "pid",
    label: "PID",
    line: "PID pushes back on the error in height and position.",
    tip: "Pushes back on the error in height and position. It does not plan the whole burn ahead.",
  },
  {
    id: "mpc",
    label: "MPC",
    line: "MPC recomputes a path to the pad inside the thrust limits.",
    tip: "Recomputes a path to the pad and keeps the thrust between the floor and the ceiling.",
  },
  {
    id: "scvx",
    label: "SCvx",
    line: "SCvx chooses a path that soft-lands on less fuel.",
    tip: "Searches for a path that soft-lands on less fuel. The search is successive convexification.",
  },
];

const GROUP_TIPS = {
  environment: "Earth has air, wind, and a barge that can drift. The Moon has no air, gravity of 1.62 m/s², and a pad that stays put.",
  controller: "How the ship steers the landing burn. All three use the same engines and the same limits.",
  start: "Where the burn starts. Drop is over the pad. Divert starts off to the side. Custom keeps the numbers you set.",
  sea: "A steady wind, a short gust, and a barge that drifts. The Moon has none of these.",
  limits: "What the engine and the guidance are allowed to do during the burn.",
};

const KNOB_TIPS: Record<KnobKey, string> = {
  height: "Height above the ground when the burn starts.",
  descent: "Vertical speed at the start. A negative rate is a fall.",
  offset: "Distance ahead of the pad, along the approach, when the burn starts.",
  cross: "Distance to the side of the pad when the burn starts. Divert uses −40 m.",
  along: "Sideways speed along the approach at the start. Drop and Divert use 0.",
  across: "Sideways speed across the approach at the start. Drop and Divert use 0.",
  fuel: "Propellant on board at the start. The burn stops when this is gone or the ship is down.",
  wind: "A steady sideways wind during the burn.",
  pad: "How fast the landing barge drifts. The ship has to meet a moving deck.",
  gust: "A short extra shove of wind on top of the steady wind.",
  gain: "How hard PID pushes back when height or position is off. Used only with PID.",
  floor: "The least thrust the engine can make while it is lit. Below this it shuts off.",
  ceiling: "The most thrust the engine can make.",
  cone: "How far the thrust may lean off vertical. That lean is the pointing cone.",
  glide: "How far the ship may fly sideways for the height it has left. A tight glide cannot reach a far divert.",
  lag: "How long the engine takes to reach a commanded thrust.",
  gimbal: "How far the engine bell can tilt to steer the ship.",
  inertia: "How hard the ship is to rotate. A larger number turns more slowly.",
};

// 400 ms confirms the pointer stopped, so a pass across the panel does not flash.
// 300 ms is the window where the next tip opens at once.
const TIP_DELAY_MS = 400;
const TIP_SKIP_MS = 300;

const SIDE_MIN = 260;
const SIDE_DEFAULT = 480;
const PLOTS_MIN = 300;
const PLOTS_DEFAULT = 420;
const PICTURE_MIN = 180;

function formatTime(time: number): string {
  return `${time.toFixed(1)} s`;
}

function formatMeters(value: number): string {
  return `${value.toFixed(0)} m`;
}

function formatRate(value: number): string {
  return `${value.toFixed(1)} m/s`;
}

function touchdownOf(samples: Sample[]): { miss: string; speed: string; burned: string } | null {
  if (samples.length === 0) return null;
  const first = samples[0];
  const last = samples[samples.length - 1];
  const miss = Math.hypot(last.x - last.padX, last.y - last.padY);
  const burned = Math.max(0, first.fuel - last.fuel);
  return {
    miss: miss.toFixed(1),
    speed: Math.abs(last.vz).toFixed(2),
    burned: burned.toFixed(0),
  };
}

function snap(value: number, min: number, max: number, step: number): number {
  const steps = Math.round((value - min) / step);
  const next = min + steps * step;
  if (next < min) return min;
  if (next > max) return max;
  return next;
}

function specFor(key: KnobKey): KnobSpec {
  const spec = KNOBS.find((item) => item.key === key);
  if (!spec) throw new Error(`Missing knob ${key}`);
  return spec;
}

function useFigureDrag(
  spec: KnobSpec,
  value: number,
  range: number,
  pixels: number,
  onScrub: (next: number) => void,
  onCommit: () => void,
) {
  const drag = useRef<{ y: number; value: number } | null>(null);
  return {
    onPointerDown(event: ReactPointerEvent<HTMLDivElement>) {
      if (event.button !== 0) return;
      drag.current = { y: event.clientY, value };
      event.currentTarget.setPointerCapture(event.pointerId);
    },
    onPointerMove(event: ReactPointerEvent<HTMLDivElement>) {
      if (!drag.current) return;
      const dy = drag.current.y - event.clientY;
      onScrub(snap(drag.current.value + dy * (range / pixels), spec.min, spec.max, spec.step));
    },
    onPointerUp() {
      if (!drag.current) return;
      drag.current = null;
      onCommit();
    },
    onKeyDown(event: ReactKeyboardEvent<HTMLDivElement>) {
      if (event.key === "ArrowUp") onScrub(snap(value + spec.step, spec.min, spec.max, spec.step));
      if (event.key === "ArrowDown") onScrub(snap(value - spec.step, spec.min, spec.max, spec.step));
    },
    onKeyUp(event: ReactKeyboardEvent<HTMLDivElement>) {
      if (event.key === "ArrowUp" || event.key === "ArrowDown") onCommit();
    },
  };
}

function Tip({
  text,
  side,
  children,
}: {
  text: string;
  side: "top" | "right" | "bottom";
  children: ReactElement;
}) {
  return (
    <Tooltip.Root>
      <Tooltip.Trigger asChild>{children}</Tooltip.Trigger>
      <Tooltip.Portal>
        <Tooltip.Content className="tip sim-tip" side={side} align="center" sideOffset={8} collisionPadding={12}>
          <p>{text}</p>
          <Tooltip.Arrow className="tip-arrow" width={10} height={6} />
        </Tooltip.Content>
      </Tooltip.Portal>
    </Tooltip.Root>
  );
}

function InfoMark({ label, text }: { label: string; text: string }) {
  return (
    <Tip text={text} side="right">
      <button
        type="button"
        className="info"
        aria-label={`About ${label}`}
        onPointerDown={(event) => event.stopPropagation()}
        onClick={(event) => event.stopPropagation()}
      >
        <Info size={13} strokeWidth={1.75} aria-hidden />
      </button>
    </Tip>
  );
}

function GroupTitle({ title, text }: { title: string; text: string }) {
  return (
    <div className="sim-group-head">
      <h2 className="sim-group-title">{title}</h2>
      <InfoMark label={title} text={text} />
    </div>
  );
}

function Choice({
  checked,
  label,
  tip,
  onChoose,
}: {
  checked: boolean;
  label: string;
  tip: string;
  onChoose: () => void;
}) {
  return (
    <Tip text={tip} side="bottom">
      <button type="button" role="radio" aria-checked={checked} onClick={onChoose}>
        {label}
      </button>
    </Tip>
  );
}

function SignalPlot({
  label,
  signal,
  names,
  unit,
  onSignal,
  option,
  onScrub,
}: {
  label: string;
  signal?: FlightSignal;
  names?: { label: string; color: string }[];
  unit: string;
  onSignal?: (next: FlightSignal) => void;
  option: EChartsOption;
  onScrub: (time: number) => void;
}) {
  return (
    <div className="sim-plot">
      <div className="sim-plot-head">
        {signal && onSignal ? (
          <label className="sim-signal">
            <select aria-label={label} value={signal} onChange={(event) => onSignal(event.target.value as FlightSignal)}>
              {FLIGHT_SIGNALS.map((item) => (
                <option key={item.id} value={item.id}>
                  {item.label}
                </option>
              ))}
            </select>
          </label>
        ) : (
          <span className="sim-plot-names">
            {(names ?? []).map((item) => (
              <span key={item.label} className="sim-plot-key">
                <i className="sim-plot-swatch" style={{ background: item.color }} />
                {item.label}
              </span>
            ))}
          </span>
        )}
        <span className="sim-plot-unit">{unit}</span>
      </div>
      <ChartView option={option} onScrub={onScrub} />
    </div>
  );
}

function Tape({
  spec,
  value,
  span,
  tick,
  onScrub,
  onCommit,
}: {
  spec: KnobSpec;
  value: number;
  span: number;
  tick: number;
  onScrub: (next: number) => void;
  onCommit: () => void;
}) {
  const handlers = useFigureDrag(spec, value, span, 148, onScrub, onCommit);
  const marks: number[] = [];
  const start = Math.ceil((value - span / 2) / tick) * tick;
  for (let mark = start; mark <= value + span / 2 + 1e-6; mark += tick) {
    marks.push(Number(mark.toFixed(3)));
  }
  const shown = formatKnob(spec.key, value).replace(` ${spec.unit}`, "");

  return (
    <div
      className="sim-tape"
      role="slider"
      tabIndex={0}
      aria-label={spec.label}
      aria-valuemin={spec.min}
      aria-valuemax={spec.max}
      aria-valuenow={value}
      aria-valuetext={formatKnob(spec.key, value)}
      {...handlers}
    >
      {marks.map((mark) => {
        const y = 50 - ((mark - value) / span) * 100;
        return (
          <span key={mark} className="sim-tick" style={{ top: `${y}%` }}>
            <i />
            {mark.toFixed(0)}
          </span>
        );
      })}
      <span className="sim-tape-info">
        <InfoMark label={spec.label} text={KNOB_TIPS[spec.key]} />
      </span>
      <span className="sim-tape-read">
        <span className="sim-tape-label">{spec.label}</span>
        <span className="sim-tape-value">{shown}</span>
        <span className="sim-tape-unit">{spec.unit}</span>
      </span>
    </div>
  );
}

function Figure({
  spec,
  value,
  onScrub,
  onCommit,
}: {
  spec: KnobSpec;
  value: number;
  onScrub: (next: number) => void;
  onCommit: () => void;
}) {
  const handlers = useFigureDrag(spec, value, spec.max - spec.min, 160, onScrub, onCommit);
  const shown = formatKnob(spec.key, value).replace(` ${spec.unit}`, "");

  return (
    <div
      className="sim-figure"
      role="slider"
      tabIndex={0}
      aria-label={spec.label}
      aria-valuemin={spec.min}
      aria-valuemax={spec.max}
      aria-valuenow={value}
      aria-valuetext={formatKnob(spec.key, value)}
      {...handlers}
    >
      <span className="sim-figure-head">
        <span className="sim-figure-label">{spec.label}</span>
        <InfoMark label={spec.label} text={KNOB_TIPS[spec.key]} />
      </span>
      <span className="sim-figure-value">
        {shown}
        {spec.unit ? <span className="sim-figure-unit">{spec.unit}</span> : null}
      </span>
    </div>
  );
}

export function SimulatorPage() {
  const [flight, setFlight] = useState<FlightCase>(NOMINAL_CASE);
  const [samples, setSamples] = useState<Sample[]>([]);
  const [time, setTime] = useState(0);
  const [playing, setPlaying] = useState(true);
  const [done, setDone] = useState(false);
  const [note, setNote] = useState("");
  const [sideWidth, setSideWidth] = useState(SIDE_DEFAULT);
  const [plotsWidth, setPlotsWidth] = useState(PLOTS_DEFAULT);
  const [world, setWorld] = useState<WorldName>("earth");
  const [limitsOpen, setLimitsOpen] = useState(true);
  const [startMode, setStartMode] = useState<StartMode>("custom");
  const [signals, setSignals] = useState<FlightSignal[]>(["thrust", "rate"]);

  const samplesRef = useRef<Sample[]>([]);
  const playingRef = useRef(true);
  const doneRef = useRef(false);
  const draftRef = useRef<FlightCase>(NOMINAL_CASE);
  const flownRef = useRef<FlightCase>(NOMINAL_CASE);
  const abortRef = useRef<AbortController | null>(null);
  const generation = useRef(0);
  const requested = useRef(false);
  const timeRef = useRef(0);
  const scrubbing = useRef(false);

  playingRef.current = playing;
  doneRef.current = done;
  timeRef.current = time;

  function keep(next: Sample[]) {
    samplesRef.current = next;
    setSamples(next);
  }

  useEffect(() => {
    let cancel = false;
    fetch("/flights/nominal.ndjson")
      .then((response) => response.text())
      .then((text) => {
        if (cancel) return;
        if (requested.current) return;
        const stored = parseLines(text);
        if (stored.length === 0) return;
        keep(stored);
        setDone(true);
        timeRef.current = 0;
        setTime(0);
        setPlaying(true);
      })
      .catch(() => {
        if (!cancel) setNote("The saved landing did not load.");
      });
    return () => {
      cancel = true;
    };
  }, []);

  useEffect(() => {
    let frame = 0;
    let last = performance.now();
    const tick = (now: number) => {
      const elapsed = Math.min(0.05, (now - last) / 1000);
      last = now;
      if (playingRef.current && !scrubbing.current) {
        const list = samplesRef.current;
        const end = endTime(list);
        if (list.length > 0) {
          const next = Math.min(timeRef.current + elapsed, end);
          timeRef.current = next;
          if (next >= end && doneRef.current) {
            playingRef.current = false;
            setPlaying(false);
          }
          setTime(next);
        }
      }
      frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(frame);
  }, []);

  useEffect(() => {
    return () => abortRef.current?.abort();
  }, []);

  async function fly(next: FlightCase) {
    abortRef.current?.abort();
    const controller = new AbortController();
    abortRef.current = controller;
    requested.current = true;
    const ticket = generation.current + 1;
    generation.current = ticket;
    flownRef.current = next;
    keep([]);
    timeRef.current = 0;
    setTime(0);
    setDone(false);
    setPlaying(true);
    playingRef.current = true;
    doneRef.current = false;
    setNote("");

    let response: Response;
    try {
      const session = await fetch("/api/session", { signal: controller.signal });
      if (!session.ok) throw new Error("session");
      response = await fetch("/api/flight", {
        method: "POST",
        headers: { "content-type": "application/json" },
        body: JSON.stringify(next),
        signal: controller.signal,
      });
    } catch {
      if (ticket !== generation.current) return;
      setNote("This computer cannot start a new flight.");
      setDone(true);
      return;
    }

    if (ticket !== generation.current) return;
    if (!response.ok || !response.body) {
      setNote("This computer cannot start a new flight.");
      setDone(true);
      return;
    }

    const reader = response.body.getReader();
    const decoder = new TextDecoder();
    let pending = "";
    const gathered: Sample[] = [];

    try {
      while (true) {
        const chunk = await reader.read();
        if (ticket !== generation.current) return;
        if (chunk.done) break;
        pending += decoder.decode(chunk.value, { stream: true });
        const lines = pending.split("\n");
        pending = lines.pop() ?? "";
        let added = false;
        for (const line of lines) {
          const sample = parseSample(line);
          if (!sample) continue;
          gathered.push(sample);
          added = true;
        }
        if (added) keep(gathered.slice());
      }
    } catch {
      if (ticket !== generation.current) return;
      if (gathered.length === 0) setNote("This computer cannot start a new flight.");
    }

    if (ticket !== generation.current) return;
    const tail = parseSample(pending);
    if (tail) {
      gathered.push(tail);
      keep(gathered.slice());
    }
    setDone(true);
    doneRef.current = true;
  }

  function settle(next: FlightCase) {
    setFlight(next);
    if (sameCase(next, flownRef.current)) return;
    void fly(next);
  }

  function onKnob(key: KnobKey, value: number) {
    const next = withKnob(draftRef.current, key, value);
    draftRef.current = next;
    setFlight(next);
    if (key === "offset" || key === "cross" || key === "along" || key === "across") setStartMode("custom");
  }

  function release() {
    settle(draftRef.current);
  }

  function chooseWorld(nextWorld: WorldName) {
    let next = withWorld(draftRef.current, nextWorld);
    if (startMode === "divert") next = withDivert(next, true);
    else if (startMode === "drop") next = withDivert(next, false);
    draftRef.current = next;
    setWorld(nextWorld);
    settle(next);
  }

  function chooseDivert(on: boolean) {
    const next = withDivert(draftRef.current, on);
    draftRef.current = next;
    setStartMode(on ? "divert" : "drop");
    settle(next);
  }

  function chooseCustom() {
    setStartMode("custom");
  }

  function chooseController(nextController: ControllerName) {
    const next = withController(draftRef.current, nextController);
    draftRef.current = next;
    settle(next);
  }

  const shown = sampleAt(samples, time);
  const duration = Math.max(endTime(samples), 0.01);
  const bands = {
    floorKn: flight.simulation.thrust_min / 1000,
    ceilingKn: flight.simulation.thrust_max / 1000,
    gimbalDeg: flight.simulation.gimbal_max_deg,
  };
  const pathOption = flightTrace(samples, time, ["height", "range"], bands, false);
  const plotOptions = signals.map((signal, index) =>
    flightTrace(samples, time, signal, bands, index === signals.length - 1),
  );
  const controller = flight.simulation.controller;
  const controllerLine = CONTROLLERS.find((item) => item.id === controller)?.line ?? CONTROLLERS[1].line;
  const landed = done && samples.length > 0 && time >= endTime(samples) - 0.05;
  const score = landed ? touchdownOf(samples) : null;
  const startKeys = startMode === "custom" ? CUSTOM_START : START;
  const startLine =
    startMode === "divert"
      ? "70 m by 40 m off the pad."
      : startMode === "drop"
        ? "Over the pad."
        : "Crossrange and sideways speed stay as you set them.";
  const limitKeys: KnobKey[] = controller === "pid" ? ["gain", ...LIMITS] : LIMITS;

  function beginScrub() {
    scrubbing.current = true;
    playingRef.current = false;
    setPlaying(false);
  }

  function endScrub() {
    scrubbing.current = false;
  }

  // The slider also emits a change when its range grows, as the saved landing
  // arrives. That is not a drag. Only a pointer or a key should move the clock.
  function scrub(next: number) {
    if (!scrubbing.current) return;
    const end = endTime(samplesRef.current);
    const placed = Math.min(Math.max(0, next), end);
    timeRef.current = placed;
    playingRef.current = false;
    setPlaying(false);
    setTime(placed);
  }

  function onResizeDown(event: ReactPointerEvent<HTMLDivElement>) {
    if (event.button !== 0) return;
    const startX = event.clientX;
    const startWidth = sideWidth;
    const room = window.innerWidth - plotsWidth - PICTURE_MIN;
    event.currentTarget.setPointerCapture(event.pointerId);
    function move(next: PointerEvent) {
      const width = startWidth + (next.clientX - startX);
      setSideWidth(Math.min(room, Math.max(SIDE_MIN, width)));
    }
    function up() {
      window.removeEventListener("pointermove", move);
      window.removeEventListener("pointerup", up);
    }
    window.addEventListener("pointermove", move);
    window.addEventListener("pointerup", up);
  }

  function onPlotsResizeDown(event: ReactPointerEvent<HTMLDivElement>) {
    if (event.button !== 0) return;
    const startX = event.clientX;
    const startWidth = plotsWidth;
    const room = window.innerWidth - sideWidth - PICTURE_MIN;
    event.currentTarget.setPointerCapture(event.pointerId);
    function move(next: PointerEvent) {
      const width = startWidth + (startX - next.clientX);
      setPlotsWidth(Math.min(room, Math.max(PLOTS_MIN, width)));
    }
    function up() {
      window.removeEventListener("pointermove", move);
      window.removeEventListener("pointerup", up);
    }
    window.addEventListener("pointermove", move);
    window.addEventListener("pointerup", up);
  }

  function setSignal(index: number, next: FlightSignal) {
    setSignals((current) => current.map((item, itemIndex) => (itemIndex === index ? next : item)));
  }

  const shellStyle = {
    "--sim-side": `${sideWidth}px`,
    "--sim-plots": `${plotsWidth}px`,
  } as CSSProperties;

  return (
    <Tooltip.Provider delayDuration={TIP_DELAY_MS} skipDelayDuration={TIP_SKIP_MS}>
    <div className="frame is-sim">
      <div className="sim-shell" style={shellStyle}>
        <div className={world === "moon" ? "sim-view is-moon" : "sim-view"}>
          <FlightView
            sample={shown}
            samples={samples}
            time={time}
            world={world}
            windX={flight.simulation.wind_x_velocity}
            windY={flight.simulation.wind_y_velocity}
            gust={flight.simulation.gust_magnitude}
            gustHz={flight.simulation.gust_frequency}
            thrustFull={flight.simulation.thrust_max}
          />
          <div className="sim-readout">
            <span>ALT {formatMeters(shown?.z ?? 0)}</span>
            <span>VZ {formatRate(shown?.vz ?? 0)}</span>
            <span>FUEL {(shown?.fuel ?? 0).toFixed(0)} kg</span>
            {score ? (
              <>
                <span>MISS {score.miss} m</span>
                <span>TOUCH {score.speed} m/s</span>
                <span>BURNED {score.burned} kg</span>
              </>
            ) : null}
          </div>
          <p className="sim-credit">
            Ship model:{" "}
            <a href="https://sketchfab.com/3d-models/spacex-starship-sn24-superheavy-bn7-cb1b096d64b443ef9f87dd0cf960cf47">
              SpaceX Starship SN24 Superheavy BN7
            </a>{" "}
            by <a href="https://sketchfab.com/clarence365">Clarence365</a>,{" "}
            <a href="https://creativecommons.org/licenses/by/4.0/">CC BY 4.0</a>
            {world === "moon" ? (
              <>
                {" "}
                Ground:{" "}
                <a href="https://commons.wikimedia.org/wiki/File:Apollo_Basin-_Mare_in_a_Sea_of_Highlands_(LROCM114953774LE_thumb).png">
                  Apollo Basin floor
                </a>
                , LROC, NASA/GSFC/Arizona State University.
              </>
            ) : null}
          </p>
        </div>
        <aside className="sim-side" style={{ width: sideWidth }}>
          <div
            className="sim-resize"
            role="separator"
            aria-orientation="vertical"
            aria-label="Console width"
            onPointerDown={onResizeDown}
          />
          <div className="sim-console">
            <section>
              <GroupTitle title="Environment" text={GROUP_TIPS.environment} />
              <div className="sim-choice" role="radiogroup" aria-label="Environment">
                {WORLDS.map((item) => (
                  <Choice
                    key={item.id}
                    checked={world === item.id}
                    label={item.label}
                    tip={item.tip}
                    onChoose={() => chooseWorld(item.id)}
                  />
                ))}
              </div>
            </section>
            <section>
              <GroupTitle title="Controller" text={GROUP_TIPS.controller} />
              <div className="sim-choice" role="radiogroup" aria-label="Controller">
                {CONTROLLERS.map((item) => (
                  <Choice
                    key={item.id}
                    checked={controller === item.id}
                    label={item.label}
                    tip={item.tip}
                    onChoose={() => chooseController(item.id)}
                  />
                ))}
              </div>
              <p className="sim-lead">{controllerLine}</p>
            </section>
            <section>
              <GroupTitle title="Initial conditions" text={GROUP_TIPS.start} />
              <div className="sim-choice" role="radiogroup" aria-label="Start">
                <Choice checked={startMode === "drop"} label="Drop" tip="Starts above the pad, with no sideways speed." onChoose={() => chooseDivert(false)} />
                <Choice
                  checked={startMode === "divert"}
                  label="Divert"
                  tip="Starts 70 m to the side and 40 m downrange, with no sideways speed."
                  onChoose={() => chooseDivert(true)}
                />
                <Choice
                  checked={startMode === "custom"}
                  label="Custom"
                  tip="Neither preset. Crossrange and sideways speed stay on the numbers you set."
                  onChoose={chooseCustom}
                />
              </div>
              <p className="sim-lead">{startLine}</p>
              <div className="sim-tapes">
                {TAPES.map((tape) => (
                  <Tape
                    key={tape.key}
                    spec={specFor(tape.key)}
                    value={readKnob(flight, tape.key)}
                    span={tape.span}
                    tick={tape.tick}
                    onScrub={(next) => onKnob(tape.key, next)}
                    onCommit={release}
                  />
                ))}
              </div>
              <div className="sim-figures">
                {startKeys.map((key) => (
                  <Figure
                    key={key}
                    spec={specFor(key)}
                    value={readKnob(flight, key)}
                    onScrub={(next) => onKnob(key, next)}
                    onCommit={release}
                  />
                ))}
              </div>
            </section>
            {world === "earth" ? (
              <section>
                <GroupTitle title="Sea" text={GROUP_TIPS.sea} />
                <div className="sim-figures">
                  {SEA.map((key) => (
                    <Figure
                      key={key}
                      spec={specFor(key)}
                      value={readKnob(flight, key)}
                      onScrub={(next) => onKnob(key, next)}
                      onCommit={release}
                    />
                  ))}
                </div>
              </section>
            ) : (
              <p className="sim-lead sim-still">Still air. The pad stays put.</p>
            )}
            <details className="sim-limits" open={limitsOpen} onToggle={(event) => setLimitsOpen(event.currentTarget.open)}>
              <summary>
                <span>Limits</span>
                <InfoMark label="Limits" text={GROUP_TIPS.limits} />
              </summary>
              <div className="sim-figures">
                {limitKeys.map((key) => {
                  const spec = specFor(key);
                  const locked = world === "moon" && (key === "floor" || key === "ceiling");
                  if (locked) {
                    const raw = key === "floor" ? flight.simulation.thrust_min / 1000 : flight.simulation.thrust_max / 1000;
                    return (
                      <div key={key} className="sim-figure sim-metric">
                        <span className="sim-figure-head">
                          <span className="sim-figure-label">{spec.label}</span>
                          <InfoMark label={spec.label} text={KNOB_TIPS[key]} />
                        </span>
                        <span className="sim-figure-value">
                          {raw.toFixed(2)}
                          <span className="sim-figure-unit">{spec.unit}</span>
                        </span>
                      </div>
                    );
                  }
                  return (
                    <Figure
                      key={key}
                      spec={spec}
                      value={readKnob(flight, key)}
                      onScrub={(next) => onKnob(key, next)}
                      onCommit={release}
                    />
                  );
                })}
              </div>
            </details>
            {note ? <p className="sim-note">{note}</p> : null}
          </div>
        </aside>
        <div className="sim-strip">
          <div className="sim-clock">
            <button
              type="button"
              className="play sim-play"
              aria-label={playing ? "Pause" : "Play"}
              aria-pressed={playing}
              onClick={() => {
                if (!playing && time >= endTime(samples) - 1e-3 && done) {
                  timeRef.current = 0;
                  setTime(0);
                  setPlaying(true);
                  playingRef.current = true;
                  return;
                }
                playingRef.current = !playing;
                setPlaying((value) => !value);
              }}
            >
              {playing ? <Pause size={18} strokeWidth={2.25} /> : <Play size={18} strokeWidth={2.25} />}
            </button>
            <label className="sim-clock-label" htmlFor="flight-time">
              T+ {formatTime(time)}
            </label>
            <input
              id="flight-time"
              className="sim-scrub"
              type="range"
              min={0}
              max={duration}
              step={0.01}
              value={Math.min(time, duration)}
              onPointerDown={beginScrub}
              onPointerUp={endScrub}
              onPointerCancel={endScrub}
              onKeyDown={(event) => {
                if (event.key.startsWith("Arrow")) beginScrub();
              }}
              onKeyUp={(event) => {
                if (event.key.startsWith("Arrow")) endScrub();
              }}
              onChange={(event) => scrub(Number(event.target.value))}
            />
          </div>
        </div>
        <aside className="sim-plots-panel" style={{ width: plotsWidth }} aria-label="Plots">
          <div
            className="sim-plots-resize"
            role="separator"
            aria-orientation="vertical"
            aria-label="Plots width"
            onPointerDown={onPlotsResizeDown}
          />
          <div className="sim-plots">
            <SignalPlot
              label="Altitude and range"
              names={[signalSpec("height"), signalSpec("range")]}
              unit="m"
              option={pathOption}
              onScrub={scrub}
            />
            {signals.map((signal, index) => (
              <SignalPlot
                key={index}
                label={`Plot ${index + 2}`}
                signal={signal}
                unit={signalSpec(signal).unit}
                onSignal={(next) => setSignal(index, next)}
                option={plotOptions[index]}
                onScrub={scrub}
              />
            ))}
          </div>
        </aside>
      </div>
    </div>
    </Tooltip.Provider>
  );
}
