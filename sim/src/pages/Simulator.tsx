import { Pause, Play } from "lucide-react";
import { useEffect, useRef, useState, type CSSProperties, type KeyboardEvent as ReactKeyboardEvent, type PointerEvent as ReactPointerEvent } from "react";
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
  divertOn,
  dropOn,
  type ControllerName,
  type FlightCase,
  type KnobKey,
  type KnobSpec,
  type WorldName,
} from "../flight/case";
import { FLIGHT_SIGNALS, flightTrace, type FlightSignal } from "../flight/chart";
import type { EChartsOption } from "echarts";
import { endTime, parseLines, parseSample, sampleAt, type Sample } from "../flight/samples";

const TAPES: { key: KnobKey; span: number; tick: number }[] = [
  { key: "height", span: 200, tick: 40 },
  { key: "descent", span: 40, tick: 5 },
];

const START: KnobKey[] = ["offset", "fuel"];
const SEA: KnobKey[] = ["wind", "pad", "gust"];
const LIMITS: KnobKey[] = ["floor", "ceiling", "cone", "glide", "lag", "gimbal", "inertia"];

const WORLDS: { id: WorldName; label: string }[] = [
  { id: "earth", label: "Earth" },
  { id: "moon", label: "Moon" },
];

const CONTROLLERS: { id: ControllerName; label: string; line: string }[] = [
  { id: "pid", label: "PID", line: "PID pushes back on the error in height and position." },
  { id: "mpc", label: "MPC", line: "MPC recomputes a path to the pad inside the thrust limits." },
  { id: "scvx", label: "SCvx", line: "SCvx chooses a path that soft-lands on less fuel." },
];

const SIDE_MIN = 260;
const SIDE_DEFAULT = 480;

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

function SignalPlot({
  label,
  signal,
  onSignal,
  option,
  onScrub,
}: {
  label: string;
  signal: FlightSignal;
  onSignal: (next: FlightSignal) => void;
  option: EChartsOption;
  onScrub: (time: number) => void;
}) {
  return (
    <div className="sim-plot">
      <label className="sim-signal">
        <select aria-label={label} value={signal} onChange={(event) => onSignal(event.target.value as FlightSignal)}>
          {FLIGHT_SIGNALS.map((item) => (
            <option key={item.id} value={item.id}>
              {item.label}
            </option>
          ))}
        </select>
      </label>
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
      <span className="sim-figure-label">{spec.label}</span>
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
  const [world, setWorld] = useState<WorldName>("earth");
  const [leftSignal, setLeftSignal] = useState<FlightSignal>("height");
  const [rightSignal, setRightSignal] = useState<FlightSignal>("rate");

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
  }

  function release() {
    settle(draftRef.current);
  }

  function chooseWorld(nextWorld: WorldName) {
    const current = draftRef.current;
    const useDivert = divertOn(current) || !dropOn(current);
    const next = withDivert(withWorld(current, nextWorld), useDivert);
    draftRef.current = next;
    setWorld(nextWorld);
    settle(next);
  }

  function chooseDivert(on: boolean) {
    const next = withDivert(draftRef.current, on);
    draftRef.current = next;
    settle(next);
  }

  function chooseController(nextController: ControllerName) {
    const next = withController(draftRef.current, nextController);
    draftRef.current = next;
    settle(next);
  }

  const shown = sampleAt(samples, time);
  const duration = Math.max(endTime(samples), 0.01);
  const altitude = flightTrace(samples, time, leftSignal);
  const rate = flightTrace(samples, time, rightSignal);
  const controller = flight.simulation.controller;
  const controllerLine = CONTROLLERS.find((item) => item.id === controller)?.line ?? CONTROLLERS[1].line;
  const landed = done && samples.length > 0 && time >= endTime(samples) - 0.05;
  const score = landed ? touchdownOf(samples) : null;
  const divert = divertOn(flight);
  const drop = dropOn(flight);
  const startLine = divert ? "70 m by 40 m off the pad." : drop ? "Over the pad." : "";
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
    event.currentTarget.setPointerCapture(event.pointerId);
    function move(next: PointerEvent) {
      const max = window.innerWidth - 120;
      const width = startWidth + (next.clientX - startX);
      setSideWidth(Math.min(max, Math.max(SIDE_MIN, width)));
    }
    function up() {
      window.removeEventListener("pointermove", move);
      window.removeEventListener("pointerup", up);
    }
    window.addEventListener("pointermove", move);
    window.addEventListener("pointerup", up);
  }

  const shellStyle = { "--sim-side": `${sideWidth}px` } as CSSProperties;

  return (
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
              <h2 className="sim-group-title">Environment</h2>
              <div className="sim-choice" role="radiogroup" aria-label="Environment">
                {WORLDS.map((item) => (
                  <button
                    key={item.id}
                    type="button"
                    role="radio"
                    aria-checked={world === item.id}
                    onClick={() => chooseWorld(item.id)}
                  >
                    {item.label}
                  </button>
                ))}
              </div>
            </section>
            <section>
              <h2 className="sim-group-title">Controller</h2>
              <div className="sim-choice" role="radiogroup" aria-label="Controller">
                {CONTROLLERS.map((item) => (
                  <button
                    key={item.id}
                    type="button"
                    role="radio"
                    aria-checked={controller === item.id}
                    onClick={() => chooseController(item.id)}
                  >
                    {item.label}
                  </button>
                ))}
              </div>
              <p className="sim-lead">{controllerLine}</p>
            </section>
            <section>
              <h2 className="sim-group-title">Initial conditions</h2>
              <div className="sim-choice" role="radiogroup" aria-label="Start">
                <button type="button" role="radio" aria-checked={drop} onClick={() => chooseDivert(false)}>
                  Drop
                </button>
                <button type="button" role="radio" aria-checked={divert} onClick={() => chooseDivert(true)}>
                  Divert
                </button>
              </div>
              {startLine ? <p className="sim-lead">{startLine}</p> : null}
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
                {START.map((key) => (
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
                <h2 className="sim-group-title">Sea</h2>
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
            <details className="sim-limits">
              <summary>Limits</summary>
              <div className="sim-figures">
                {limitKeys.map((key) => {
                  const spec = specFor(key);
                  const locked = world === "moon" && (key === "floor" || key === "ceiling");
                  if (locked) {
                    const raw = key === "floor" ? flight.simulation.thrust_min / 1000 : flight.simulation.thrust_max / 1000;
                    return (
                      <div key={key} className="sim-figure sim-metric">
                        <span className="sim-figure-label">{spec.label}</span>
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
          <div className="sim-plots">
            <SignalPlot label="Left plot" signal={leftSignal} onSignal={setLeftSignal} option={altitude} onScrub={scrub} />
            <SignalPlot label="Right plot" signal={rightSignal} onSignal={setRightSignal} option={rate} onScrub={scrub} />
          </div>
        </div>
      </div>
    </div>
  );
}
