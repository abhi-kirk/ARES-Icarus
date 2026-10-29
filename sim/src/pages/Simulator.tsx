import { Pause, Play } from "lucide-react";
import { useEffect, useRef, useState, type KeyboardEvent as ReactKeyboardEvent, type PointerEvent as ReactPointerEvent } from "react";
import { ChartView } from "../ChartView";
import { FlightView } from "../FlightView";
import {
  KNOBS,
  NOMINAL_CASE,
  formatKnob,
  readKnob,
  sameCase,
  withKnob,
  type FlightCase,
  type KnobKey,
  type KnobSpec,
} from "../flight/case";
import { flightTrace } from "../flight/chart";
import { endTime, parseLines, parseSample, sampleAt, type Sample } from "../flight/samples";

const TAPES: { key: KnobKey; span: number; tick: number }[] = [
  { key: "height", span: 200, tick: 40 },
  { key: "descent", span: 40, tick: 5 },
];

const FIGURES: { title: string; keys: KnobKey[] }[] = [
  { title: "Initial", keys: ["offset", "fuel"] },
  { title: "Sea", keys: ["wind", "pad"] },
  { title: "Guidance", keys: ["gain"] },
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

function flightMetrics(sample: Sample | null): { label: string; value: string; unit: string }[] {
  const dx = (sample?.x ?? 0) - (sample?.padX ?? 0);
  const dy = (sample?.y ?? 0) - (sample?.padY ?? 0);
  const range = Math.hypot(dx, dy);
  const speed = Math.hypot(sample?.vx ?? 0, sample?.vy ?? 0);
  const thrust = Math.hypot(sample?.fx ?? 0, sample?.fy ?? 0, sample?.fz ?? 0) / 1000;
  const side = Math.hypot(sample?.fx ?? 0, sample?.fy ?? 0) / 1000;
  return [
    { label: "Range", value: range.toFixed(0), unit: "m" },
    { label: "Speed", value: speed.toFixed(1), unit: "m/s" },
    { label: "Along", value: dx.toFixed(0), unit: "m" },
    { label: "Across", value: dy.toFixed(0), unit: "m" },
    { label: "Thrust", value: thrust.toFixed(1), unit: "kN" },
    { label: "Side", value: side.toFixed(1), unit: "kN" },
  ];
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

  const samplesRef = useRef<Sample[]>([]);
  const playingRef = useRef(true);
  const doneRef = useRef(false);
  const draftRef = useRef<FlightCase>(NOMINAL_CASE);
  const flownRef = useRef<FlightCase>(NOMINAL_CASE);
  const abortRef = useRef<AbortController | null>(null);
  const generation = useRef(0);
  const requested = useRef(false);

  playingRef.current = playing;
  doneRef.current = done;

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
      if (playingRef.current) {
        const list = samplesRef.current;
        const end = endTime(list);
        setTime((current) => {
          if (list.length === 0) return 0;
          const next = current + elapsed;
          if (next < end) return next;
          if (doneRef.current) {
            playingRef.current = false;
            setPlaying(false);
            return end;
          }
          return end;
        });
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

  const shown = sampleAt(samples, time);
  const duration = Math.max(endTime(samples), 0.01);
  const altitude = flightTrace(samples, time, "height");
  const rate = flightTrace(samples, time, "rate");

  function scrub(next: number) {
    const end = endTime(samplesRef.current);
    const placed = Math.min(Math.max(0, next), end);
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

  return (
    <div className="frame is-sim">
      <div className="sim-shell">
        <div className="sim-view">
          <FlightView sample={shown} samples={samples} time={time} />
          <div className="sim-readout">
            <span>ALT {formatMeters(shown?.z ?? 0)}</span>
            <span>VZ {formatRate(shown?.vz ?? 0)}</span>
            <span>FUEL {(shown?.fuel ?? 0).toFixed(0)} kg</span>
          </div>
          <p className="sim-credit">
            Ship model:{" "}
            <a href="https://sketchfab.com/3d-models/spacex-starship-sn24-superheavy-bn7-cb1b096d64b443ef9f87dd0cf960cf47">
              SpaceX Starship SN24 Superheavy BN7
            </a>{" "}
            by <a href="https://sketchfab.com/clarence365">Clarence365</a>,{" "}
            <a href="https://creativecommons.org/licenses/by/4.0/">CC BY 4.0</a>
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
              <h2 className="sim-group-title">Initial conditions</h2>
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
            </section>
            {FIGURES.map((group) => (
              <section key={group.title}>
                {group.title !== "Initial" ? <h2 className="sim-group-title">{group.title}</h2> : null}
                <div className="sim-figures">
                  {group.keys.map((key) => (
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
            ))}
            <section>
              <h2 className="sim-group-title">Now</h2>
              <div className="sim-figures">
                {flightMetrics(shown).map((metric) => (
                  <div key={metric.label} className="sim-figure sim-metric">
                    <span className="sim-figure-label">{metric.label}</span>
                    <span className="sim-figure-value">
                      {metric.value}
                      <span className="sim-figure-unit">{metric.unit}</span>
                    </span>
                  </div>
                ))}
              </div>
            </section>
            {note ? <p className="sim-note">{note}</p> : null}
          </div>
          <div className="sim-charts">
            <div className="sim-clock">
              <button
                type="button"
                className="play sim-play"
                aria-label={playing ? "Pause" : "Play"}
                aria-pressed={playing}
                onClick={() => {
                  if (!playing && time >= endTime(samples) - 1e-3 && done) {
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
                onChange={(event) => scrub(Number(event.target.value))}
              />
            </div>
            <div className="sim-plot">
              <ChartView option={altitude} onScrub={scrub} />
            </div>
            <div className="sim-plot">
              <ChartView option={rate} onScrub={scrub} />
            </div>
          </div>
        </aside>
      </div>
    </div>
  );
}
