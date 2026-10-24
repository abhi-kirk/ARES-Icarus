import { useEffect, useRef, useState, type RefObject } from "react";
import { COMING_SOON, DOORS } from "./doors";
import { follow } from "./route";

// Take off through the top of the screen, stay gone, come back down onto the pad.

const G = 9.81;
const LIFTOFF = 1.65 * G;
const LANDING = 2.2 * G;
const DT = 1 / 60;
const HOLD = 2.6;
const AWAY = 3.2;
const GRAPHITE = "#6a6f76";

type Frame = { y: number; thrust: number; legs: number };

function hopFrames(clearY: number): Frame[] {
  let y = 0;
  let vy = 0;
  let phase: "up" | "away" | "down" = "up";
  let awayLeft = AWAY;
  let burning = false;
  const frames: Frame[] = [];

  const push = (thrust: number) => {
    const legs = y < 18 ? Math.min(1, (18 - y) / 12) : 0;
    frames.push({ y, thrust: thrust / LANDING, legs });
  };

  for (let i = 0; i < 60 * 24; i++) {
    if (phase === "up") {
      push(LIFTOFF);
      const ay = -G + LIFTOFF;
      y += vy * DT + 0.5 * ay * DT * DT;
      vy += ay * DT;
      if (y >= clearY) {
        phase = "away";
        y = clearY + 40;
        vy = 0;
      }
      continue;
    }

    if (phase === "away") {
      push(0);
      awayLeft -= DT;
      if (awayLeft <= 0) {
        phase = "down";
        y = clearY + 8;
        vy = -16;
      }
      continue;
    }

    let thrust = 0;
    if (!burning) {
      const need = (vy * vy) / (2 * Math.max(y, 0.2)) + G;
      if (need >= LANDING * 0.9) burning = true;
    }
    if (burning) {
      const need = (vy * vy) / (2 * Math.max(y, 0.15)) + G;
      thrust = Math.min(LANDING, need);
    }
    push(thrust);

    const ay = -G + thrust;
    const nextY = y + vy * DT + 0.5 * ay * DT * DT;
    const nextVy = vy + ay * DT;
    if (burning && (nextY <= 0 || (nextVy >= 0 && y < 2))) {
      frames.push({ y: 0, thrust: 0, legs: 1 });
      return frames;
    }
    y = nextY;
    vy = nextVy;
  }

  frames.push({ y: 0, thrust: 0, legs: 1 });
  return frames;
}

let cached = { key: "", frames: [] as Frame[] };

function framesFor(clearY: number): Frame[] {
  const key = clearY.toFixed(0);
  if (key !== cached.key) cached = { key, frames: hopFrames(clearY) };
  return cached.frames;
}

function frameAt(frames: Frame[], time: number): Frame {
  const last = frames[frames.length - 1];
  if (time < 0) return last;
  const flight = (frames.length - 1) * DT;
  const t = time % (flight + HOLD);
  if (t >= flight) return last;
  const index = t / DT;
  const i = Math.floor(index);
  const u = index - i;
  const a = frames[i];
  const b = frames[Math.min(frames.length - 1, i + 1)];
  return {
    y: a.y + (b.y - a.y) * u,
    thrust: a.thrust + (b.thrust - a.thrust) * u,
    legs: a.legs + (b.legs - a.legs) * u,
  };
}

function draw(ctx: CanvasRenderingContext2D, width: number, height: number, time: number, x: number) {
  const narrow = width < 860;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#000000";
  ctx.fillRect(0, 0, width, height);

  const padY = narrow ? height * 0.58 : height * 0.8;
  const bodyH = Math.min(height * (narrow ? 0.22 : 0.26), 220);
  const bodyW = bodyH / 10.5;
  const pxPerM = bodyH / 47;
  const drop = Math.max(bodyH, padY - bodyH - height * 0.04);
  const clearY = (padY + bodyH * 0.55 + 24) / pxPerM;
  const frame = frameAt(framesFor(clearY), time);
  const contact = padY - frame.y * pxPerM;

  const engineDrop = bodyH * 0.07;
  const footDrop = bodyH * 0.13 * frame.legs;
  const skirt = contact - Math.max(engineDrop, footDrop);
  const top = skirt - bodyH;

  const near = 1 - Math.min(1, frame.y / Math.max(1, drop / pxPerM));
  const glow = frame.thrust * (0.12 + 0.88 * near);
  if (glow > 0.02) {
    const light = ctx.createRadialGradient(x, padY, 0, x, padY, bodyH * 0.85);
    light.addColorStop(0, `rgba(226, 164, 58, ${0.45 * glow})`);
    light.addColorStop(1, "rgba(226, 164, 58, 0)");
    ctx.fillStyle = light;
    ctx.beginPath();
    ctx.ellipse(x, padY, bodyH * 0.72, bodyH * 0.07, 0, 0, Math.PI * 2);
    ctx.fill();
  }

  ctx.strokeStyle = "rgba(245, 245, 245, 0.35)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(x - bodyH * 0.42, padY);
  ctx.lineTo(x + bodyH * 0.42, padY);
  ctx.stroke();

  const pivotY = top + bodyH * 0.55;
  ctx.save();
  ctx.translate(x, pivotY);
  const localSkirt = skirt - pivotY;
  drawLegs(ctx, 0, localSkirt, bodyW, bodyH, frame.legs);
  drawBody(ctx, 0, top - pivotY, localSkirt, bodyW, bodyH);
  const room = Math.max(0, padY - (skirt + engineDrop) - 2);
  drawFlame(ctx, 0, localSkirt + engineDrop, bodyW, bodyH, frame.thrust, time, localSkirt + engineDrop + room);
  ctx.restore();
}

function drawLegs(ctx: CanvasRenderingContext2D, x: number, skirt: number, bodyW: number, bodyH: number, legs: number) {
  if (legs < 0.04) return;
  const hingeY = skirt - bodyH * 0.04;
  ctx.strokeStyle = "#f3f4f5";
  ctx.fillStyle = GRAPHITE;
  ctx.lineWidth = Math.max(1.5, bodyW * 0.14);
  ctx.lineCap = "round";
  for (const side of [-1, 1]) {
    const footX = x + side * (bodyW * 0.55 + legs * bodyW * 1.8);
    const footY = hingeY + bodyH * 0.16 * legs;
    ctx.beginPath();
    ctx.moveTo(x + side * bodyW * 0.42, hingeY);
    ctx.lineTo(footX, footY);
    ctx.stroke();
    if (legs > 0.4) ctx.fillRect(footX - bodyW * 0.42, footY - bodyW * 0.08, bodyW * 0.84, bodyW * 0.16);
  }
}

function drawBody(ctx: CanvasRenderingContext2D, x: number, top: number, skirt: number, bodyW: number, bodyH: number) {
  const left = x - bodyW / 2;
  const stage = bodyH * 0.2;
  ctx.fillStyle = "#f3f4f5";
  ctx.beginPath();
  ctx.roundRect(left, top + stage * 0.72, bodyW, bodyH - stage * 0.72, [2, 2, bodyW * 0.15, bodyW * 0.15]);
  ctx.fill();

  ctx.fillStyle = GRAPHITE;
  ctx.beginPath();
  ctx.roundRect(left, top, bodyW, stage, [bodyW * 0.45, bodyW * 0.45, 1, 1]);
  ctx.fill();

  ctx.strokeStyle = "#9a9188";
  ctx.lineWidth = 1;
  const finTop = top + stage - bodyH * 0.015;
  const finH = bodyH * 0.07;
  const finW = bodyW * 0.7;
  for (const side of [-1, 1]) {
    const finX = side > 0 ? x + bodyW / 2 : x - bodyW / 2 - finW;
    ctx.strokeRect(finX, finTop, finW, finH);
    ctx.beginPath();
    ctx.moveTo(finX + finW * 0.33, finTop);
    ctx.lineTo(finX + finW * 0.33, finTop + finH);
    ctx.moveTo(finX + finW * 0.66, finTop);
    ctx.lineTo(finX + finW * 0.66, finTop + finH);
    ctx.stroke();
  }

  ctx.fillStyle = "#c4c0b6";
  ctx.fillRect(left, skirt - bodyH * 0.16, bodyW, bodyH * 0.1);

  ctx.fillStyle = GRAPHITE;
  ctx.beginPath();
  ctx.moveTo(left, skirt - bodyH * 0.05);
  ctx.lineTo(x - bodyW * 0.7, skirt + bodyH * 0.07);
  ctx.lineTo(x + bodyW * 0.7, skirt + bodyH * 0.07);
  ctx.lineTo(left + bodyW, skirt - bodyH * 0.05);
  ctx.closePath();
  ctx.fill();
}

function drawFlame(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  bodyW: number,
  bodyH: number,
  thrust: number,
  time: number,
  padY: number,
) {
  if (thrust < 0.02) return;
  const room = Math.max(0, padY - y - 2);
  const length = Math.min(bodyH * (0.08 + thrust * 0.42), room);
  if (length < 2) return;
  const wobble = Math.sin(time * 22) * bodyW * 0.15 * thrust;
  ctx.save();
  ctx.globalCompositeOperation = "lighter";
  const color = ctx.createLinearGradient(x, y, x, y + length);
  color.addColorStop(0, `rgba(255, 228, 180, ${0.95 * thrust})`);
  color.addColorStop(0.4, `rgba(226, 148, 48, ${0.55 * thrust})`);
  color.addColorStop(1, "rgba(226, 100, 20, 0)");
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(x - bodyW * 0.42, y);
  ctx.lineTo(x + bodyW * 0.42, y);
  ctx.lineTo(x + wobble, y + length);
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function Stage({ mark }: { mark: RefObject<HTMLDivElement | null> }) {
  const ref = useRef<HTMLCanvasElement>(null);

  useEffect(() => {
    const canvas = ref.current;
    const parent = canvas?.parentElement;
    if (!canvas || !parent) return;
    const media = window.matchMedia("(prefers-reduced-motion: reduce)");
    let frame = 0;
    let alive = true;

    const paint = (now: number) => {
      const rect = parent.getBoundingClientRect();
      const dpr = Math.min(window.devicePixelRatio || 1, 2);
      const width = Math.max(1, rect.width);
      const height = Math.max(1, rect.height);
      const bufferW = Math.round(width * dpr);
      const bufferH = Math.round(height * dpr);
      if (canvas.width !== bufferW || canvas.height !== bufferH) {
        canvas.width = bufferW;
        canvas.height = bufferH;
      }
      const ctx = canvas.getContext("2d");
      if (!ctx) return;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      const time = media.matches ? -1 : now / 1000;
      const slot = mark.current?.getBoundingClientRect();
      const origin = parent.getBoundingClientRect();
      const x = slot ? slot.left - origin.left + slot.width / 2 : width * 0.62;
      draw(ctx, width, height, time, x);
    };

    const tick = (now: number) => {
      if (!alive) return;
      paint(now);
      if (!media.matches) frame = requestAnimationFrame(tick);
    };
    frame = requestAnimationFrame(tick);
    return () => {
      alive = false;
      cancelAnimationFrame(frame);
    };
  }, []);

  return <canvas ref={ref} aria-hidden="true" />;
}

export function Landing() {
  const mark = useRef<HTMLDivElement>(null);
  const [note, setNote] = useState("");

  return (
    <main className="landing">
      <div className="landing-stage">
        <Stage mark={mark} />
      </div>
      <div className="landing-combo">
        <div className="landing-copy">
          <h1>Icarus</h1>
          <div className="landing-doors">
            {DOORS.map((door) =>
              door.href ? (
                <a key={door.id} href={door.href} onClick={(event) => follow(event, door.href)}>
                  {door.name}
                </a>
              ) : (
                <button key={door.id} type="button" onClick={() => setNote(COMING_SOON)}>
                  {door.name}
                </button>
              ),
            )}
          </div>
          {note ? <p className="nav-note">{note}</p> : null}
        </div>
        <div className="landing-mark" ref={mark} />
      </div>
    </main>
  );
}
