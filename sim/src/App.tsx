import { Pause, Play, Volume2, VolumeX } from "lucide-react";
import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { ChartView } from "./ChartView";
import { ChatDock } from "./chat/ChatDock";
import { Essay } from "./Essay";
import { Landing } from "./Landing";
import { TopBar } from "./TopBar";
import { Viewport } from "./Viewport";
import { liveOption } from "./chartOption";
import { CHAPTERS, chapterForSceneId, pathForEntry, readyScenesInChapter } from "./catalog";
import { chatBrief, durationOf, knobOf, readout, sceneById, type SceneId } from "./scenes";
import { setKnob, setScene, setTime, togglePlay, useKnobs, usePlaying, useScene, useTime } from "./sceneStore";
import { soundOn, toggleSound, unlock } from "./sound";
import { isDynamicsPath, pathForScene, redirectForPath, sceneFromPath } from "./paths";
import { usePath, visit } from "./route";
import { clampSplit, defaultSplit, loadSplit, saveSplit } from "./splitPane";
import { SCENE_VEIL_IN_MS, SCENE_VEIL_OUT_MS, sceneMotionOk } from "./sceneTransition";

export function App() {
  const path = usePath();
  const dynamics = isDynamicsPath(path) || redirectForPath(path) !== null;
  const [asking, setAsking] = useState(false);

  useEffect(() => {
    const redirect = redirectForPath(path);
    if (redirect) {
      visit(redirect);
      return;
    }
    const sceneId = sceneFromPath(path);
    if (sceneId) setScene(sceneId);
  }, [path]);

  useEffect(() => {
    document.title = dynamics ? "Icarus · Rocket Dynamics" : "Icarus";
  }, [dynamics]);

  return (
    <div className={dynamics ? "site" : "site is-cover"}>
      <TopBar dynamics={dynamics} asking={asking} onAsk={() => setAsking((open) => !open)} />
      {dynamics ? <Model open={asking} onClose={() => setAsking(false)} /> : <Landing />}
    </div>
  );
}

function Model({ open, onClose }: { open: boolean; onClose: () => void }) {
  const time = useTime();
  const scene = useScene();
  const knobs = useKnobs();
  const playing = usePlaying();
  const spec = sceneById(scene);
  const chapter = chapterForSceneId(scene);
  const beats = readyScenesInChapter(chapter.id);
  const chapterLinks = CHAPTERS.map((item) => {
    const first = item.scenes.find((entry) => entry.ready && entry.sceneId);
    if (!first) return null;
    return { id: item.id, title: item.title, href: pathForEntry(item.slug, first.slug) };
  }).filter(Boolean) as { id: string; title: string; href: string }[];
  const live = useMemo(() => liveOption(scene, knobs, time), [scene, knobs, time]);
  const knob = knobOf(scene, knobs);
  const lines = readout(scene, time, knobs);
  const [hear, setHear] = useState(soundOn);
  const [stageVeil, setStageVeil] = useState(false);
  const sceneSwitchRef = useRef<number | null>(null);
  const sceneSwitchGen = useRef(0);
  const [prompts, setPrompts] = useState<Record<string, string>>({});
  const remember = useCallback((id: string, prompt: string) => {
    setPrompts((prev) => (prev[id] === prompt ? prev : { ...prev, [id]: prompt }));
  }, []);

  useEffect(() => {
    setPrompts({});
  }, [scene]);

  useEffect(() => {
    return () => {
      if (sceneSwitchRef.current !== null) window.clearTimeout(sceneSwitchRef.current);
    };
  }, []);

  const question = Object.values(prompts).filter(Boolean).join("\n");

  const frameRef = useRef<HTMLDivElement>(null);
  const [readingPx, setReadingPx] = useState(() => loadSplit() ?? 560);
  const [resizing, setResizing] = useState(false);

  useEffect(() => {
    const frame = frameRef.current;
    if (!frame) return;
    const stored = loadSplit();
    const width = frame.getBoundingClientRect().width;
    setReadingPx(stored !== null ? clampSplit(stored, width) : defaultSplit(width));
  }, []);

  useEffect(() => {
    if (!resizing) return;
    const frame = frameRef.current;
    if (!frame) return;

    const onMove = (event: PointerEvent) => {
      const rect = frame.getBoundingClientRect();
      setReadingPx(clampSplit(event.clientX - rect.left, rect.width));
    };

    const onUp = (event: PointerEvent) => {
      setResizing(false);
      const rect = frame.getBoundingClientRect();
      const next = clampSplit(event.clientX - rect.left, rect.width);
      setReadingPx(next);
      saveSplit(next);
    };

    window.addEventListener("pointermove", onMove);
    window.addEventListener("pointerup", onUp);
    window.addEventListener("pointercancel", onUp);
    return () => {
      window.removeEventListener("pointermove", onMove);
      window.removeEventListener("pointerup", onUp);
      window.removeEventListener("pointercancel", onUp);
    };
  }, [resizing]);

  useEffect(() => {
    const onResize = () => {
      const frame = frameRef.current;
      if (!frame) return;
      setReadingPx((prev) => clampSplit(prev, frame.getBoundingClientRect().width));
    };
    window.addEventListener("resize", onResize);
    return () => window.removeEventListener("resize", onResize);
  }, []);

  useEffect(() => {
    requestAnimationFrame(() => window.dispatchEvent(new Event("resize")));
  }, [readingPx]);

  function openScene(id: SceneId) {
    if (id === scene) return;
    if (sceneSwitchRef.current !== null) window.clearTimeout(sceneSwitchRef.current);

    if (!sceneMotionOk()) {
      visit(pathForScene(id));
      return;
    }

    const gen = sceneSwitchGen.current + 1;
    sceneSwitchGen.current = gen;
    setStageVeil(true);
    sceneSwitchRef.current = window.setTimeout(() => {
      if (sceneSwitchGen.current !== gen) return;
      visit(pathForScene(id));
      sceneSwitchRef.current = window.setTimeout(() => {
        if (sceneSwitchGen.current !== gen) return;
        setStageVeil(false);
        sceneSwitchRef.current = null;
      }, SCENE_VEIL_OUT_MS);
    }, SCENE_VEIL_IN_MS);
  }

  return (
    <>
      <div
        ref={frameRef}
        className={resizing ? "frame is-resizing" : "frame"}
        style={{ gridTemplateColumns: `${readingPx}px 6px minmax(0, 1fr)` }}
      >
        <main className="reading">
          <header className="reading-head">
            <nav className="chapter-links" aria-label="Chapters">
              {chapterLinks.map((item) => (
                <a
                  key={item.id}
                  href={item.href}
                  className={item.id === chapter.id ? "chapter-link is-current" : "chapter-link"}
                  onClick={(event) => {
                    event.preventDefault();
                    visit(item.href);
                  }}
                >
                  {item.title}
                </a>
              ))}
            </nav>
            <h1>{spec.title}</h1>
            <p className="lede">{spec.lede}</p>
            <nav className="beats" aria-label="Scenes">
              {beats.map((item, index) => (
                <button
                  key={item.sceneId}
                  type="button"
                  aria-current={item.sceneId === scene ? "page" : undefined}
                  onClick={() => openScene(item.sceneId)}
                >
                  <span className="num">0{index + 1}</span>
                  {item.name}
                </button>
              ))}
            </nav>
          </header>
          <div className="reading-scroll">
            <Essay scene={scene} brief={chatBrief(scene, knobs)} onPrompt={remember} />
          </div>
        </main>

        <div
          className="split-handle"
          role="separator"
          aria-orientation="vertical"
          aria-label="Resize reading panel"
          aria-valuenow={Math.round(readingPx)}
          onPointerDown={(event) => {
            if (event.button !== 0) return;
            event.preventDefault();
            setResizing(true);
            event.currentTarget.setPointerCapture(event.pointerId);
          }}
          onDoubleClick={() => {
            const frame = frameRef.current;
            if (!frame) return;
            const next = defaultSplit(frame.getBoundingClientRect().width);
            setReadingPx(next);
            saveSplit(next);
          }}
        />

        <section className="stage">
          <div className="viewport">
            <div className="viewport-stack">
            <Viewport />
            <div className="controls">
              <div className="slider-row">
                <label htmlFor="time">Time</label>
                <div className="transport">
                  <button
                    type="button"
                    className="play"
                    aria-label={playing ? "Pause" : "Play"}
                    aria-pressed={playing}
                    onClick={() => {
                      void unlock();
                      togglePlay();
                    }}
                  >
                    {playing ? <Pause size={14} strokeWidth={1.75} /> : <Play size={14} strokeWidth={1.75} />}
                  </button>
                  <button
                    type="button"
                    className="play"
                    aria-label={hear ? "Sound" : "Muted"}
                    aria-pressed={hear}
                    onClick={() => setHear(toggleSound())}
                  >
                    {hear ? <Volume2 size={14} strokeWidth={1.75} /> : <VolumeX size={14} strokeWidth={1.75} />}
                  </button>
                </div>
                <input
                  id="time"
                  type="range"
                  min={0}
                  max={durationOf(scene)}
                  step={0.01}
                  value={time}
                  onChange={(event) => {
                    void unlock();
                    setTime(Number(event.target.value));
                  }}
                />
                <output>
                  {lines.map((line) => (
                    <span key={line.text} className={line.gold ? "gold" : undefined}>
                      {line.text}
                    </span>
                  ))}
                </output>
              </div>
              {knob ? (
                <div className="slider-row">
                  <label htmlFor="knob">{knob.label}</label>
                  <span />
                  <input
                    id="knob"
                    type="range"
                    min={knob.min}
                    max={knob.max}
                    step={knob.step}
                    value={knob.value}
                    onChange={(event) => setKnob(knob.key, Number(event.target.value))}
                  />
                  <output>
                    <span>{knob.show}</span>
                    {knob.aside ? <span className="gold">{knob.aside}</span> : null}
                  </output>
                </div>
              ) : null}
            </div>
            <ChartView option={live} layoutKey={readingPx} sceneKey={scene} />
            <div className={stageVeil ? "stage-veil is-active" : "stage-veil"} aria-hidden />
            </div>
          </div>
          <ChatDock open={open} onClose={onClose} question={question} />
        </section>
      </div>
    </>
  );
}
