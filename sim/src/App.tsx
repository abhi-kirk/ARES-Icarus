import { Pause, Play, Volume2, VolumeX } from "lucide-react";
import { useCallback, useEffect, useMemo, useState } from "react";
import { ChartView } from "./ChartView";
import { ChatDock } from "./chat/ChatDock";
import { Essay } from "./Essay";
import { Landing } from "./Landing";
import { TopBar } from "./TopBar";
import { Viewport } from "./Viewport";
import { liveOption } from "./chartOption";
import { chapterForSceneId, readyScenesInChapter } from "./catalog";
import { chatBrief, durationOf, knobOf, readout, sceneById, type SceneId } from "./scenes";
import { setKnob, setScene, setTime, togglePlay, useKnobs, usePlaying, useScene, useTime } from "./sceneStore";
import { soundOn, toggleSound, unlock } from "./sound";
import { isDynamicsPath, pathForScene, redirectForPath, sceneFromPath } from "./paths";
import { usePath, visit } from "./route";

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
  const live = useMemo(() => liveOption(scene, knobs, time), [scene, knobs, time]);
  const knob = knobOf(scene, knobs);
  const lines = readout(scene, time, knobs);
  const [hear, setHear] = useState(soundOn);
  const [prompts, setPrompts] = useState<Record<string, string>>({});
  const remember = useCallback((id: string, prompt: string) => {
    setPrompts((prev) => (prev[id] === prompt ? prev : { ...prev, [id]: prompt }));
  }, []);

  useEffect(() => {
    setPrompts({});
  }, [scene]);

  const question = Object.values(prompts).filter(Boolean).join("\n");

  function openScene(id: SceneId) {
    if (id === scene) return;
    visit(pathForScene(id));
  }

  return (
    <>
      <div className="frame">
        <main className="reading">
          <header className="reading-head">
            <p className="meta">{chapter.title}</p>
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

        <section className="stage">
          <div className="viewport">
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
            <ChartView option={live} />
          </div>
          <ChatDock key={scene} open={open} onClose={onClose} question={question} />
        </section>
      </div>
    </>
  );
}
