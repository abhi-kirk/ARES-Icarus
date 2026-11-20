import { useFrame } from "@react-three/fiber";
import { useEffect, useRef, useState } from "react";
import {
  AdditiveBlending,
  NormalBlending,
  Vector3,
  type Blending,
  type Object3D,
} from "three";
import {
  createParticleSystem,
  updateParticleSystems,
  type ParticleSystem,
  type ParticleSystemConfig,
} from "@newkrok/three-particles";
import type { Touch } from "./touch";

// The burn clock stops on the last sample, which is the ground. The puff and
// the fire play on the render clock after the playhead sits there, so a scrub
// back into the flight can rewind them.

// The library treats a clock of 0 as "use the wall clock". A fixed origin keeps
// the burst on flight time, so a scrub can replay it.
const CLOCK_ORIGIN = 1000;

const FADE = {
  isActive: true,
  lifetimeCurve: {
    type: "BEZIER" as const,
    scale: 1,
    bezierPoints: [
      { x: 0, y: 1, percentage: 0 },
      { x: 0.25, y: 0.85 },
      { x: 1, y: 0, percentage: 1 },
    ],
  },
};

function renderer(blending: Blending) {
  return {
    blending,
    discardBackgroundColor: false,
    backgroundColorTolerance: 1,
    backgroundColor: { r: 0, g: 0, b: 0 },
    transparent: true,
    depthTest: true,
    depthWrite: false,
  };
}

function puff(touch: Touch, world: "earth" | "moon"): ParticleSystemConfig {
  const moon = world === "moon";
  // Circle emits in its local xy plane. A 90° tilt lays that plane on the ground.
  // A cone's axis is local +z. Tilting it by -90° points the spray upward.
  const position = new Vector3(touch.x, 0.6, touch.y);
  if (moon) {
    return {
      transform: {
        position,
        rotation: new Vector3(90, 0, 0),
        scale: new Vector3(1, 1, 1),
      },
      duration: 0.12,
      looping: false,
      startLifetime: { min: 0.45, max: 0.7 },
      startSpeed: { min: 40, max: 70 },
      startSize: { min: 50, max: 110 },
      startOpacity: 0.7,
      maxParticles: 160,
      gravity: 0,
      emission: { rateOverTime: 700, rateOverDistance: 0 },
      shape: {
        shape: "CIRCLE",
        circle: { radius: 8, radiusThickness: 1, arc: 360 },
      },
      startColor: {
        min: { r: 0.62, g: 0.58, b: 0.5 },
        max: { r: 0.86, g: 0.82, b: 0.74 },
      },
      renderer: renderer(NormalBlending),
      opacityOverLifetime: FADE,
    } as ParticleSystemConfig;
  }
  return {
    transform: {
      position,
      rotation: new Vector3(-90, 0, 0),
      scale: new Vector3(1, 1, 1),
    },
    duration: 0.15,
    looping: false,
    startLifetime: { min: 1.4, max: 2.2 },
    startSpeed: { min: 14, max: 28 },
    startSize: { min: 36, max: 80 },
    startOpacity: 0.55,
    maxParticles: 140,
    gravity: 6,
    emission: { rateOverTime: 500, rateOverDistance: 0 },
    shape: {
      shape: "CONE",
      cone: { angle: 72, radius: 4, radiusThickness: 1, arc: 360 },
    },
    startColor: {
      min: { r: 0.75, g: 0.84, b: 0.9 },
      max: { r: 0.95, g: 0.97, b: 0.98 },
    },
    renderer: renderer(NormalBlending),
    opacityOverLifetime: FADE,
  } as ParticleSystemConfig;
}

function fire(touch: Touch): ParticleSystemConfig {
  return {
    transform: {
      position: new Vector3(touch.x, 1.4, touch.y),
      rotation: new Vector3(-90, 0, 0),
      scale: new Vector3(1, 1, 1),
    },
    duration: 0.55,
    looping: false,
    startDelay: 0.35,
    startLifetime: { min: 0.7, max: 1.2 },
    startSpeed: { min: 6, max: 14 },
    startSize: { min: 70, max: 140 },
    startOpacity: 0.9,
    maxParticles: 100,
    gravity: 2,
    emission: { rateOverTime: 90, rateOverDistance: 0 },
    shape: {
      shape: "CONE",
      cone: { angle: 22, radius: 2.5, radiusThickness: 1, arc: 360 },
    },
    startColor: {
      min: { r: 1, g: 0.28, b: 0.05 },
      max: { r: 1, g: 0.72, b: 0.15 },
    },
    renderer: renderer(AdditiveBlending),
    opacityOverLifetime: FADE,
  } as ParticleSystemConfig;
}

export function LandingPicture({
  touch,
  world,
  age,
}: {
  touch: Touch;
  world: "earth" | "moon";
  age: { current: number };
}) {
  const live = useRef<ParticleSystem[]>([]);
  const integrated = useRef(0);
  const [nodes, setNodes] = useState<Object3D[]>([]);

  useEffect(() => {
    return () => {
      for (const system of live.current) system.dispose();
      live.current = [];
      integrated.current = 0;
    };
  }, [touch, world]);

  useFrame(() => {
    const shown = age.current;
    if (shown + 0.02 < integrated.current) {
      for (const system of live.current) system.dispose();
      live.current = [];
      integrated.current = 0;
      setNodes([]);
      return;
    }
    if (shown <= 0) return;

    if (live.current.length === 0) {
      const made: ParticleSystem[] = [];
      if (touch.kind === "place") made.push(createParticleSystem(puff(touch, world), CLOCK_ORIGIN));
      if (touch.kind === "hard") made.push(createParticleSystem(fire(touch), CLOCK_ORIGIN));
      live.current = made;
      integrated.current = 0;
      setNodes(made.map((system) => system.instance));
    }

    let cursor = integrated.current;
    const capped = Math.min(shown, 3.5);
    while (cursor < capped - 1e-4) {
      const step = Math.min(0.05, capped - cursor);
      cursor += step;
      updateParticleSystems({
        now: CLOCK_ORIGIN + cursor * 1000,
        delta: step,
        elapsed: cursor,
      });
    }
    integrated.current = cursor;
  });

  return (
    <>
      {nodes.map((node, index) => (
        <primitive key={index} object={node} />
      ))}
    </>
  );
}
