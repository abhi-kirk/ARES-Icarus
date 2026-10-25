import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { Environment, Lightformer, OrbitControls, RoundedBox } from "@react-three/drei";
import { useEffect, useLayoutEffect, useMemo, useRef, type RefObject } from "react";
import {
  AdditiveBlending,
  CatmullRomCurve3,
  ExtrudeGeometry,
  Path,
  ShaderMaterial,
  Shape,
  TubeGeometry,
  Vector3,
} from "three";
import type { Group, Mesh } from "three";
import type { OrbitControls as OrbitRig } from "three-stdlib";
import { Rocket } from "./Rocket";
import { bodyAt, clampKnob, viewSpan, type SceneId, type Knobs } from "./scenes";
import { getKnobs, getScene, getTime, setKnob, setTime } from "./sceneStore";
import { unlock } from "./sound";

const NIGHT = "#000000";
const GOLD = "#f0c15a";
const METAL = "#d7dfda";
const SLEEVE = "#8b9690";

const CEILING = 1.62;
const MASS_H = 0.3;
const DAMPER_X = 0.28;
const SLEEVE_LEN = 0.72;
const CAMERA = { position: [4.2, 0.4, 6.2] as [number, number, number], fov: 40 };
const TARGET: [number, number, number] = [0, 0.15, 0];
const POLAR = Math.acos(
  (CAMERA.position[1] - TARGET[1]) / Math.hypot(CAMERA.position[0], CAMERA.position[1] - TARGET[1], CAMERA.position[2]),
);

function viewY(height: number, scene: SceneId, knobs: Knobs): number {
  const span = viewSpan(scene, knobs);
  const fraction = (height - span.min) / (span.max - span.min);
  if (scene === "state" || scene === "step") return -1.2 + fraction * 1.15;
  return -1.5 + fraction * 3;
}

function dragValue(dy: number, scene: SceneId): void {
  const knobs = getKnobs();
  if (scene === "state") {
    // dy is upward pointer motion. A downward pull should lower the mass,
    // which is later in the swing.
    setTime(getTime() - dy * 0.012);
    return;
  }
  if (scene === "step") setKnob("dt", clampKnob("step", knobs.dt + dy * 0.004));
  if (scene === "mass") setKnob("thrust", clampKnob("mass", knobs.thrust + dy * 0.15));
  if (scene === "engine") setKnob("tau", clampKnob("engine", knobs.tau + dy * 0.004));
  if (scene === "loopOpen") setKnob("extraThrust", clampKnob("loopOpen", knobs.extraThrust + dy * 0.12));
  if (scene === "loopError") setKnob("kp", clampKnob("loopError", knobs.kp + dy * 0.08));
  if (scene === "loopEquil") setKnob("perturb", clampKnob("loopEquil", knobs.perturb + dy * 0.002));
  if (scene === "loopHover") setKnob("unstable", clampKnob("loopHover", knobs.unstable + dy * 0.015));
  if (scene === "loopMode") setKnob("coupling", clampKnob("loopMode", knobs.coupling + dy * 0.003));
  if (scene === "loopSample") setKnob("tau", clampKnob("loopSample", knobs.tau + dy * 0.004));
}

// Coil from the mount down to the mass. The ends pull in to the center line
// so the wire meets the bar and the block instead of stopping in midair.
function springGeometry(top: number, bottom: number): TubeGeometry {
  const length = Math.max(0.22, top - bottom);
  const lead = Math.min(0.07, length * 0.12);
  const coilTop = top - lead;
  const coilBottom = bottom + lead;
  const coilLength = Math.max(0.1, coilTop - coilBottom);
  const turns = 7;
  const radius = 0.085;
  const points: Vector3[] = [];

  points.push(new Vector3(0, top, 0));
  points.push(new Vector3(0, top - lead * 0.4, 0));

  const samples = turns * 12;
  for (let i = 0; i <= samples; i++) {
    const along = i / samples;
    const angle = along * turns * Math.PI * 2;
    const turnsFromEnd = Math.min(along * turns, (1 - along) * turns);
    const open = Math.min(1, turnsFromEnd / 0.45);
    const y = coilTop - coilLength * along;
    points.push(new Vector3(Math.cos(angle) * radius * open, y, Math.sin(angle) * radius * open));
  }

  points.push(new Vector3(0, bottom + lead * 0.35, 0));
  points.push(new Vector3(0, bottom, 0));

  const curve = new CatmullRomCurve3(points);
  return new TubeGeometry(curve, samples + 12, 0.011, 6, false);
}

// A tube, so the rod can sit in the bore. Extrude runs along Z. The mesh turns it upright.
function sleeveGeometry(length: number, outer: number, inner: number): ExtrudeGeometry {
  const wall = new Shape();
  wall.absarc(0, 0, outer, 0, Math.PI * 2, false);
  const bore = new Path();
  bore.absarc(0, 0, inner, 0, Math.PI * 2, true);
  wall.holes.push(bore);
  const geometry = new ExtrudeGeometry(wall, { depth: length, bevelEnabled: false, curveSegments: 24, steps: 1 });
  geometry.translate(0, 0, -length / 2);
  return geometry;
}

const FLAME_VERT = `
  varying vec2 vUv;
  varying vec3 vLocal;
  void main() {
    vUv = uv;
    vLocal = position;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }
`;

const FLAME_FRAG = `
  varying vec2 vUv;
  varying vec3 vLocal;
  uniform float uTime;
  uniform float uPower;

  float hash(vec2 p) {
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
  }

  float noise(vec2 p) {
    vec2 i = floor(p);
    vec2 f = fract(p);
    float a = hash(i);
    float b = hash(i + vec2(1.0, 0.0));
    float c = hash(i + vec2(0.0, 1.0));
    float d = hash(i + vec2(1.0, 1.0));
    vec2 u = f * f * (3.0 - 2.0 * f);
    return mix(a, b, u.x) + (c - a) * u.y * (1.0 - u.x) + (d - b) * u.x * u.y;
  }

  void main() {
    float tip = clamp(vLocal.y / 0.85 + 0.5, 0.0, 1.0);
    float n = noise(vec2(vUv.x * 7.0, tip * 4.0 - uTime * 2.6));
    float radial = length(vLocal.xz);
    float edge = smoothstep(0.17, 0.02, radial * (0.45 + tip));
    float alpha = (1.0 - tip) * (0.35 + 0.65 * n) * edge * uPower;
    vec3 hot = vec3(1.0, 0.82, 0.42);
    vec3 plume = vec3(0.89, 0.45, 0.1);
    vec3 color = mix(plume, hot, (1.0 - tip) * n);
    gl_FragColor = vec4(color, alpha);
  }
`;

function Picture({ orbit }: { orbit: RefObject<OrbitRig | null> }) {
  const rig = useRef<Group>(null);
  const spring = useRef<Mesh>(null);
  const piston = useRef<Mesh>(null);
  const mass = useRef<Mesh>(null);
  const ghost = useRef<Mesh>(null);
  const rocket = useRef<Group>(null);
  const flame = useRef<Mesh>(null);
  const ground = useRef<Mesh>(null);
  const drag = useRef<{ y: number } | null>(null);
  const release = useRef<(() => void) | null>(null);
  const sprung = useRef(Number.NaN);
  const sleeve = useMemo(() => sleeveGeometry(SLEEVE_LEN, 0.05, 0.03), []);
  const flameMat = useMemo(
    () =>
      new ShaderMaterial({
        transparent: true,
        depthWrite: false,
        toneMapped: false,
        blending: AdditiveBlending,
        uniforms: {
          uTime: { value: 0 },
          uPower: { value: 1 },
        },
        vertexShader: FLAME_VERT,
        fragmentShader: FLAME_FRAG,
      }),
    [],
  );

  useFrame((state) => {
    const scene = getScene();
    const knobs = getKnobs();
    const body = bodyAt(scene, getTime(), knobs);
    const y = viewY(body.height, scene, knobs);
    const showRig = scene === "state" || scene === "step";

    if (rig.current) rig.current.visible = showRig;
    if (showRig && mass.current && spring.current && piston.current && ghost.current) {
      mass.current.position.y = y;
      ghost.current.visible = scene === "step";
      ghost.current.position.y = viewY(body.truth, scene, knobs);

      const massTop = y + MASS_H / 2;
      if (Number.isNaN(sprung.current) || Math.abs(sprung.current - massTop) > 0.001) {
        sprung.current = massTop;
        const nextSpring = springGeometry(CEILING - 0.04, massTop);
        const previous = spring.current.geometry;
        spring.current.geometry = nextSpring;
        previous.dispose();
      }

      const sleeveBottom = CEILING - 0.05 - SLEEVE_LEN;
      const pistonTop = Math.min(CEILING - 0.08, Math.max(massTop + 0.04, sleeveBottom + 0.16));
      const length = Math.max(0.08, pistonTop - massTop);
      piston.current.scale.y = length;
      piston.current.position.y = massTop + length / 2;
    }

    if (rocket.current) {
      rocket.current.visible = !showRig;
      rocket.current.position.y = y;
      rocket.current.scale.setScalar(0.4);
    }
    if (flame.current) {
      const burning = !showRig && body.flame > 0.02;
      flame.current.visible = burning;
      const width = 0.55 + body.flame * 0.45;
      const height = 0.45 + body.flame * 0.9;
      flame.current.scale.set(width, height, width);
      flameMat.uniforms.uTime.value = state.clock.elapsedTime;
      flameMat.uniforms.uPower.value = burning ? 0.45 + body.flame * 0.55 : 0;
    }
    if (ground.current) {
      ground.current.visible = !showRig;
      ground.current.position.y = viewY(0, scene, knobs) - 0.62;
    }
  });

  useEffect(() => {
    return () => release.current?.();
  }, []);

  function onPointerDown(event: { clientY: number; stopPropagation: () => void }) {
    void unlock();
    event.stopPropagation();
    release.current?.();
    const controls = orbit.current;
    if (controls) controls.enabled = false;
    drag.current = { y: event.clientY };

    const move = (pointer: PointerEvent) => {
      if (!drag.current) return;
      const dy = drag.current.y - pointer.clientY;
      drag.current.y = pointer.clientY;
      if (dy !== 0) dragValue(dy, getScene());
    };
    const up = () => {
      drag.current = null;
      window.removeEventListener("pointermove", move);
      window.removeEventListener("pointerup", up);
      if (controls) controls.enabled = true;
      if (release.current === up) release.current = null;
    };
    release.current = up;
    window.addEventListener("pointermove", move);
    window.addEventListener("pointerup", up);
  }

  return (
    <group>
      <group ref={rig}>
        <RoundedBox args={[1.2, 0.08, 0.46]} radius={0.02} smoothness={3} position={[0.12, CEILING, 0]}>
          <meshPhysicalMaterial color={METAL} metalness={0.55} roughness={0.32} />
        </RoundedBox>
        <mesh ref={spring}>
          <boxGeometry args={[0.02, 0.02, 0.02]} />
          <meshPhysicalMaterial color={METAL} metalness={0.62} roughness={0.24} />
        </mesh>
        <mesh geometry={sleeve} position={[DAMPER_X, CEILING - 0.05 - SLEEVE_LEN / 2, 0]} rotation={[-Math.PI / 2, 0, 0]}>
          <meshPhysicalMaterial color={SLEEVE} metalness={0.5} roughness={0.36} />
        </mesh>
        <mesh ref={piston} position={[DAMPER_X, CEILING - 0.9, 0]}>
          <cylinderGeometry args={[0.016, 0.016, 1, 16]} />
          <meshPhysicalMaterial color="#e7eeea" metalness={0.7} roughness={0.18} />
        </mesh>
        <mesh ref={ghost} visible={false}>
          <boxGeometry args={[0.86, 0.4, 0.5]} />
          <meshBasicMaterial color="#d7ded8" wireframe />
        </mesh>
        <RoundedBox
          ref={mass}
          args={[0.78, MASS_H, 0.4]}
          radius={0.035}
          smoothness={3}
          onPointerDown={onPointerDown}
        >
          <meshPhysicalMaterial color={GOLD} metalness={0.48} roughness={0.32} />
          <mesh position={[DAMPER_X, MASS_H / 2, 0]}>
            <cylinderGeometry args={[0.028, 0.028, 0.045, 12]} />
            <meshPhysicalMaterial color={SLEEVE} metalness={0.5} roughness={0.32} />
          </mesh>
        </RoundedBox>
      </group>

      <group
        ref={rocket}
        visible={false}
        scale={0.4}
        onPointerDown={onPointerDown}
      >
        <Rocket flameRef={flame} flameMat={flameMat} />
      </group>

      <mesh ref={ground} visible={false} rotation={[-Math.PI / 2, 0, 0]}>
        <circleGeometry args={[2.4, 40]} />
        <meshStandardMaterial color="#070707" roughness={1} metalness={0} />
      </mesh>
    </group>
  );
}

function FirstFrame({ target }: { target: [number, number, number] }) {
  const { gl, scene, camera, size, setSize } = useThree();
  useLayoutEffect(() => {
    const parent = gl.domElement.parentElement;
    const rect = parent ? parent.getBoundingClientRect() : null;
    if (rect && rect.width > 1 && (Math.abs(rect.width - size.width) > 1 || Math.abs(rect.height - size.height) > 1)) {
      setSize(rect.width, rect.height);
    }
    camera.lookAt(target[0], target[1], target[2]);
    const width = parent?.clientWidth || size.width;
    const height = parent?.clientHeight || size.height;
    gl.setSize(width, height, false);
    gl.render(scene, camera);
  }, [camera, gl, scene, setSize, size.width, size.height, target]);
  return null;
}

export function Viewport() {
  const orbit = useRef<OrbitRig | null>(null);

  return (
    <Canvas camera={CAMERA} gl={{ antialias: true, alpha: false }} onCreated={({ gl }) => gl.setClearColor(NIGHT, 1)}>
      <color attach="background" args={[NIGHT]} />
      <ambientLight intensity={0.38} />
      <directionalLight position={[4, 6, 3]} intensity={3.4} color="#fff6ea" />
      <directionalLight position={[-3.5, 1.2, -2]} intensity={0.55} color="#9aafc2" />
      <Environment resolution={256} frames={1} environmentIntensity={1.15}>
        <Lightformer form="rect" intensity={2.4} color="#fff3df" position={[2.4, 2.4, 2.6]} scale={[3.2, 2.2, 1]} />
        <Lightformer form="rect" intensity={0.9} color="#8ea6bb" position={[-3.2, 1, -2]} scale={[4, 1.6, 1]} />
        <Lightformer form="rect" intensity={1.3} color="#ffffff" position={[0, 3.4, -2]} scale={[5, 1.4, 1]} />
      </Environment>
      <FirstFrame target={TARGET} />
      <Picture orbit={orbit} />
      <OrbitControls ref={orbit} enablePan={false} minPolarAngle={POLAR} maxPolarAngle={POLAR} target={TARGET} />
    </Canvas>
  );
}
