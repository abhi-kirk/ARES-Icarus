import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { Line } from "@react-three/drei";
import { useEffect, useMemo, useRef } from "react";
import { AdditiveBlending, BackSide, DoubleSide, Quaternion, ShaderMaterial, Spherical, Vector3 } from "three";
import type { Group } from "three";
import { Ship } from "./flight/Ship";
import { sampleAt, type Sample } from "./flight/samples";

const FLAME_FULL = 25000;
const FLAME_LENGTH = 14;

const FLAME_VERT = `
  uniform float uTime;
  uniform float uPhase;
  uniform float uAmp;
  uniform vec3 uBend;
  varying float vAlong;
  varying float vBulge;
  void main() {
    float along = clamp(position.y / 7.0 + 0.5, 0.0, 1.0);
    vAlong = along;
    vec2 radial = position.xz;
    float radius = length(radial);
    vec2 dir = radius > 0.001 ? radial / radius : vec2(1.0, 0.0);
    // Turbulence travels down the jet. It does not orbit the ship.
    float travel = along * 8.0 - uTime * 5.0 + uPhase;
    float wave = sin(travel) + 0.45 * sin(travel * 1.7 + 1.2);
    float amp = smoothstep(0.05, 0.3, along) * (0.28 + along * 0.45) * uAmp;
    vBulge = wave;
    vec3 moved = position;
    moved.xz += dir * wave * amp;
    vec4 world = modelMatrix * vec4(moved, 1.0);
    world.xyz += uBend * smoothstep(0.1, 1.0, along);
    gl_Position = projectionMatrix * viewMatrix * world;
  }
`;

const FLAME_FRAG = `
  varying float vAlong;
  varying float vBulge;
  uniform float uTime;
  uniform float uPower;
  uniform float uPhase;
  uniform float uHeat;
  float hash(vec2 p) {
    return fract(sin(dot(p, vec2(127.1, 311.7))) * 43758.5453);
  }
  void main() {
    float roll = sin(vAlong * 12.0 - uTime * 8.0 + uPhase);
    float boil = sin(vAlong * 5.5 - uTime * 3.6 + uPhase * 1.4);
    float heat = clamp(0.75 + 0.28 * roll + 0.12 * boil, 0.4, 1.4);
    float fade = 1.0 - smoothstep(0.4, 1.0, vAlong);
    float shred = mix(1.0, 0.5 + 0.5 * hash(vec2(floor(vAlong * 7.0), floor(uTime * 14.0))), smoothstep(0.5, 1.0, vAlong));
    vec3 orange = vec3(1.05, 0.32, 0.05);
    vec3 gold = vec3(1.35, 0.72, 0.25);
    vec3 color = mix(orange, gold, uHeat * (1.0 - vAlong * 0.3));
    color *= 0.7 + 0.4 * heat;
    color += vec3(0.22, 0.1, 0.02) * clamp(vBulge, 0.0, 1.0);
    float alpha = fade * heat * shred * uPower * (0.5 + 0.5 * uHeat);
    gl_FragColor = vec4(color, alpha);
  }
`;

const WATER_VERT = `
  uniform float uTime;
  varying vec3 vView;
  varying vec3 vNormal;
  void main() {
    vec3 p = position;
    float wave = sin(p.x * 0.01 + uTime * 0.25) * 0.35;
    wave += cos(p.y * 0.008 + uTime * 0.16) * 0.25;
    float slopeX = cos(p.x * 0.01 + uTime * 0.25) * 0.0035;
    float slopeY = -sin(p.y * 0.008 + uTime * 0.16) * 0.002;
    p.z += wave;
    vNormal = normalize(vec3(-slopeX * 40.0, 1.0, slopeY * 40.0));
    vec4 world = modelMatrix * vec4(p, 1.0);
    vView = cameraPosition - world.xyz;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(p, 1.0);
  }
`;

const WATER_FRAG = `
  varying vec3 vView;
  varying vec3 vNormal;
  void main() {
    vec3 viewDir = normalize(vView);
    vec3 normal = normalize(vNormal);
    float fres = pow(1.0 - max(dot(normal, viewDir), 0.0), 3.2);
    vec3 deep = vec3(0.01, 0.13, 0.2);
    vec3 glaze = vec3(0.22, 0.42, 0.5);
    vec3 color = mix(deep, glaze, fres);
    vec3 sun = normalize(vec3(0.4, 0.75, 0.2));
    float shade = dot(normal, sun) * 0.5 + 0.5;
    color *= 0.82 + 0.28 * shade;
    vec3 refl = reflect(-viewDir, normal);
    float glint = pow(max(dot(refl, sun), 0.0), 48.0);
    color += vec3(0.9, 0.86, 0.7) * glint * 0.85;
    gl_FragColor = vec4(color, 1.0);
  }
`;

const SKY_VERT = `
  varying vec3 vDir;
  void main() {
    vDir = position;
    gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
  }
`;

const SKY_FRAG = `
  varying vec3 vDir;
  void main() {
    float h = normalize(vDir).y;
    vec3 top = vec3(0.05, 0.1, 0.16);
    vec3 horizon = vec3(0.55, 0.68, 0.74);
    vec3 color = mix(horizon, top, smoothstep(-0.05, 0.45, h));
    gl_FragColor = vec4(color, 1.0);
  }
`;

function pointsOf(samples: Sample[], time: number, future: boolean): [number, number, number][] {
  const out: [number, number, number][] = [];
  if (samples.length < 2) return out;
  const stride = Math.max(1, Math.floor(samples.length / 220));
  const now = sampleAt(samples, time);
  if (now && future) out.push([now.x, Math.max(now.z, 2), now.y]);
  for (let i = 0; i < samples.length; i += stride) {
    const sample = samples[i];
    const keep = future ? sample.t >= time : sample.t <= time;
    if (!keep) continue;
    out.push([sample.x, Math.max(sample.z, 2), sample.y]);
  }
  const last = samples[samples.length - 1];
  if (future && last.t >= time) out.push([last.x, Math.max(last.z, 2), last.y]);
  if (!future && now) out.push([now.x, Math.max(now.z, 2), now.y]);
  return out;
}

function padPoints(samples: Sample[], time: number, future: boolean): [number, number, number][] {
  const out: [number, number, number][] = [];
  if (samples.length < 2) return out;
  const stride = Math.max(1, Math.floor(samples.length / 90));
  const now = sampleAt(samples, time);
  const height = 1.28;
  if (now && future) out.push([now.padX, height, now.padY]);
  for (let i = 0; i < samples.length; i += stride) {
    const sample = samples[i];
    const keep = future ? sample.t >= time : sample.t <= time;
    if (!keep) continue;
    out.push([sample.padX, height, sample.padY]);
  }
  const last = samples[samples.length - 1];
  if (future && last.t >= time) out.push([last.padX, height, last.padY]);
  if (!future && now) out.push([now.padX, height, now.padY]);
  return out;
}

function padYaw(samples: Sample[], time: number): number {
  const now = sampleAt(samples, time);
  if (!now) return 0;
  const later = sampleAt(samples, time + 8);
  const earlier = sampleAt(samples, Math.max(0, time - 8));
  const other = later && later.t - now.t > 0.4 ? later : earlier;
  if (!other) return 0;
  const dt = other.t - now.t;
  if (Math.abs(dt) < 0.2) return 0;
  const vx = (other.padX - now.padX) / dt;
  const vy = (other.padY - now.padY) / dt;
  if (Math.hypot(vx, vy) < 0.04) return 0;
  return -Math.atan2(vy, vx);
}

function flameMaterial(phase: number, amp: number, heat: number): ShaderMaterial {
  const material = new ShaderMaterial({
    transparent: true,
    depthWrite: false,
    toneMapped: false,
    side: DoubleSide,
    blending: AdditiveBlending,
    uniforms: {
      uTime: { value: 0 },
      uPower: { value: 0 },
      uPhase: { value: phase },
      uAmp: { value: amp },
      uHeat: { value: heat },
      uBend: { value: new Vector3() },
    },
    vertexShader: FLAME_VERT,
    fragmentShader: FLAME_FRAG,
  });
  return material;
}

function Scene({ sample, samples, time }: { sample: Sample | null; samples: Sample[]; time: number }) {
  const latest = useRef(sample);
  latest.current = sample;
  const ship = useRef<Group>(null);
  const flame = useRef<Group>(null);
  const exhaust = useRef(new Vector3());
  const wake = useRef(new Vector3());
  const plumeDown = useRef(new Vector3(0, -1, 0));
  const shipQuat = useRef(new Quaternion());
  const rig = useRef({
    theta: 0.6,
    phi: 1.33,
    zoom: 1,
    panX: 0,
    panY: 0,
    panZ: 0,
    distance: 320,
  });
  const { gl } = useThree();
  const sheathMat = useMemo(() => flameMaterial(0, 1, 0.2), []);
  const coreMat = useMemo(() => flameMaterial(2.2, 0.55, 1), []);
  const waterMat = useMemo(
    () =>
      new ShaderMaterial({
        uniforms: { uTime: { value: 0 } },
        vertexShader: WATER_VERT,
        fragmentShader: WATER_FRAG,
      }),
    [],
  );
  const skyMat = useMemo(
    () =>
      new ShaderMaterial({
        side: BackSide,
        depthWrite: false,
        vertexShader: SKY_VERT,
        fragmentShader: SKY_FRAG,
      }),
    [],
  );

  useEffect(() => {
    const el = gl.domElement;
    const drag = { mode: "orbit" as "orbit" | "pan", x: 0, y: 0 };
    let active = false;
    const offset = new Vector3();
    const right = new Vector3();
    const lift = new Vector3();

    function down(event: PointerEvent) {
      if (event.button !== 0 && event.button !== 1 && event.button !== 2) return;
      active = true;
      drag.mode = event.button === 0 && !event.shiftKey ? "orbit" : "pan";
      drag.x = event.clientX;
      drag.y = event.clientY;
      el.setPointerCapture(event.pointerId);
    }

    function move(event: PointerEvent) {
      if (!active) return;
      const dx = event.clientX - drag.x;
      const dy = event.clientY - drag.y;
      drag.x = event.clientX;
      drag.y = event.clientY;
      const view = rig.current;
      if (drag.mode === "orbit") {
        view.theta -= dx * 0.005;
        const nextPhi = view.phi + dy * 0.004;
        view.phi = Math.min(1.48, Math.max(0.22, nextPhi));
        return;
      }
      const scale = view.distance * 0.00115;
      offset.setFromSpherical(new Spherical(1, view.phi, view.theta));
      offset.negate();
      right.crossVectors(offset, new Vector3(0, 1, 0));
      if (right.lengthSq() < 1e-8) right.set(1, 0, 0);
      right.normalize();
      lift.crossVectors(right, offset).normalize();
      view.panX += (-right.x * dx + lift.x * dy) * scale;
      view.panY += (-right.y * dx + lift.y * dy) * scale;
      view.panZ += (-right.z * dx + lift.z * dy) * scale;
    }

    function endDrag() {
      active = false;
    }

    function wheel(event: WheelEvent) {
      event.preventDefault();
      const view = rig.current;
      const next = view.zoom * Math.exp(event.deltaY * 0.0014);
      view.zoom = Math.min(4, Math.max(0.28, next));
    }

    function menu(event: Event) {
      event.preventDefault();
    }

    el.addEventListener("pointerdown", down);
    el.addEventListener("pointermove", move);
    el.addEventListener("pointerup", endDrag);
    el.addEventListener("pointercancel", endDrag);
    el.addEventListener("wheel", wheel, { passive: false });
    el.addEventListener("contextmenu", menu);
    return () => {
      el.removeEventListener("pointerdown", down);
      el.removeEventListener("pointermove", move);
      el.removeEventListener("pointerup", endDrag);
      el.removeEventListener("pointercancel", endDrag);
      el.removeEventListener("wheel", wheel);
      el.removeEventListener("contextmenu", menu);
    };
  }, [gl]);

  useFrame((state) => {
    waterMat.uniforms.uTime.value = state.clock.elapsedTime;
    const current = latest.current;
    if (!current || !ship.current) return;

    ship.current.position.set(current.x, current.z, current.y);
    const leanX = Math.max(-0.18, Math.min(0.18, current.vy * 0.008));
    const leanZ = Math.max(-0.18, Math.min(0.18, -current.vx * 0.008));
    ship.current.rotation.x += (leanX - ship.current.rotation.x) * 0.08;
    ship.current.rotation.z += (leanZ - ship.current.rotation.z) * 0.08;

    const thrust = Math.max(0, current.fz / FLAME_FULL);
    const burning = thrust > 0.04;
    const power = burning ? 0.95 + Math.min(thrust, 1) * 0.4 : 0;
    sheathMat.uniforms.uTime.value = state.clock.elapsedTime;
    sheathMat.uniforms.uPower.value = power;
    coreMat.uniforms.uTime.value = state.clock.elapsedTime;
    coreMat.uniforms.uPower.value = power;
    if (flame.current) {
      const room = Math.max(5, current.z + 1.2);
      const fit = Math.min(1, room / FLAME_LENGTH);
      flame.current.scale.set(fit, fit, fit);

      // Nozzle points along the thrust. The tip then trails the velocity
      // that is left over sideways to that thrust.
      exhaust.current.set(-current.fx, -current.fz, -current.fy);
      if (exhaust.current.lengthSq() > 1) {
        exhaust.current.normalize();
        wake.current.set(-current.vx, -current.vz, -current.vy);
        const alongThrust = exhaust.current.dot(wake.current);
        wake.current.addScaledVector(exhaust.current, -alongThrust);
        const sideways = wake.current.length();
        if (sideways > 0.2) {
          const tip = Math.min(sideways * 0.4, 6.5) * fit;
          wake.current.multiplyScalar(tip / sideways);
        } else {
          wake.current.set(0, 0, 0);
        }
        sheathMat.uniforms.uBend.value.copy(wake.current);
        coreMat.uniforms.uBend.value.copy(wake.current);
        shipQuat.current.copy(ship.current.quaternion).invert();
        exhaust.current.applyQuaternion(shipQuat.current);
        flame.current.quaternion.setFromUnitVectors(plumeDown.current, exhaust.current);
      }
    }

    const low = current.z < 90;
    const lookY = low ? current.z + 18 : Math.max(20, current.z * 0.4);
    const rise = Math.max(36, current.z + 24 - lookY);
    const base = low ? 200 : Math.max(320, rise / Math.tan((9.5 * Math.PI) / 180));
    const view = rig.current;
    view.distance = base * view.zoom;
    const look = new Vector3(
      current.x * 0.68 + current.padX * 0.32 + view.panX,
      lookY + view.panY,
      current.y * 0.68 + current.padY * 0.32 + view.panZ,
    );
    const offset = new Vector3().setFromSpherical(new Spherical(view.distance, view.phi, view.theta));
    state.camera.position.copy(look).add(offset);
    state.camera.lookAt(look);
  });

  const padX = sample?.padX ?? 0;
  const padY = sample?.padY ?? 0;
  const yaw = padYaw(samples, time);
  const future = pointsOf(samples, time, true);
  const past = pointsOf(samples, time, false);
  const padFuture = padPoints(samples, time, true);
  const padPast = padPoints(samples, time, false);
  const touchdown = samples.length > 0 ? samples[samples.length - 1] : null;

  return (
    <>
      <color attach="background" args={["#8eacb8"]} />
      <fog attach="fog" args={["#7ea4b4", 2400, 7000]} />
      <mesh scale={4200} material={skyMat}>
        <sphereGeometry args={[1, 32, 16]} />
      </mesh>
      <hemisphereLight args={["#e7eef2", "#1a4a52", 1.15]} />
      <directionalLight position={[240, 280, 80]} intensity={2.4} color="#fff6ea" />
      <directionalLight position={[-120, 60, -160]} intensity={0.55} color="#b7d0dc" />
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]}>
        <planeGeometry args={[7000, 7000, 48, 48]} />
        <primitive object={waterMat} attach="material" />
      </mesh>
      <group position={[padX, 0, padY]} rotation={[0, yaw, 0]}>
        <mesh position={[0, -0.9, 0]}>
          <boxGeometry args={[54, 2.8, 26]} />
          <meshStandardMaterial color="#12181e" metalness={0.4} roughness={0.62} />
        </mesh>
        <mesh position={[0, 0.55, 0]}>
          <boxGeometry args={[50.4, 0.28, 22.4]} />
          <meshStandardMaterial color="#d5dde3" metalness={0.25} roughness={0.45} />
        </mesh>
        <mesh position={[0, 0.78, 0]}>
          <boxGeometry args={[47.6, 0.24, 19.6]} />
          <meshStandardMaterial color="#3a454c" metalness={0.3} roughness={0.72} />
        </mesh>
        <mesh position={[0, 0.96, 0]} rotation={[-Math.PI / 2, 0, 0]}>
          <ringGeometry args={[6.2, 7.8, 48]} />
          <meshBasicMaterial color="#f4f7f8" />
        </mesh>
        <mesh position={[-16, 1.7, 0]}>
          <boxGeometry args={[8, 2.4, 5]} />
          <meshStandardMaterial color="#e4eaee" metalness={0.25} roughness={0.4} />
        </mesh>
        <group position={[6, 1.15, 0]}>
          <mesh>
            <boxGeometry args={[16, 0.12, 0.45]} />
            <meshBasicMaterial color="#f2c14e" />
          </mesh>
          <mesh position={[9.2, 0, 0]} rotation={[0, 0, -Math.PI / 2]}>
            <coneGeometry args={[0.85, 2.2, 4]} />
            <meshBasicMaterial color="#f2c14e" />
          </mesh>
        </group>
      </group>
      {padPast.length > 1 ? <Line points={padPast} color="#f4f1e8" lineWidth={2.5} /> : null}
      {padFuture.length > 1 ? (
        <Line points={padFuture} color="#f2c14e" dashed dashSize={4} gapSize={3} lineWidth={2} />
      ) : null}
      {past.length > 1 ? <Line points={past} color="#8ea6b2" lineWidth={1.4} transparent opacity={0.4} /> : null}
      {future.length > 1 ? (
        <Line points={future} color="#f7fbfe" dashed dashSize={6} gapSize={4.5} lineWidth={2.2} />
      ) : null}
      {touchdown ? (
        <mesh position={[touchdown.x, Math.max(touchdown.z, 2.4), touchdown.y]} rotation={[0, Math.PI / 4, 0]}>
          <octahedronGeometry args={[1.6, 0]} />
          <meshBasicMaterial color="#f7fbfe" wireframe />
        </mesh>
      ) : null}
      <group ref={ship}>
        <Ship flameRef={flame} sheathMat={sheathMat} coreMat={coreMat} />
      </group>
    </>
  );
}

export function FlightView({
  sample,
  samples,
  time,
}: {
  sample: Sample | null;
  samples: Sample[];
  time: number;
}) {
  return (
    <Canvas camera={{ position: [80, 160, 420], fov: 32, near: 1, far: 12000 }} dpr={[1, 1.75]}>
      <Scene sample={sample} samples={samples} time={time} />
    </Canvas>
  );
}
