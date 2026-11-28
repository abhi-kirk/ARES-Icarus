import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { Line, OrbitControls, useTexture } from "@react-three/drei";
import { Suspense, useEffect, useMemo, useRef, type RefObject } from "react";
import {
  BackSide,
  InterleavedBufferAttribute,
  MirroredRepeatWrapping,
  MOUSE,
  Quaternion,
  ShaderMaterial,
  Spherical,
  SRGBColorSpace,
  Vector3,
} from "three";
import type { Group } from "three";
import { LineMaterial, LineSegments2, LineSegmentsGeometry, type OrbitControls as OrbitRig } from "three-stdlib";
import { flameMaterial, FLAME_LENGTH } from "./flight/flame";
import { LandingPicture } from "./flight/LandingPicture";
import { Ship } from "./flight/Ship";
import { sampleAt, type Sample } from "./flight/samples";
import { touchOf } from "./flight/touch";

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
  uniform float uMoon;
  void main() {
    float h = normalize(vDir).y;
    vec3 topDay = vec3(0.05, 0.1, 0.16);
    vec3 horizonDay = vec3(0.55, 0.68, 0.74);
    vec3 topNight = vec3(0.03, 0.034, 0.05);
    vec3 horizonNight = vec3(0.22, 0.24, 0.27);
    vec3 top = mix(topDay, topNight, uMoon);
    vec3 horizon = mix(horizonDay, horizonNight, uMoon);
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

/** MPC predicted path still ahead (engine frame x,z,y → three.js x,y,z). */
function planPoints(sample: Sample | null): [number, number, number][] {
  const trail = sample?.planTrail;
  if (!trail || trail.length < 6) return [];
  const out: [number, number, number][] = [];
  for (let i = 0; i + 2 < trail.length; i += 3) {
    out.push([trail[i], Math.max(trail[i + 2], 2), trail[i + 1]]);
  }
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

// One steady wind plus a sine gust, the same sum the plant uses.
// The streaks are drawn faster than those meters so the flow still reads
// from the landing camera. Direction and the gust stay in proportion.
const WIND_SHOW = 8;
const WIND_COUNT = 72;
const WIND_SPAN = 220;

function unitHash(n: number): number {
  const value = Math.sin(n * 127.1) * 43758.5453;
  return value - Math.floor(value);
}

function wrapSpan(value: number, span: number): number {
  const half = span * 0.5;
  return ((value + half) % span + span) % span - half;
}

function WindStreaks({
  x,
  y,
  z,
  time,
  windX,
  windY,
  gust,
  gustHz,
}: {
  x: number;
  y: number;
  z: number;
  time: number;
  windX: number;
  windY: number;
  gust: number;
  gustHz: number;
}) {
  const size = useThree((state) => state.size);
  const seeds = useMemo(() => {
    const next = new Float32Array(WIND_COUNT * 3);
    for (let i = 0; i < WIND_COUNT; i += 1) {
      next[i * 3] = (unitHash(i + 1) - 0.5) * WIND_SPAN;
      next[i * 3 + 1] = (unitHash(i + 19) - 0.5) * WIND_SPAN;
      next[i * 3 + 2] = unitHash(i + 47);
    }
    return next;
  }, []);

  const field = useMemo(() => {
    const geometry = new LineSegmentsGeometry();
    const positions = new Float32Array(WIND_COUNT * 6);
    geometry.setPositions(positions);
    const colors = new Float32Array(WIND_COUNT * 8);
    for (let i = 0; i < WIND_COUNT; i += 1) {
      const offset = i * 8;
      colors[offset] = 0.82;
      colors[offset + 1] = 0.9;
      colors[offset + 2] = 0.94;
      colors[offset + 3] = 0;
      colors[offset + 4] = 0.94;
      colors[offset + 5] = 0.97;
      colors[offset + 6] = 0.99;
      colors[offset + 7] = 1;
    }
    geometry.setColors(colors, 4);
    const material = new LineMaterial({
      color: 0xffffff,
      vertexColors: true,
      linewidth: 2,
      transparent: true,
      opacity: 0.38,
      depthWrite: false,
    });
    const mesh = new LineSegments2(geometry, material);
    mesh.frustumCulled = false;
    return { mesh, positions };
  }, []);

  useEffect(() => {
    return () => {
      field.mesh.geometry.dispose();
      const material = field.mesh.material;
      if (material instanceof LineMaterial) material.dispose();
    };
  }, [field]);

  useFrame(() => {
    const gustNow = gustHz > 0.001 ? gust * Math.sin(2 * Math.PI * gustHz * time) : 0;
    const wx = windX + gustNow;
    const wy = windY + gustNow;
    const speed = Math.hypot(wx, wy);
    const dirX = speed > 0.05 ? wx / speed : 1;
    const dirZ = speed > 0.05 ? wy / speed : 0;
    let gustDrift = 0;
    if (gustHz > 0.001) {
      gustDrift = (gust * (1 - Math.cos(2 * Math.PI * gustHz * time))) / (2 * Math.PI * gustHz);
    }
    const driftX = (windX * time + gustDrift) * WIND_SHOW;
    const driftZ = (windY * time + gustDrift) * WIND_SHOW;
    const visual = speed * WIND_SHOW;
    const length = Math.min(Math.max(22, visual * 1.1), 64);
    const material = field.mesh.material;
    if (material instanceof LineMaterial) {
      material.resolution.set(size.width, size.height);
      material.opacity = Math.min(0.3 + speed * 0.015, 0.45);
    }

    const mid = Math.max(z, 24);
    const low = Math.max(8, mid - 70);
    const high = mid + 48;
    const buffer = field.positions;
    for (let i = 0; i < WIND_COUNT; i += 1) {
      const hx = wrapSpan(seeds[i * 3] + driftX, WIND_SPAN);
      const hz = wrapSpan(seeds[i * 3 + 1] + driftZ, WIND_SPAN);
      const hy = low + seeds[i * 3 + 2] * (high - low);
      const tail = i * 6;
      buffer[tail] = hx - dirX * length;
      buffer[tail + 1] = hy;
      buffer[tail + 2] = hz - dirZ * length;
      buffer[tail + 3] = hx;
      buffer[tail + 4] = hy;
      buffer[tail + 5] = hz;
    }
    const start = field.mesh.geometry.getAttribute("instanceStart");
    if (start instanceof InterleavedBufferAttribute) start.data.needsUpdate = true;
  });

  return (
    <group position={[x, 0, y]}>
      <primitive object={field.mesh} />
    </group>
  );
}

// The Apollo Basin frame is 880 m across 1000 pixels, so 0.88 m per pixel.
// The ground plane is 7 km. Repeating the photo at that scale keeps the craters sharp.
const METERS_PER_PIXEL = 0.88;
const FIELD_M = 7000;

function MoonGround() {
  const texture = useTexture("/moon/apollo-basin.png");
  const { gl } = useThree();
  useEffect(() => {
    const pixels = texture.image.width as number;
    const copies = FIELD_M / (pixels * METERS_PER_PIXEL);
    texture.colorSpace = SRGBColorSpace;
    texture.wrapS = MirroredRepeatWrapping;
    texture.wrapT = MirroredRepeatWrapping;
    texture.repeat.set(copies, copies);
    texture.anisotropy = gl.capabilities.getMaxAnisotropy();
    texture.needsUpdate = true;
  }, [texture, gl]);
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]}>
      <planeGeometry args={[FIELD_M, FIELD_M]} />
      <meshStandardMaterial
        map={texture}
        bumpMap={texture}
        bumpScale={2.5}
        roughness={0.9}
        metalness={0}
        color="#f4f1ea"
      />
    </mesh>
  );
}

function frameOf(sample: Sample | null) {
  const z = sample?.z ?? 120;
  const low = z < 90;
  const lookY = low ? z + 18 : Math.max(20, z * 0.4);
  const rise = Math.max(36, z + 24 - lookY);
  const distance = low ? 200 : Math.max(320, rise / Math.tan((9.5 * Math.PI) / 180));
  const x = sample ? sample.x * 0.68 + sample.padX * 0.32 : 0;
  const ground = sample ? sample.y * 0.68 + sample.padY * 0.32 : 0;
  return { x, y: lookY, z: ground, distance };
}

function FlightCamera({ sample }: { sample: RefObject<Sample | null> }) {
  const controls = useRef<OrbitRig>(null);
  const camera = useThree((state) => state.camera);
  const gl = useThree((state) => state.gl);
  const placed = useRef(false);
  const lastLook = useRef(new Vector3());
  const lastRadius = useRef(1);
  const zoom = useRef(1);
  const offset = useRef(new Vector3());
  const keys = useRef({ meta: false, ctrl: false, shift: false });

  useEffect(() => {
    const el = gl.domElement;
    function syncKey(event: KeyboardEvent) {
      const down = event.type === "keydown";
      keys.current.meta = event.key === "Meta" ? down : event.metaKey;
      keys.current.ctrl = event.key === "Control" ? down : event.ctrlKey;
      keys.current.shift = event.key === "Shift" ? down : event.shiftKey;
    }
    function blur() {
      keys.current.meta = false;
      keys.current.ctrl = false;
      keys.current.shift = false;
    }
    // OrbitControls reads Command off the pointer event. A Mac often leaves
    // that flag off, so the held keys decide the button before the control runs.
    function choose(event: PointerEvent) {
      const rig = controls.current;
      if (!rig) return;
      const held =
        keys.current.meta ||
        keys.current.ctrl ||
        keys.current.shift ||
        event.getModifierState("Meta") ||
        event.getModifierState("Control") ||
        event.getModifierState("Shift");
      const onEvent = event.metaKey || event.ctrlKey || event.shiftKey;
      const pan = held || onEvent;
      if (event.button === 0) {
        rig.mouseButtons.LEFT = pan && !onEvent ? MOUSE.PAN : MOUSE.ROTATE;
      } else if (event.button === 2) {
        rig.mouseButtons.RIGHT = onEvent ? MOUSE.ROTATE : MOUSE.PAN;
      }
    }
    function resetButtons() {
      const rig = controls.current;
      if (!rig) return;
      rig.mouseButtons.LEFT = MOUSE.ROTATE;
      rig.mouseButtons.RIGHT = MOUSE.PAN;
    }
    window.addEventListener("keydown", syncKey);
    window.addEventListener("keyup", syncKey);
    window.addEventListener("blur", blur);
    el.addEventListener("pointerdown", choose, true);
    el.addEventListener("pointerup", resetButtons, true);
    el.addEventListener("pointercancel", resetButtons, true);
    return () => {
      window.removeEventListener("keydown", syncKey);
      window.removeEventListener("keyup", syncKey);
      window.removeEventListener("blur", blur);
      el.removeEventListener("pointerdown", choose, true);
      el.removeEventListener("pointerup", resetButtons, true);
      el.removeEventListener("pointercancel", resetButtons, true);
    };
  }, [gl]);

  useFrame(() => {
    const rig = controls.current;
    if (!rig) return;
    const frame = frameOf(sample.current);
    if (!placed.current) {
      rig.target.set(frame.x, frame.y, frame.z);
      offset.current.setFromSpherical(new Spherical(frame.distance, 1.33, 0.6));
      camera.position.copy(rig.target).add(offset.current);
      lastLook.current.set(frame.x, frame.y, frame.z);
      lastRadius.current = frame.distance;
      placed.current = true;
      return;
    }
    const dx = frame.x - lastLook.current.x;
    const dy = frame.y - lastLook.current.y;
    const dz = frame.z - lastLook.current.z;
    rig.target.x += dx;
    rig.target.y += dy;
    rig.target.z += dz;
    camera.position.x += dx;
    camera.position.y += dy;
    camera.position.z += dz;
    lastLook.current.set(frame.x, frame.y, frame.z);

    offset.current.copy(camera.position).sub(rig.target);
    const len = Math.max(offset.current.length(), 1);
    const ratio = len / lastRadius.current;
    if (Math.abs(ratio - 1) > 0.002) {
      zoom.current = Math.min(4, Math.max(0.28, zoom.current * ratio));
    }
    const next = frame.distance * zoom.current;
    offset.current.multiplyScalar(next / len);
    camera.position.copy(rig.target).add(offset.current);
    lastRadius.current = next;
  });

  return (
    <OrbitControls
      ref={controls}
      enableDamping={false}
      enablePan
      minPolarAngle={0.22}
      maxPolarAngle={1.48}
      minDistance={40}
      maxDistance={5000}
    />
  );
}

function Scene({
  sample,
  samples,
  time,
  world,
  windX,
  windY,
  gust,
  gustHz,
  thrustFull,
}: {
  sample: Sample | null;
  samples: Sample[];
  time: number;
  world: "earth" | "moon";
  windX: number;
  windY: number;
  gust: number;
  gustHz: number;
  thrustFull: number;
}) {
  const latest = useRef(sample);
  latest.current = sample;
  const ship = useRef<Group>(null);
  const flame = useRef<Group>(null);
  const exhaust = useRef(new Vector3());
  const wake = useRef(new Vector3());
  const plumeDown = useRef(new Vector3(0, -1, 0));
  const shipQuat = useRef(new Quaternion());
  const tipAxis = useRef(new Vector3());
  const aftermath = useRef(0);
  const parkedAt = useRef(-1);
  const touch = useMemo(() => touchOf(samples), [samples]);
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
        uniforms: { uMoon: { value: 0 } },
        vertexShader: SKY_VERT,
        fragmentShader: SKY_FRAG,
      }),
    [],
  );

  useFrame((state) => {
    waterMat.uniforms.uTime.value = state.clock.elapsedTime;
    const current = latest.current;
    if (!current || !ship.current) return;

    ship.current.position.set(current.x, current.z, current.y);
    const end = samples.length > 0 ? samples[samples.length - 1].t : 0;
    const parked = touch !== null && time >= end - 0.05;
    if (!parked) {
      aftermath.current = 0;
      parkedAt.current = -1;
    } else if (parkedAt.current < 0) {
      parkedAt.current = state.clock.elapsedTime;
      aftermath.current = 0;
    } else {
      aftermath.current = state.clock.elapsedTime - parkedAt.current;
    }

    const leanX = Math.max(-0.18, Math.min(0.18, current.vy * 0.008));
    const leanZ = Math.max(-0.18, Math.min(0.18, -current.vx * 0.008));
    const tipping = touch !== null && touch.kind === "hard" && aftermath.current > 0;
    if (tipping && touch) {
      const sideways = Math.hypot(touch.vx, touch.vy);
      const alongX = sideways > 0.4 ? touch.vx / sideways : 1;
      const alongZ = sideways > 0.4 ? touch.vy / sideways : 0;
      tipAxis.current.set(alongZ, 0, -alongX);
      const full = Math.min(1.15, 0.5 + sideways * 0.035);
      const u = Math.min(1, aftermath.current / 1.2);
      const ease = u * u * (3 - 2 * u);
      ship.current.quaternion.setFromAxisAngle(tipAxis.current, full * ease);
    } else {
      ship.current.rotation.x += (leanX - ship.current.rotation.x) * 0.08;
      ship.current.rotation.y += (0 - ship.current.rotation.y) * 0.2;
      ship.current.rotation.z += (leanZ - ship.current.rotation.z) * 0.08;
    }

    const settled = touch !== null && current.t >= touch.t - 1e-3;
    const thrust = Math.max(0, current.fz / Math.max(thrustFull, 1));
    const burning = !settled && thrust > 0.04;
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

  });

  const padX = sample?.padX ?? 0;
  const padY = sample?.padY ?? 0;
  const yaw = padYaw(samples, time);
  const moon = world === "moon";
  const windy = !moon && (Math.hypot(windX, windY) > 0.15 || gust > 0.15);
  skyMat.uniforms.uMoon.value = moon ? 1 : 0;
  const future = pointsOf(samples, time, true);
  const past = pointsOf(samples, time, false);
  const plan = planPoints(sample);
  const padFuture = padPoints(samples, time, true);
  const padPast = padPoints(samples, time, false);
  const touchdown = samples.length > 0 ? samples[samples.length - 1] : null;

  return (
    <>
      <FlightCamera sample={latest} />
      <color attach="background" args={[moon ? "#14181e" : "#8eacb8"]} />
      <fog attach="fog" args={[moon ? "#2a3036" : "#7ea4b4", moon ? 2200 : 2400, moon ? 7400 : 7000]} />
      <mesh scale={4200} material={skyMat}>
        <sphereGeometry args={[1, 32, 16]} />
      </mesh>
      <hemisphereLight args={[moon ? "#d5dde6" : "#e7eef2", moon ? "#8a8176" : "#1a4a52", moon ? 0.95 : 1.15]} />
      <directionalLight position={[240, 280, 80]} intensity={moon ? 2.7 : 2.4} color="#fff6ea" />
      <directionalLight position={[-120, 60, -160]} intensity={0.55} color="#b7d0dc" />
      {moon ? (
        <Suspense fallback={null}>
          <MoonGround />
        </Suspense>
      ) : (
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]}>
          <planeGeometry args={[7000, 7000, 48, 48]} />
          <primitive object={waterMat} attach="material" />
        </mesh>
      )}
      {windy ? (
        <WindStreaks
          x={sample?.x ?? 0}
          y={sample?.y ?? 0}
          z={sample?.z ?? 120}
          time={time}
          windX={windX}
          windY={windY}
          gust={gust}
          gustHz={gustHz}
        />
      ) : null}
      <group position={[padX, 0, padY]} rotation={[0, yaw, 0]}>
        {moon ? (
          <>
            <mesh position={[0, 0.08, 0]} rotation={[-Math.PI / 2, 0, 0]}>
              <circleGeometry args={[16, 48]} />
              <meshStandardMaterial color="#6a6560" roughness={0.92} metalness={0.04} />
            </mesh>
            <mesh position={[0, 0.12, 0]} rotation={[-Math.PI / 2, 0, 0]}>
              <ringGeometry args={[6.2, 7.8, 48]} />
              <meshBasicMaterial color="#f4f7f8" />
            </mesh>
          </>
        ) : (
          <>
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
          </>
        )}
      </group>
      {!moon && padPast.length > 1 ? <Line points={padPast} color="#f4f1e8" lineWidth={2.5} /> : null}
      {!moon && padFuture.length > 1 ? (
        <Line points={padFuture} color="#f2c14e" dashed dashSize={4} gapSize={3} lineWidth={2} />
      ) : null}
      {past.length > 1 ? <Line points={past} color="#8ea6b2" lineWidth={1.4} transparent opacity={0.4} /> : null}
      {future.length > 1 ? (
        <Line points={future} color="#f7fbfe" dashed dashSize={6} gapSize={4.5} lineWidth={2.2} />
      ) : null}
      {plan.length > 1 ? (
        <Line points={plan} color="#f2c14e" dashed dashSize={5} gapSize={3.5} lineWidth={2.4} />
      ) : null}
      {touchdown ? (
        <mesh position={[touchdown.x, Math.max(touchdown.z, 2.4), touchdown.y]} rotation={[0, Math.PI / 4, 0]}>
          <octahedronGeometry args={[1.6, 0]} />
          <meshBasicMaterial color="#f7fbfe" wireframe />
        </mesh>
      ) : null}
      <group ref={ship}>
        <Suspense fallback={null}>
          <Ship flameRef={flame} sheathMat={sheathMat} coreMat={coreMat} />
        </Suspense>
      </group>
      {touch ? (
        <LandingPicture
          key={`${touch.kind}:${touch.t}:${world}`}
          touch={touch}
          world={world}
          age={aftermath}
        />
      ) : null}
    </>
  );
}

export function FlightView({
  sample,
  samples,
  time,
  world,
  windX,
  windY,
  gust,
  gustHz,
  thrustFull,
}: {
  sample: Sample | null;
  samples: Sample[];
  time: number;
  world: "earth" | "moon";
  windX: number;
  windY: number;
  gust: number;
  gustHz: number;
  thrustFull: number;
}) {
  return (
    <Canvas camera={{ position: [80, 160, 420], fov: 32, near: 1, far: 12000 }} dpr={[1, 1.75]}>
      <Scene
        sample={sample}
        samples={samples}
        time={time}
        world={world}
        windX={windX}
        windY={windY}
        gust={gust}
        gustHz={gustHz}
        thrustFull={thrustFull}
      />
    </Canvas>
  );
}
