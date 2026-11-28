import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { OrbitControls, useGLTF, useTexture } from "@react-three/drei";
import { Atmosphere, Sky, SkyLight, SunLight, type AtmosphereApi } from "@takram/three-atmosphere/r3f";
import { Ellipsoid } from "@takram/three-geospatial";
import { Suspense, useEffect, useMemo, useRef, useState, type ComponentRef } from "react";
import {
  BufferAttribute,
  BufferGeometry,
  Color,
  Group,
  AdditiveBlending,
  CylinderGeometry,
  DoubleSide,
  Mesh,
  NormalBlending,
  MeshStandardMaterial,
  Quaternion,
  ShaderMaterial,
  SRGBColorSpace,
  Vector3,
  type Material,
  type Object3D,
} from "three";
import { flameMaterial, FLAME_LENGTH } from "../flight/flame";
import { Ship } from "../flight/Ship";
import { parseSample, sampleAt, type Sample } from "../flight/samples";

const NOON = new Date(Date.UTC(2026, 5, 21, 12, 0, 0));
const EARTH_RADIUS = 6378137;
const SHIP_FILE_BOTTOM = 23.95;
const SHIP_FILE_TOP = 41.32;
const SHIP_HEIGHT = 50;
// The insertion recording burns at 100 kN. The return brake is 25 kN.
const CLIMB_THRUST_N = 100000;
const RETURN_THRUST_N = 25000;

function noseOf(sample: Sample, out: Vector3): boolean {
  if (
    sample.qw === undefined ||
    sample.qx === undefined ||
    sample.qy === undefined ||
    sample.qz === undefined
  ) {
    return false;
  }
  const w = sample.qw;
  const x = sample.qx;
  const y = sample.qy;
  const z = sample.qz;
  // Body +z is the nose. Plant axes are east, north, up. The scene is
  // north, up, east.
  const east = 2 * (x * z + w * y);
  const north = 2 * (y * z - w * x);
  const up = 1 - 2 * (x * x + y * y);
  out.set(north, up, east);
  const length = out.length();
  if (length < 0.5) return false;
  out.multiplyScalar(1 / length);
  return true;
}

function alongVelocity(sample: Sample): number {
  const nose = new Vector3();
  if (!noseOf(sample, nose)) return 1;
  const speed = Math.hypot(sample.vx, sample.vy, sample.vz);
  if (speed < 1) return 1;
  return (nose.z * sample.vx + nose.x * sample.vy + nose.y * sample.vz) / speed;
}

// sqrt(rho) * v^3, the same total the plant integrates for the shield.
// The 1 km/s coast peaks near 6e8. The hot entry climbs through 1e10.
function heatGlow(sample: Sample | null): number {
  if (!sample || sample.rho === undefined) return 0;
  const speed = Math.hypot(sample.vx, Math.hypot(sample.vy, sample.vz));
  const rate = Math.sqrt(sample.rho) * speed * speed * speed;
  return Math.min(1, Math.max(0, (rate - 7e8) / 7e9));
}

function smoothstep(edge0: number, edge1: number, value: number): number {
  const span = edge1 - edge0;
  const t = Math.min(1, Math.max(0, (value - edge0) / span));
  return t * t * (3 - 2 * t);
}

export function playbackRate(sample: Sample | null, time: number): number {
  // The return opens at real time, eases up to 6× for the long sideways
  // fall, and eases back toward 2× as the ship slows. Real time covers the
  // nose swing (the side comes off the wind just after 328 s) and, once
  // the ship is under about 6 km, the flip and the landing. Every change
  // is a slope.
  if (!sample) return 1;
  const speed = Math.hypot(sample.vx, Math.hypot(sample.vy, sample.vz));
  const opening = smoothstep(12, 26, time);
  const stillFast = smoothstep(2200, 6000, speed);
  const cruise = 2 + 4 * stillFast;
  const aboveTheFlip = smoothstep(6000, 15000, sample.z);
  const paced = 1 + (cruise - 1) * aboveTheFlip;
  const eased = 1 + (paced - 1) * opening;
  const swing = smoothstep(326, 328.2, time) * (1 - smoothstep(332.5, 338, time));
  return eased + (1 - eased) * swing;
}

export function phaseOf(sample: Sample | null, _time: number): string {
  if (!sample) return "Entry";
  const thrust = Math.hypot(sample.fx, sample.fy, sample.fz);
  const speed = Math.hypot(sample.vx, Math.hypot(sample.vy, sample.vz));
  if (thrust > 1000 && sample.z < 700) return "Landing";
  if (thrust > 1000) return "Brake";
  // The belly points the nose across the wind, and just after 2 km/s the
  // nose swings onto the velocity while the ship is still near 40 km.
  // That swing stays Entry. The flip is the later turn, under 6 km.
  if (speed < 2000 && sample.z < 6000 && alongVelocity(sample) < 0.85) return "Flip";
  return "Entry";
}

// Yale Bright Star Catalog, the same file takram ships. Its own shader writes
// GLSL3 and never appears with the logarithmic depth buffer, so the points
// are drawn with a stock material on a shell around the ship.
function StarField() {
  const [geometry, setGeometry] = useState<BufferGeometry | null>(null);
  useEffect(() => {
    let cancel = false;
    fetch("/sky/stars.bin")
      .then((response) => response.arrayBuffer())
      .then((data) => {
        if (cancel) return;
        const raw = new Int16Array(data);
        const count = Math.floor(raw.length / 5);
        const positions = new Float32Array(count * 3);
        const radius = 2_000_000;
        for (let i = 0; i < count; i += 1) {
          const base = i * 5;
          positions[i * 3] = (raw[base] / 32767) * radius;
          positions[i * 3 + 1] = (raw[base + 1] / 32767) * radius;
          positions[i * 3 + 2] = (raw[base + 2] / 32767) * radius;
        }
        const next = new BufferGeometry();
        next.setAttribute("position", new BufferAttribute(positions, 3));
        setGeometry(next);
      })
      .catch(() => setGeometry(null));
    return () => {
      cancel = true;
    };
  }, []);
  if (!geometry) return null;
  return (
    <points geometry={geometry} frustumCulled={false} renderOrder={-1}>
      <pointsMaterial color="#f7f4ee" size={2.4} sizeAttenuation={false} depthWrite={false} />
    </points>
  );
}

function EarthSurface({ altitude }: { altitude: number }) {
  const [color, clouds] = useTexture(["/earth/blue-marble.jpg", "/earth/clouds.jpg"]);
  color.colorSpace = SRGBColorSpace;
  clouds.colorSpace = SRGBColorSpace;
  // Poles are the sphere's local Y. The ship sits on the +Y side of the
  // planet's center. This turn puts the ship over the Atlantic, just off
  // the Amazon, so the disk shows ocean, the coast, and cloud.
  const spin: [number, number, number] = [-Math.PI / 2, -0.73, 0];
  return (
    <group frustumCulled={false} position={[0, -(EARTH_RADIUS + altitude), 0]} rotation={spin}>
      <mesh frustumCulled={false}>
        <sphereGeometry args={[EARTH_RADIUS, 192, 96]} />
        {/* Blue Marble is already a daylight photo. A lit material darkens it. */}
        <meshBasicMaterial map={color} />
      </mesh>
      {/* NASA's cloud map is white on black. The black drops out, and the
          shell sits above the surface so it reads from orbit. */}
      <mesh frustumCulled={false} renderOrder={1}>
        <sphereGeometry args={[EARTH_RADIUS + 14000, 192, 96]} />
        <meshBasicMaterial color="#f7f8fa" alphaMap={clouds} transparent depthWrite={false} opacity={0.85} />
      </mesh>
    </group>
  );
}

function calmSteel(material: Material): Material {
  const next = material.clone() as MeshStandardMaterial;
  if (typeof next.metalness === "number" && next.metalness > 0.4) next.metalness = 0.32;
  if (typeof next.roughness === "number" && next.roughness < 0.35) next.roughness = 0.4;
  return next;
}

// Windward orange, violet at the rim. The shape follows the hull. Brightness
// is the plant's heat rate. The lit face is the one along the velocity,
// which is the black tile side after the ship rolls that side forward.
function sheathMaterial(): ShaderMaterial {
  return new ShaderMaterial({
    transparent: true,
    depthWrite: false,
    blending: AdditiveBlending,
    uniforms: {
      glow: { value: 0 },
      flow: { value: new Vector3(0, 0, 1) },
      time: { value: 0 },
    },
    vertexShader: `
      #include <common>
      #include <logdepthbuf_pars_vertex>
      varying vec3 vNormalW;
      varying vec3 vWorld;
      void main() {
        vec4 world = modelMatrix * vec4(position, 1.0);
        vWorld = world.xyz;
        vNormalW = normalize(mat3(modelMatrix) * normal);
        gl_Position = projectionMatrix * viewMatrix * world;
        #include <logdepthbuf_vertex>
      }
    `,
    fragmentShader: `
      #include <common>
      #include <logdepthbuf_pars_fragment>
      uniform float glow;
      uniform vec3 flow;
      uniform float time;
      varying vec3 vNormalW;
      varying vec3 vWorld;
      void main() {
        vec3 normal = normalize(vNormalW);
        vec3 wind = normalize(flow);
        float face = clamp(dot(normal, wind), 0.0, 1.0);
        float mask = smoothstep(-0.05, 0.55, face);
        vec3 viewDir = normalize(cameraPosition - vWorld);
        float edge = pow(1.0 - abs(dot(normal, viewDir)), 3.0) * (1.0 - mask);
        float streak = 0.78 + 0.22 * sin(dot(vWorld, wind) * 0.35 - time * 5.0);
        vec3 hot = vec3(1.0, 0.42, 0.08);
        vec3 ion = vec3(0.62, 0.22, 0.95);
        vec3 color = mix(ion, hot, mask);
        float alpha = glow * streak * (mask * 1.35 + edge * 0.3);
        gl_FragColor = vec4(color * alpha, 1.0);
        #include <logdepthbuf_fragment>
      }
    `,
  });
}

function wakeMaterial(): ShaderMaterial {
  return new ShaderMaterial({
    transparent: true,
    depthWrite: false,
    blending: AdditiveBlending,
    side: DoubleSide,
    uniforms: {
      glow: { value: 0 },
    },
    vertexShader: `
      #include <common>
      #include <logdepthbuf_pars_vertex>
      varying float vAlong;
      void main() {
        vAlong = uv.y;
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        #include <logdepthbuf_vertex>
      }
    `,
    fragmentShader: `
      #include <common>
      #include <logdepthbuf_pars_fragment>
      uniform float glow;
      varying float vAlong;
      void main() {
        float fade = pow(1.0 - clamp(vAlong, 0.0, 1.0), 1.6);
        vec3 color = mix(vec3(0.45, 0.16, 0.75), vec3(1.0, 0.4, 0.1), fade * 0.35);
        float alpha = glow * fade * 0.12;
        gl_FragColor = vec4(color * alpha, 1.0);
        #include <logdepthbuf_fragment>
      }
    `,
  });
}

function Plasma({ glow, flow }: { glow: number; flow: Vector3 }) {
  const gltf = useGLTF("/models/sn24/scene.gltf");
  const shell = useMemo(() => sheathMaterial(), []);
  const wake = useMemo(() => wakeMaterial(), []);
  const wakeMesh = useRef<Mesh>(null);
  const wakeUp = useMemo(() => new Vector3(0, 1, 0), []);
  const localFlow = useMemo(() => new Vector3(), []);
  const model = useMemo(() => {
    const copy = gltf.scene.clone(true);
    const booster = copy.getObjectByName("Superheavy_4");
    if (booster) booster.visible = false;
    copy.traverse((obj: Object3D) => {
      const mesh = obj as Mesh;
      if (!mesh.isMesh) return;
      mesh.material = shell;
    });
    return copy;
  }, [gltf.scene, shell]);
  useFrame((state) => {
    shell.uniforms.glow.value = glow;
    // The face in front, along the velocity, is the one in the air.
    shell.uniforms.flow.value.copy(flow);
    shell.uniforms.time.value = state.clock.elapsedTime;
    wake.uniforms.glow.value = glow;
    const wakeNode = wakeMesh.current;
    const shipGroup = wakeNode?.parent?.parent;
    if (wakeNode && shipGroup) {
      localFlow.copy(flow).negate().applyQuaternion(shipGroup.quaternion.clone().invert());
      if (localFlow.lengthSq() > 0.25) {
        localFlow.normalize();
        wakeNode.quaternion.setFromUnitVectors(wakeUp, localFlow);
        wakeNode.position.copy(localFlow).multiplyScalar(16);
      }
    }
  });
  const scale = SHIP_HEIGHT / (SHIP_FILE_TOP - SHIP_FILE_BOTTOM);
  return (
    <group visible={glow > 0.001}>
      <group scale={1.05}>
        <primitive
          object={model}
          scale={scale}
          position={[0.39 * scale, -SHIP_FILE_BOTTOM * scale, 0.149 * scale]}
        />
      </group>
      <mesh ref={wakeMesh} material={wake}>
        <coneGeometry args={[3.2, 22, 16]} />
      </mesh>
    </group>
  );
}

// Compressed nitrogen. A webcast jet is white vapor: dense at the throat,
// ragged and widening as it leaves, gone a short way out. The mesh is only
// a volume to draw into. The shader throws away the tube edge.
function gasMaterial(): ShaderMaterial {
  return new ShaderMaterial({
    transparent: true,
    depthWrite: false,
    depthTest: true,
    toneMapped: false,
    blending: NormalBlending,
    side: DoubleSide,
    uniforms: {
      time: { value: 0 },
      power: { value: 0 },
    },
    vertexShader: `
      #include <common>
      #include <logdepthbuf_pars_vertex>
      uniform float time;
      varying float vAlong;
      varying vec3 vViewNormal;
      void main() {
        vAlong = uv.y;
        float around = uv.x * 6.28318;
        float billow = sin(around * 3.0 + time * 3.4 + vAlong * 8.0);
        float wisp = sin(around * 7.0 - time * 2.1 + vAlong * 15.0);
        vec3 pos = position;
        pos.xz *= 1.0 + vAlong * vAlong * (0.34 * billow + 0.16 * wisp);
        vec4 viewPosition = modelViewMatrix * vec4(pos, 1.0);
        vViewNormal = normalize(normalMatrix * normal);
        gl_Position = projectionMatrix * viewPosition;
        #include <logdepthbuf_vertex>
      }
    `,
    fragmentShader: `
      #include <common>
      #include <logdepthbuf_pars_fragment>
      uniform float time;
      uniform float power;
      varying float vAlong;
      varying vec3 vViewNormal;

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
        float facing = abs(vViewNormal.z);
        float body = smoothstep(0.02, 0.42, facing);
        float flow = vAlong * 6.5 - time * 2.4;
        float streak = noise(vec2(vAlong * 2.0 + facing, flow));
        float grain = noise(vec2(vViewNormal.x * 4.0 + time * 0.3, flow * 1.7));
        float vapor = body * (0.28 + 0.72 * streak) * (0.45 + 0.55 * grain);
        float throat = smoothstep(0.0, 0.12, vAlong);
        float tipBreak = noise(vec2(vAlong * 3.0, time * 0.6));
        float tip = smoothstep(1.0, 0.05, vAlong + 0.22 * tipBreak * vAlong);
        float core = pow(facing, 2.0) * pow(1.0 - vAlong, 1.6);
        float alpha = (vapor * tip * throat * 0.78 + core * 0.5) * power;
        vec3 cold = vec3(0.74, 0.81, 0.86);
        vec3 bright = vec3(0.97, 0.98, 1.0);
        vec3 color = mix(cold, bright, clamp(core + streak * 0.35, 0.0, 1.0));
        gl_FragColor = vec4(color, clamp(alpha, 0.0, 0.82));
        #include <logdepthbuf_fragment>
      }
    `,
  });
}

const GAS_JETS = [
  { x: 4.6, y: 44, z: 0, dx: 1, dy: 0, dz: 0 },
  { x: -4.6, y: 44, z: 0, dx: -1, dy: 0, dz: 0 },
  { x: 0, y: 44, z: 4.6, dx: 0, dy: 0, dz: 1 },
  { x: 0, y: 44, z: -4.6, dx: 0, dy: 0, dz: -1 },
  { x: 4.6, y: 8, z: 0, dx: 1, dy: 0, dz: 0 },
  { x: -4.6, y: 8, z: 0, dx: -1, dy: 0, dz: 0 },
  { x: 0, y: 8, z: 4.6, dx: 0, dy: 0, dz: 1 },
  { x: 0, y: 8, z: -4.6, dx: 0, dy: 0, dz: -1 },
];

function ColdGas({
  jet,
}: {
  jet: { power: number; x: number; y: number; z: number };
}) {
  const meshes = useRef<(Mesh | null)[]>([]);
  const geometry = useMemo(() => {
    // Narrow at the throat, wide at the open end. The nozzle sits on the skin.
    const tube = new CylinderGeometry(3.1, 0.28, 22, 28, 22, true);
    tube.translate(0, 11, 0);
    return tube;
  }, []);
  const materials = useMemo(() => GAS_JETS.map(() => gasMaterial()), []);
  const up = useMemo(() => new Vector3(0, 1, 0), []);
  const dir = useMemo(() => new Vector3(), []);
  const lever = useMemo(() => new Vector3(), []);
  const torque = useMemo(() => new Vector3(), []);
  const spin = useMemo(() => new Vector3(), []);
  useFrame((state) => {
    spin.set(jet.x, jet.y, jet.z);
    const power = jet.power;
    for (let i = 0; i < GAS_JETS.length; i += 1) {
      const pose = GAS_JETS[i];
      const mesh = meshes.current[i];
      const mat = materials[i];
      if (!mesh || !mat) continue;
      mat.uniforms.time.value = state.clock.elapsedTime;
      lever.set(pose.x, pose.y - 25, pose.z);
      dir.set(pose.dx, pose.dy, pose.dz);
      torque.crossVectors(lever, dir);
      const leverLen = torque.length();
      const align = leverLen > 1e-6 ? torque.dot(spin) / leverLen : 0;
      const lit = power > 0.02 ? Math.max(0, align) * power : 0;
      mat.uniforms.power.value = lit;
      mesh.visible = lit > 0.06;
      const reach = 0.4 + lit * 0.85;
      const width = 0.55 + lit * 0.7;
      mesh.scale.set(width, reach, width);
      mesh.quaternion.setFromUnitVectors(up, dir);
    }
  });
  return (
    <group>
      {GAS_JETS.map((pose, i) => (
        <mesh
          key={`${pose.x}-${pose.y}-${pose.z}`}
          ref={(node) => {
            meshes.current[i] = node;
          }}
          geometry={geometry}
          material={materials[i]}
          position={[pose.x, pose.y, pose.z]}
          renderOrder={2}
        />
      ))}
    </group>
  );
}

function Booster() {
  const gltf = useGLTF("/models/sn24/scene.gltf");
  const model = useMemo(() => {
    // The file is one stack, and the root node turns the file axes into the
    // ship axes. Cloning only Superheavy drops that turn, which parks the
    // booster off the tail. Clone the whole file and keep the booster.
    const root = gltf.scene.clone(true);
    const booster = root.getObjectByName("Superheavy_4");
    const chain = new Set<Object3D>();
    let node: Object3D | null = booster ?? null;
    while (node) {
      chain.add(node);
      node = node.parent;
    }
    root.traverse((obj: Object3D) => {
      let underBooster = obj === booster;
      let parent = obj.parent;
      while (parent && !underBooster) {
        if (parent === booster) underBooster = true;
        parent = parent.parent;
      }
      if (!underBooster && !chain.has(obj)) obj.visible = false;
      const mesh = obj as Mesh;
      if (!mesh.isMesh) return;
      const list = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
      const cloned = list.map(calmSteel);
      mesh.material = Array.isArray(mesh.material) ? cloned : cloned[0];
    });
    return root;
  }, [gltf.scene]);
  const scale = SHIP_HEIGHT / (SHIP_FILE_TOP - SHIP_FILE_BOTTOM);
  return (
    <primitive
      object={model}
      scale={scale}
      position={[0.39 * scale, -SHIP_FILE_BOTTOM * scale, 0.149 * scale]}
    />
  );
}

function CoastScene({
  samples,
  time,
  altitude,
  glow,
  showBooster,
  followCamera = true,
  thrustFull = RETURN_THRUST_N,
}: {
  samples: Sample[];
  time: number;
  altitude: number;
  glow: number;
  showBooster: boolean;
  followCamera?: boolean;
  thrustFull?: number;
}) {
  const sky = useRef<AtmosphereApi>(null);
  const skyMesh = useRef<ComponentRef<typeof Sky>>(null);
  const ship = useRef<Group>(null);
  const booster = useRef<Group>(null);
  const flame = useRef<Group>(null);
  const ecef = useMemo(() => new Vector3(), []);
  const velocity = useMemo(() => new Vector3(), []);
  const nose = useMemo(() => new Vector3(), []);
  const noseAxis = useMemo(() => new Vector3(0, 1, 0), []);
  const horizontal = useMemo(() => new Vector3(), []);
  const flank = useMemo(() => new Vector3(), []);
  const behind = useMemo(() => new Vector3(), []);
  const sheath = useMemo(() => flameMaterial(0, 1, 0.2), []);
  const core = useMemo(() => flameMaterial(2.2, 0.55, 1), []);
  const exhaust = useMemo(() => new Vector3(), []);
  const wake = useMemo(() => new Vector3(), []);
  const plumeDown = useMemo(() => new Vector3(0, -1, 0), []);
  const shipQuat = useMemo(() => new Quaternion(), []);
  const shield = useMemo(() => new Vector3(), []);
  const wind = useMemo(() => new Vector3(), []);
  const roll = useMemo(() => new Quaternion(), []);
  const tile = useMemo(() => new Vector3(), []);
  const aft = useMemo(() => new Vector3(), []);
  const jet = useRef({ power: 0, x: 0, y: 1, z: 0 });
  const camera = useThree((state) => state.camera);
  const scene = useThree((state) => state.scene);

  useEffect(() => {
    scene.background = new Color("#020308");
  }, [scene]);

  useFrame((state) => {
    const now = sampleAt(samples, time);
    const frame = sky.current;
    if (!now || !frame) return;
    const radius = EARTH_RADIUS + Math.max(now.z, 0);
    ecef.set(radius, 0, 0);
    Ellipsoid.WGS84.getNorthUpEastFrame(ecef, frame.worldToECEFMatrix);
    frame.sunDirection.copy(ecef).multiplyScalar(1 / radius);

    velocity.set(now.vy, now.vz, now.vx);
    const speed = velocity.length();
    if (speed > 1) velocity.multiplyScalar(1 / speed);
    const thrust = Math.hypot(now.fx, now.fy, now.fz);
    if (ship.current) {
      if (!noseOf(now, nose)) {
        nose.copy(velocity);
        if (thrust > 1000) nose.multiplyScalar(-1);
      }
      if (nose.lengthSq() > 0.5) {
        ship.current.quaternion.setFromUnitVectors(noseAxis, nose);
        // The black tiles sit on the model's +X side. Roll about the nose
        // so that side faces the air. The air comes from -velocity. Once
        // the nose lines up with the velocity that crosswind disappears.
        // Keep the last tile direction instead of dropping the roll: the
        // drop was 171 degrees in one step near t = 332 s.
        shield.set(1, 0, 0).applyQuaternion(ship.current.quaternion);
        wind.copy(velocity).negate();
        wind.addScaledVector(nose, -wind.dot(nose));
        const crosswind = Math.sqrt(wind.lengthSq());
        if (crosswind > 0.12) {
          tile.copy(wind).multiplyScalar(1 / crosswind);
        } else if (tile.lengthSq() > 0.5) {
          tile.addScaledVector(nose, -tile.dot(nose));
          if (tile.lengthSq() > 1e-6) tile.normalize();
        }
        if (tile.lengthSq() > 0.5) {
          shield.addScaledVector(nose, -shield.dot(nose));
          if (shield.lengthSq() > 1e-6) {
            shield.normalize();
            roll.setFromUnitVectors(shield, tile);
            ship.current.quaternion.premultiply(roll);
          }
        }
      }
      const qNow = ship.current.quaternion;
      const earlier = sampleAt(samples, Math.max(0, now.t - 0.1));
      const later = sampleAt(samples, now.t + 0.1);
      jet.current.power = 0;
      if (earlier && later && noseOf(earlier, aft) && noseOf(later, wind)) {
        const span = Math.max(0.04, later.t - earlier.t);
        wind.sub(aft);
        shield.crossVectors(aft, wind);
        const rate = shield.length() / span;
        jet.current.power = thrust < 1500 ? Math.min(1, Math.max(0, (rate - 0.1) / 0.35)) : 0;
        shield.applyQuaternion(qNow.clone().invert());
        if (shield.lengthSq() > 1e-8) {
          shield.normalize();
          jet.current.x = shield.x;
          jet.current.y = shield.y;
          jet.current.z = shield.z;
        }
      }
    }
    // Same jet as the landing. Brightness follows thrust over the ceiling,
    // the tip trails the velocity left over sideways to the thrust, and the
    // cone shortens to whatever room is aft of the nozzle.
    const age = Math.max(now.t, 0);
    const sep = Math.min(age / 12, 1);
    const fraction = Math.max(0, thrust / Math.max(thrustFull, 1));
    const burning = fraction > 0.04;
    const power = burning ? 0.95 + Math.min(fraction, 1) * 0.4 : 0;
    sheath.uniforms.uTime.value = state.clock.elapsedTime;
    sheath.uniforms.uPower.value = power;
    core.uniforms.uTime.value = state.clock.elapsedTime;
    core.uniforms.uPower.value = power;
    if (flame.current) {
      const room = Math.max(5, now.z + 1.2);
      let fit = Math.min(1, room / FLAME_LENGTH);
      if (showBooster) fit = Math.min(fit, (80 * sep) / FLAME_LENGTH);
      flame.current.scale.set(fit, fit, fit);
      // Scene axes are north, up, east. Exhaust leaves opposite the force.
      exhaust.set(-now.fy, -now.fz, -now.fx);
      if (exhaust.lengthSq() > 1 && ship.current) {
        exhaust.normalize();
        wake.set(-now.vy, -now.vz, -now.vx);
        const alongThrust = exhaust.dot(wake);
        wake.addScaledVector(exhaust, -alongThrust);
        const sideways = wake.length();
        if (sideways > 0.2) {
          const tip = Math.min(sideways * 0.4, 6.5) * fit;
          wake.multiplyScalar(tip / sideways);
        } else {
          wake.set(0, 0, 0);
        }
        const bend = sheath.uniforms.uBend.value as Vector3;
        const coreBend = core.uniforms.uBend.value as Vector3;
        bend.copy(wake);
        coreBend.copy(wake);
        shipQuat.copy(ship.current.quaternion).invert();
        exhaust.applyQuaternion(shipQuat);
        flame.current.quaternion.setFromUnitVectors(plumeDown, exhaust);
      }
    }

    // The camera stays level with the horizon. The ship turns inside that
    // frame, so the flip is the vehicle moving and the earth stays put.
    // The console leaves this off and uses orbit controls instead.
    horizontal.set(velocity.x, 0, velocity.z);
    if (horizontal.lengthSq() < 0.04) horizontal.set(0, 0, 1);
    else horizontal.normalize();
    flank.set(-horizontal.z, 0, horizontal.x);
    if (followCamera) {
      // The coast camera sits behind the ship. The entry camera sits ahead of
      // the nose, because that is the face the air is hitting.
      const opening = Math.max(now.t, 0) < 14 ? 1.55 : 1;
      // Ahead while the ship is high, including the nose swing near 40 km.
      // The view then drifts to the flank so the flip is seen from the side.
      const ahead = smoothstep(10000, 28000, altitude);
      const lead = -90 + 190 * ahead;
      behind.copy(horizontal).multiplyScalar(lead * opening);
      behind.addScaledVector(flank, 130 * opening);
      behind.y = (26 + 2 * ahead) * opening;
      camera.position.copy(behind);
      camera.up.set(0, 1, 0);
      camera.lookAt(0, 12, 0);
    }

    if (booster.current && ship.current) {
      // The file is one stack. At the origin the booster already sits on
      // the tail, 69 m aft up to the engines. Slide it straight back along
      // that axis. The small peel waits until the gap has opened.
      const peel = sep * sep;
      booster.current.quaternion.copy(ship.current.quaternion);
      booster.current.rotateOnWorldAxis(flank, peel * 0.12);
      aft.copy(nose).multiplyScalar(-80 * sep);
      aft.addScaledVector(flank, 8 * peel);
      booster.current.position.copy(aft);
      booster.current.visible = showBooster && age < 18;
    }

    // The sky quad paints the whole frame. Draw it first, then the stars,
    // so the catalog shows in the black above the horizon.
    if (skyMesh.current) skyMesh.current.renderOrder = -2;
  });

  return (
    <Atmosphere ref={sky} date={NOON}>
      <Sky ref={skyMesh} />
      <StarField />
      <hemisphereLight args={["#d5e4f5", "#6d8f6a", 0.55]} />
      <directionalLight position={[30, 140, 20]} intensity={2.2} />
      <SunLight />
      <SkyLight />
      <EarthSurface altitude={altitude} />
      <group ref={booster}>
        <Booster />
      </group>
      <group ref={ship}>
        <Ship flameRef={flame} sheathMat={sheath} coreMat={core} />
        <Plasma glow={glow} flow={velocity} />
        <ColdGas jet={jet.current} />
      </group>
    </Atmosphere>
  );
}

// The picture has to keep drawing when the browser is not handing out
// animation frames. A live tab paints from requestAnimationFrame. The timer
// only steps in when those frames have stopped arriving.
function Pump() {
  const advance = useThree((state) => state.advance);
  useEffect(() => {
    let frame = 0;
    let alive = true;
    let last = 0;
    const paint = (now: number) => {
      if (!alive) return;
      last = now;
      advance(now);
    };
    const step = (now: number) => {
      paint(now);
      frame = requestAnimationFrame(step);
    };
    frame = requestAnimationFrame(step);
    const timer = window.setInterval(() => {
      const now = performance.now();
      if (now - last < 40) return;
      paint(now);
    }, 32);
    return () => {
      alive = false;
      cancelAnimationFrame(frame);
      window.clearInterval(timer);
    };
  }, [advance]);
  return null;
}

const FLIGHT_FILE = "/flights/entry-look.ndjson";

function readFlight(text: string): Sample[] {
  return text
    .split("\n")
    .map((line) => parseSample(line))
    .filter((sample): sample is Sample => sample !== null);
}

// The console picture for the insertion and the return. The ship stays at
// the origin, the planet sits below it, and drag, scroll, and pan stay on
// the orbit controls. The coast page keeps its own scripted camera.
export function OrbitalStage({
  samples,
  time,
  showBooster,
}: {
  samples: Sample[];
  time: number;
  showBooster: boolean;
}) {
  const now = sampleAt(samples, time);
  const altitude = now ? Math.max(now.z, 0) : 0;
  const glow = heatGlow(now);
  return (
    <Canvas
      frameloop="never"
      camera={{ fov: 42, near: 0.5, far: 5e7, position: [90, 28, 130] }}
      gl={{ antialias: true, alpha: false, logarithmicDepthBuffer: true, preserveDrawingBuffer: true }}
    >
      <color attach="background" args={["#020308"]} />
      <Pump />
      <Suspense fallback={null}>
        {samples.length > 0 ? (
          <CoastScene
            samples={samples}
            time={time}
            altitude={altitude}
            glow={glow}
            showBooster={showBooster}
            followCamera={false}
            thrustFull={showBooster ? CLIMB_THRUST_N : RETURN_THRUST_N}
          />
        ) : null}
      </Suspense>
      <OrbitControls target={[0, 12, 0]} enablePan minDistance={40} maxDistance={8000} />
    </Canvas>
  );
}

export function CoastLook() {
  const query = new URLSearchParams(window.location.search);
  const queryTime = Number(query.get("t"));
  const [samples, setSamples] = useState<Sample[]>([]);
  const [time, setTime] = useState(Number.isFinite(queryTime) ? queryTime : 0);
  const [playing, setPlaying] = useState(!Number.isFinite(queryTime));

  useEffect(() => {
    let cancel = false;
    fetch(FLIGHT_FILE)
      .then((response) => response.text())
      .then((text) => {
        if (!cancel) setSamples(readFlight(text));
      })
      .catch(() => {
        if (!cancel) setSamples([]);
      });
    return () => {
      cancel = true;
    };
  }, []);

  const end = samples.length > 0 ? samples[samples.length - 1].t : 0;

  useEffect(() => {
    if (!playing || samples.length === 0) return;
    let frame = 0;
    let previous = performance.now();
    const tick = (now: number) => {
      const wall = Math.min((now - previous) / 1000, 0.1);
      previous = now;
      setTime((current) => {
        const sample = sampleAt(samples, current);
        const rate = playbackRate(sample, current);
        const next = current + wall * rate;
        if (next >= end) {
          setPlaying(false);
          return end;
        }
        return next;
      });
    };
    const step = (now: number) => {
      tick(now);
      frame = requestAnimationFrame(step);
    };
    frame = requestAnimationFrame(step);
    const timer = window.setInterval(() => {
      const now = performance.now();
      if (now - previous < 40) return;
      tick(now);
    }, 32);
    return () => {
      cancelAnimationFrame(frame);
      window.clearInterval(timer);
    };
  }, [playing, samples, end]);

  const now = sampleAt(samples, time);
  const speed = now ? Math.hypot(now.vx, Math.hypot(now.vy, now.vz)) : 0;
  const phase = phaseOf(now, time);
  const altitude = now ? Math.max(now.z, 0) : 0;
  const glow = heatGlow(now);
  const rate = playbackRate(now, time);
  const shownRate = Math.round(rate * 2) / 2;
  const rateLabel = rate < 1.15 ? "real time" : Number.isInteger(shownRate) ? `${shownRate}×` : `${shownRate.toFixed(1)}×`;

  return (
    <div style={{ position: "relative", height: "calc(100vh - 52px)" }}>
      <Canvas
        frameloop="never"
        camera={{ fov: 42, near: 0.5, far: 5e7, position: [0, 40, -200] }}
        gl={{ antialias: true, logarithmicDepthBuffer: true }}
      >
        <Pump />
        <Suspense fallback={null}>
          {samples.length > 0 ? (
            <CoastScene
              samples={samples}
              time={time}
              altitude={altitude}
              glow={glow}
              showBooster={false}
            />
          ) : null}
        </Suspense>
      </Canvas>
      <div
        style={{
          position: "absolute",
          left: 16,
          bottom: 16,
          color: "#f4f1ea",
          background: "rgba(8, 12, 16, 0.72)",
          padding: "10px 12px",
          borderRadius: 6,
          font: "14px/1.4 sans-serif",
        }}
      >
        <div>
          {phase} · {(altitude / 1000).toFixed(1)} km · {speed.toFixed(0)} m/s · t {time.toFixed(0)} s · {rateLabel}
        </div>
        <div style={{ marginTop: 4, opacity: 0.85 }}>
          7.6 km/s from 120 km. The side stays in the wind until 2 km/s, then the engines bring it down.
        </div>
        <div style={{ marginTop: 8, display: "flex", gap: 8 }}>
          <button
            type="button"
            onClick={() => {
              setTime(0);
              setPlaying(true);
            }}
          >
            Replay
          </button>
        </div>
      </div>
    </div>
  );
}
