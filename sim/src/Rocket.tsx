import { useMemo, type RefObject } from "react";
import { DoubleSide, LatheGeometry, Vector2 } from "three";
import type { Mesh, ShaderMaterial } from "three";

// Falcon 9 first stage, from the published figures: 3.7 m across,
// 47.7 m with the interstage, four legs, four grid fins, nine Merlins.
// The mesh is that ratio, not a cone with slab fins. No wordmark.

const BODY_R = 0.11;
const FIN_Y = 0.78;
const ENGINE_Y = -1.12;
const CENTER_BELL = 1.12;
const BELL_PROFILE = [
  new Vector2(0.012, 0.015),
  new Vector2(0.016, 0),
  new Vector2(0.022, -0.035),
  new Vector2(0.03, -0.075),
  new Vector2(0.036, -0.105),
];

// Mesh y of the center nozzle. The origin of this model is the center of mass.
export const NOZZLE_Y = ENGINE_Y + Math.min(...BELL_PROFILE.map((point) => point.y)) * CENTER_BELL;

const PAINT = "#f3f4f5";
const SOOT = "#c4c0b6";
const CARBON = "#16181b";
const TITANIUM = "#9a9188";
const BELL = "#8d8478";

function bellGeometry(): LatheGeometry {
  return new LatheGeometry(BELL_PROFILE, 18);
}

function GridFin() {
  const span = 0.2;
  const bar = 0.007;
  const depth = 0.04;
  const ribs: [number, number, number, number][] = [
    [span / 2, span / 2, span, bar],
    [span / 2, -span / 2, span, bar],
    [0, 0, bar, span],
    [span, 0, bar, span],
    [span / 2, span / 6, span, bar],
    [span / 2, -span / 6, span, bar],
    [span / 3, 0, bar, span],
    [(2 * span) / 3, 0, bar, span],
  ];
  return (
    <group position={[BODY_R, FIN_Y, 0]}>
      {ribs.map(([x, y, w, h], index) => (
        <mesh key={index} position={[x, y, 0]}>
          <boxGeometry args={[w, h, depth]} />
          <meshPhysicalMaterial color={TITANIUM} metalness={0.74} roughness={0.3} />
        </mesh>
      ))}
    </group>
  );
}

function Engines({ geometry }: { geometry: LatheGeometry }) {
  const seats = [{ x: 0, z: 0, scale: CENTER_BELL }];
  for (let i = 0; i < 8; i++) {
    const angle = (i + 0.5) * (Math.PI / 4);
    seats.push({
      x: Math.cos(angle) * 0.068,
      z: Math.sin(angle) * 0.068,
      scale: 0.78,
    });
  }

  return (
    <group position={[0, ENGINE_Y, 0]}>
      {seats.map((seat, index) => (
        <mesh key={index} position={[seat.x, 0, seat.z]} scale={seat.scale} geometry={geometry}>
          <meshPhysicalMaterial color={BELL} metalness={0.78} roughness={0.32} />
        </mesh>
      ))}
    </group>
  );
}

export function Rocket({ flameRef, flameMat }: { flameRef: RefObject<Mesh | null>; flameMat: ShaderMaterial }) {
  const bell = useMemo(bellGeometry, []);
  const paint = { color: PAINT, metalness: 0.14, roughness: 0.46, clearcoat: 0.2 };
  const soot = { color: SOOT, metalness: 0.18, roughness: 0.62 };
  const legAngles = [0, Math.PI / 2, Math.PI, (3 * Math.PI) / 2];
  const finAngles = legAngles.map((angle) => angle + Math.PI / 4);

  return (
    <group>
      <mesh position={[0, 0.08, 0]}>
        <cylinderGeometry args={[BODY_R, BODY_R, 1.64, 48]} />
        <meshPhysicalMaterial {...paint} />
      </mesh>
      <mesh position={[0, -0.62, 0]}>
        <cylinderGeometry args={[BODY_R + 0.001, BODY_R + 0.001, 0.28, 48]} />
        <meshPhysicalMaterial {...soot} />
      </mesh>
      <mesh position={[0, 0.42, 0]}>
        <cylinderGeometry args={[BODY_R + 0.003, BODY_R + 0.003, 0.012, 48]} />
        <meshPhysicalMaterial color="#8e9498" metalness={0.4} roughness={0.45} />
      </mesh>
      <group rotation={[0, 0.4, 0]}>
        <mesh position={[BODY_R + 0.006, 0.1, 0]}>
          <boxGeometry args={[0.012, 1.35, 0.026]} />
          <meshPhysicalMaterial color="#2c3136" metalness={0.45} roughness={0.48} />
        </mesh>
      </group>

      <mesh position={[0, 1.02, 0]}>
        <cylinderGeometry args={[BODY_R + 0.004, BODY_R + 0.004, 0.34, 48, 1, true]} />
        <meshPhysicalMaterial color={CARBON} metalness={0.35} roughness={0.55} side={DoubleSide} />
      </mesh>
      <mesh position={[0, 0.86, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <circleGeometry args={[BODY_R - 0.01, 32]} />
        <meshPhysicalMaterial color="#0c0d0e" roughness={0.9} metalness={0.1} />
      </mesh>
      <mesh position={[0, 1.19, 0]} rotation={[-Math.PI / 2, 0, 0]}>
        <ringGeometry args={[BODY_R - 0.01, BODY_R + 0.008, 48]} />
        <meshPhysicalMaterial color={TITANIUM} metalness={0.7} roughness={0.28} />
      </mesh>

      <mesh position={[0, -0.86, 0]}>
        <cylinderGeometry args={[0.116, BODY_R, 0.14, 40]} />
        <meshPhysicalMaterial color={CARBON} metalness={0.4} roughness={0.5} />
      </mesh>
      <Engines geometry={bell} />

      {finAngles.map((angle) => (
        <group key={angle} rotation={[0, angle, 0]}>
          <GridFin />
        </group>
      ))}

      {legAngles.map((angle) => (
        <group key={angle} rotation={[0, angle, 0]}>
          <mesh position={[BODY_R + 0.012, -0.62, 0]}>
            <boxGeometry args={[0.02, 0.32, 0.07]} />
            <meshPhysicalMaterial {...paint} />
          </mesh>
        </group>
      ))}

      <mesh ref={flameRef} position={[0, -1.32, 0]} rotation={[Math.PI, 0, 0]} material={flameMat}>
        <coneGeometry args={[0.05, 0.7, 20]} />
      </mesh>
    </group>
  );
}
