import { useRef } from "react";
import { Canvas, useFrame } from "@react-three/fiber";
import { OrbitControls } from "@react-three/drei";
import type { Mesh } from "three";
import { height } from "./model";
import { getTime, setTime } from "./sceneStore";

function Bead() {
  const mesh = useRef<Mesh>(null);
  const drag = useRef({ on: false, y: 0 });

  useFrame(() => {
    if (mesh.current) mesh.current.position.y = height(getTime());
  });

  return (
    <mesh
      ref={mesh}
      onPointerDown={(event) => {
        event.stopPropagation();
        drag.current = { on: true, y: event.clientY };
        (event.target as Element).setPointerCapture(event.pointerId);
      }}
      onPointerMove={(event) => {
        if (!drag.current.on) return;
        const dy = drag.current.y - event.clientY;
        drag.current.y = event.clientY;
        setTime(getTime() + dy * 0.012);
      }}
      onPointerUp={() => {
        drag.current.on = false;
      }}
    >
      <sphereGeometry args={[0.32, 32, 32]} />
      <meshStandardMaterial color="#e2a43a" />
    </mesh>
  );
}

function Scene() {
  return (
    <>
      <color attach="background" args={["#10161c"]} />
      <ambientLight intensity={0.55} />
      <directionalLight position={[4, 6, 5]} intensity={1.4} />
      <mesh>
        <cylinderGeometry args={[0.025, 0.025, 10, 12]} />
        <meshStandardMaterial color="#8b9690" />
      </mesh>
      <Bead />
      <OrbitControls enablePan={false} target={[0, 0, 0]} />
    </>
  );
}

export function Viewport() {
  return (
    <Canvas className="viewport-canvas" camera={{ position: [4.2, 0.4, 6.2], fov: 40 }}>
      <Scene />
    </Canvas>
  );
}
