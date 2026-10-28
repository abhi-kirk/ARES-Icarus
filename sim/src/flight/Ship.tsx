import { useGLTF } from "@react-three/drei";
import { useMemo } from "react";
import type { RefObject } from "react";
import type { Group, Material, Mesh, MeshStandardMaterial, Object3D, ShaderMaterial } from "three";

// SN24, from the Sketchfab model. The file is one stack. The booster is hidden.
// The file's own units are not meters. The numbers below were measured from that
// file: the leg ring is the bottom, the nose is the top, and the engines sit
// just off the origin. Scale makes the ship 50 m tall, and the shift puts the
// leg ring on the deck with the engines over the pad.

const MODEL = "/models/sn24/scene.gltf";
const FILE_BOTTOM = 23.95;
const FILE_TOP = 41.32;
const FILE_ENGINE_X = -0.39;
const FILE_ENGINE_Z = -0.149;
const NOZZLE = 24.15;
const SHIP_HEIGHT = 50;

useGLTF.preload(MODEL);

function calmSteel(material: Material): Material {
  const next = material.clone() as MeshStandardMaterial;
  if (typeof next.metalness === "number" && next.metalness > 0.4) next.metalness = 0.32;
  if (typeof next.roughness === "number" && next.roughness < 0.35) next.roughness = 0.4;
  return next;
}

export function Ship({
  flameRef,
  sheathMat,
  coreMat,
}: {
  flameRef: RefObject<Group | null>;
  sheathMat: ShaderMaterial;
  coreMat: ShaderMaterial;
}) {
  const gltf = useGLTF(MODEL);
  const model = useMemo(() => {
    const root = gltf.scene.clone(true);
    const booster = root.getObjectByName("Superheavy_4");
    if (booster) booster.visible = false;
    root.traverse((obj: Object3D) => {
      const mesh = obj as Mesh;
      if (!mesh.isMesh) return;
      const list = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
      const cloned = list.map(calmSteel);
      mesh.material = Array.isArray(mesh.material) ? cloned : cloned[0];
    });
    return root;
  }, [gltf.scene]);

  const scale = SHIP_HEIGHT / (FILE_TOP - FILE_BOTTOM);
  const nozzle = (NOZZLE - FILE_BOTTOM) * scale;

  return (
    <group>
      <primitive
        object={model}
        scale={scale}
        position={[-FILE_ENGINE_X * scale, -FILE_BOTTOM * scale, -FILE_ENGINE_Z * scale]}
      />
      <group ref={flameRef} position={[0, nozzle, 0]}>
        <mesh position={[0, -7, 0]} rotation={[Math.PI, 0, 0]} material={sheathMat}>
          <coneGeometry args={[3.6, 14, 48]} />
        </mesh>
        <mesh position={[0, -7, 0]} rotation={[Math.PI, 0, 0]} material={coreMat}>
          <coneGeometry args={[1.7, 14, 32]} />
        </mesh>
      </group>
    </group>
  );
}
