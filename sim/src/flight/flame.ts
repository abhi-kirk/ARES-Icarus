import { AdditiveBlending, DoubleSide, ShaderMaterial, Vector3 } from "three";

// The cone in Ship.tsx is 14 m tall. The landing shortens it to the room
// left above the deck, and the climb shortens it to the gap behind the tail.
export const FLAME_LENGTH = 14;

// Turbulence travels down the jet. The tip bends with uBend, in meters,
// added in world space after the cone is placed. The orbital canvas uses a
// logarithmic depth buffer, so the same shader includes that chunk. On the
// landing canvas the chunk compiles out.
const FLAME_VERT = `
  #include <common>
  #include <logdepthbuf_pars_vertex>
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
    float travel = along * 8.0 - uTime * 5.0 + uPhase;
    float wave = sin(travel) + 0.45 * sin(travel * 1.7 + 1.2);
    float amp = smoothstep(0.05, 0.3, along) * (0.28 + along * 0.45) * uAmp;
    vBulge = wave;
    vec3 moved = position;
    moved.xz += dir * wave * amp;
    vec4 world = modelMatrix * vec4(moved, 1.0);
    world.xyz += uBend * smoothstep(0.1, 1.0, along);
    gl_Position = projectionMatrix * viewMatrix * world;
    #include <logdepthbuf_vertex>
  }
`;

const FLAME_FRAG = `
  #include <common>
  #include <logdepthbuf_pars_fragment>
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
    #include <logdepthbuf_fragment>
  }
`;

export function flameMaterial(phase: number, amp: number, heat: number): ShaderMaterial {
  return new ShaderMaterial({
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
}
