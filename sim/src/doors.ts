// The two public doors. A door with no href is named, and says it is coming soon.

export const COMING_SOON = "Coming Soon";

export const DOORS = [
  { id: "dynamics", name: "Rocket Dynamics", href: "/model" },
  { id: "simulator", name: "Simulator", href: "" },
] as const;
