// The two public doors. A door with no href says Coming Soon.
// Each blurb is markdown: paragraphs separated by a blank line.

export const COMING_SOON = "Coming Soon";

export const DOORS = [
  {
    id: "dynamics",
    name: "Rocket Dynamics",
    href: "/dynamics/state/spring",
    blurb: `Learn the fundamentals of Rocket dynamics.`,
  },
  {
    id: "simulator",
    name: "Simulator",
    href: "",
    blurb: `Experiment with sims for landing a rocket vertically.

Coming Soon.`,
  },
] as const;
