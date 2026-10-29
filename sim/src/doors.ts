// The two public doors. A door with no href says Coming Soon.
// Each blurb is markdown: paragraphs separated by a blank line.
// href stays a plain string so a later door can be empty again.

export const COMING_SOON = "Coming Soon";

type Door = {
  id: string;
  name: string;
  href: string;
  blurb: string;
};

export const DOORS: Door[] = [
  {
    id: "dynamics",
    name: "Rocket Dynamics",
    href: "/dynamics/state/spring",
    blurb: `Learn the fundamentals of Rocket dynamics.`,
  },
  {
    id: "simulator",
    name: "Simulator",
    href: "/simulator",
    blurb: `Experiment with sims for landing a rocket vertically.`,
  },
];
