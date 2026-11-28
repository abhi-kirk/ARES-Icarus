# Icarus

<p align="center">
  <img src="engine/images/icarus.jpg" alt="Icarus" width="500"/>
</p>

Icarus is a study of how a rocket comes down. The landing is the hard part: the vehicle is lightest when it has to be most precise, and the loop that holds it there is unstable if you stay too long.

Three pieces of that study live here.

**Icarus-S** ([sim/](sim/)) is the public site, at [icarus-arc.com](https://icarus-arc.com). The home page names two doors, Rocket Dynamics and Simulator. Rocket Dynamics is a picture you can move, a chart of the same numbers, and a conversation that can move the picture or ask for another series. It opens on the spring, the step, the burn, and the engine, and the course continues through the loop, the frequency, the turn, the vehicle, the best descent, and the propellant. Simulator opens on the climb from booster separation. Return is the hot entry, and Landing is the vertical hop. The Moon is a divert onto the pad.

**Icarus-I** ([rockets/](rockets/)) is the first airframe, a LOC IV in build for a Level 1 flight. The number belongs to the rocket. A later flight of this one stays Icarus-I. The next rocket built is Icarus-II.

**The landing** ([engine/](engine/)) is a C++ simulation through three finished phases: a vertical landing, a 3-DOF descent onto a moving pad in wind, and a stability analysis of that loop.

<p align="center">
  <img src="engine/images/trajectory_pid_3dof.png" alt="3-DOF powered descent from 300 m onto a drifting pad" width="900"/>
</p>

Phase 2. Cascade PID from 300 m, with wind and a pad that drifts. Thrust is a force vector. The tank gets lighter as it burns, so the same newton buys more acceleration near the ground. The rocket meets the pad.

<p align="center">
  <img src="engine/images/icarus-architecture.png" alt="Guidance, control, and dynamics for the 3-DOF landing" width="900"/>
</p>

Guidance turns position error into a reference velocity. Control turns velocity error into thrust. Dynamics integrates that force, with gravity, drag, and a mass that changes.
