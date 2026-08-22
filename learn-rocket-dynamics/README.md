# Rocket Dynamics Notebook Series

Karpathy-style learn-by-doing notebooks for rotational rigid body dynamics. The goal is to understand the physics deeply enough to implement attitude kinematics and rotational dynamics in [ARES-Icarus](../README.md).

Each notebook derives the physics from scratch, implements it in Python, and visualizes the result. Concepts are introduced in the order they are needed for the C++ engine.

This README is the public notebook list. Keep it in sync when a notebook's status changes.

---

## Notebooks

| #   | Notebook | Concepts | Status |
| --- | --- | --- | --- |
| 1   | [The Pendulum](01_pendulum.ipynb) | State vector, Euler vs RK4, phase portraits, damping, chaos, fixed-point classification | Complete |
| 2   | [Rotations](02_rotations.ipynb) | Frames, SO(3), ZYX Euler, gimbal lock, Euler kinematics, quaternions, $\dot{q}$, SLERP | Complete |
| 3   | Euler's equations | Inertia tensor, Euler's rigid-body equation, gyroscopic coupling, nutation | Planned |

---

## Notebook 1: The Pendulum

| Concept | What we build | Key takeaway |
| --- | --- | --- |
| State vector | $[\theta, \omega]$ | Every simulation starts here |
| Euler vs RK4 | Two integrators | Euler drifts; RK4 conserves energy |
| Phase portrait | Global view of dynamics | Fixed points, separatrices, stability |
| Damping | Spiral to equilibrium | Dissipation turns centers into spirals |
| Double pendulum | Chaos | Sensitive dependence on initial conditions |
| Fixed-point types | Eigenvalue classification | Center, stable spiral, stable node, saddle |

## Notebook 2: Rotations

| Concept | What we build | Key takeaway |
| --- | --- | --- |
| 2D rotations | $R(\theta)$ from trig | In 2D, rotations commute |
| Frames | Body vs world | Body X is the nose; world Z is up |
| ZYX Euler | $R_z(\psi)R_y(\theta)R_x(\phi)$ | Intuitive, and the order matters |
| Gimbal lock | Nested gimbal axes | The rocket never locks; the coordinates do |
| Euler kinematics | $\omega \to (\dot\phi,\dot\theta,\dot\psi)$ | $1/\cos\theta$ blows up on the pad |
| Quaternions | Hamilton product, $q$ and $-q$ | No singularities; double cover |
| $\dot{q}$ | $q̇ = \tfrac12 q \otimes \omega$ | This is what the 6-DOF plant integrates |
| SLERP | Great-arc interpolation | Guidance reference $q_{\mathrm{ref}}(t)$, not dynamics |
