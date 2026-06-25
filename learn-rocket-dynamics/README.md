# Rocket Dynamics Notebook Series

Karpathy-style learn-by-doing notebook series for building deep intuition of rotational rigid body dynamics. The goal is NOT to learn Python — it's to understand the physics and math deeply enough to implement [ARES-Icarus](../README.md) (6-DOF rocket landing GNC in C++17) and to build toward SpaceX GNC/simulation roles.

Each notebook derives the physics from scratch, implements it in Python, and visualizes the result. Concepts are introduced in the order they are needed for the C++ implementation.

---

## Notebooks

| # | Notebook | Concepts | Status |
|---|----------|----------|--------|
| 1 | [The Pendulum](01_pendulum.ipynb) | State vector, Euler vs RK4, phase portraits, damping, chaos, fixed-point classification | Complete |
| 2 | 2D Rigid Body | Moment of inertia tensor, angular momentum, body-fixed vs. inertial frames | Planned |
| 3 | 3D Rotation & Euler Angles | Rotation matrices, SO(3), Euler angles, gimbal lock | Planned |
| 4 | Euler's Equations | Rigid body rotational dynamics, gyroscopic coupling, nutation | Planned |
| 5 | Quaternions | Why Euler angles break, quaternion algebra, singularity-free attitude | Planned |

---

## Notebook 1: The Pendulum

| Concept | What we build | Key takeaway |
|---------|---------------|--------------|
| State vector | $[\theta, \omega]$ | Every simulation starts here |
| Euler vs RK4 | Two integrators | Euler drifts; RK4 conserves energy |
| Phase portrait | Global view of dynamics | Fixed points, separatrices, stability |
| Damping | Spiral to equilibrium | Dissipation turns centers into spirals |
| Double pendulum | Chaos | Sensitive dependence on initial conditions |
| Fixed-point types | Eigenvalue classification | Center, stable spiral, stable node, saddle |
