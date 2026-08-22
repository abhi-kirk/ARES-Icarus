# Rocket Dynamics Notebook Series

Karpathy-style learn-by-doing notebook series for building deep intuition of rotational rigid body dynamics. The goal is to understand the physics and math deeply enough to implement rotational dynamics and attitude control in [ARES-Icarus](../README.md), a rocket landing GNC engine written in C++17.

Each notebook derives the physics from scratch, implements it in Python, and visualizes the result. Concepts are introduced in the order they are needed for the C++ implementation.

---

## Notebooks


| #   | Notebook                          | Concepts                                                                                       | Status      |
| --- | --------------------------------- | ---------------------------------------------------------------------------------------------- | ----------- |
| 1   | [The Pendulum](01_pendulum.ipynb) | State vector, Euler vs RK4, phase portraits, damping, chaos, fixed-point classification        | Complete    |
| 2   | [Rotations](02_rotations.ipynb)   | Body vs. world frame, rotation matrices, SO(3), Euler angles, non-commutativity, gimbal lock   | In progress |


---



## Notebook 1: The Pendulum


| Concept           | What we build             | Key takeaway                               |
| ----------------- | ------------------------- | ------------------------------------------ |
| State vector      | $[\theta, \omega]$        | Every simulation starts here               |
| Euler vs RK4      | Two integrators           | Euler drifts; RK4 conserves energy         |
| Phase portrait    | Global view of dynamics   | Fixed points, separatrices, stability      |
| Damping           | Spiral to equilibrium     | Dissipation turns centers into spirals     |
| Double pendulum   | Chaos                     | Sensitive dependence on initial conditions |
| Fixed-point types | Eigenvalue classification | Center, stable spiral, stable node, saddle |


