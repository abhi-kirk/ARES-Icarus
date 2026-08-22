# Notebook → Icarus C++ mapping

Phase 4 (current engine work) starts once rotation notebooks are solid. Concepts are introduced in the order the C++ needs them, not textbook order.

## Status

| Notebook | Covers | C++ consumers |
|---|---|---|
| `01_pendulum.ipynb` | State `[θ, ω]`, `f(t, state)`, Euler vs RK4, phase portraits, fixed-point eigenvalues | Already in engine: `math/integrator.h`, `dynamics/` `f(t, State)` signature |
| `02_rotations.ipynb` | Body vs world, SO(3), ZYX Euler, non-commutativity, gimbal lock | Phase 4 Euler step: 13D state with `roll,pitch,yaw`; feel gimbal lock before quaternions |
| Notebook 3 (planned) | Inertia tensor, Euler's rigid-body equations, gyroscopic coupling, nutation | `dynamics/rocket_6dof.h`, angular velocity `ω` in body frame |
| Notebook 4 (planned) | Quaternion algebra, singularity-free kinematics | `math/quaternion.h`, 14D state `[q0,q1,q2,q3]`, attitude controller |

Phase 4 Euler state: `[x,y,z, vx,vy,vz, roll,pitch,yaw, wx,wy,wz, fuel]`
Phase 4 quaternion state: `[x,y,z, vx,vy,vz, q0,q1,q2,q3, wx,wy,wz, fuel]`
Control: `[thrust_magnitude, gimbal_angle]` (TVC)

## Conventions that must match C++ later

- Integrators: `deriv(t, state)` matching `SystemDynamics` / `function<State(double t, const State& y)>`
- Quaternions: Hamilton `[w, x, y, z]`, already in `learn-rocket-dynamics/utils.py`
- Euler: ZYX (`yaw` then `pitch` then `roll`); `euler_to_rotation_matrix` is `Rz(yaw) @ Ry(pitch) @ Rx(roll)`
- Body rocket in `utils.make_rocket_body`: nose along **+X**, engine −X, YZ cross-section
- Theory that outgrows a notebook goes in `docs/` (e.g. `docs/mechanics_notes.md` from Susskind)

## Not this series

Orbital mechanics (TLE, Lambert, SPICE, Horizons) is out of scope. Powered-descent rigid-body attitude is in scope.
