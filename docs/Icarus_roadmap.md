# Icarus Development Roadmap

## Goal
Progressive development from 1‑DOF PID control to 6‑DOF optimal control with reinforcement learning mastery of controls, C++, and Physical AI for Tesla Optimus / SpaceX interview preparation.

---

## Overview

Phase 1: ✅ 1‑DOF Cascade PID [COMPLETE]  
Phase 1.5: 3‑DOF Translation [NEXT]  
Phase 2: Add Rotation (Euler)  
Phase 2.5: Quaternions  
Phase 3: MPC + Successive Convexification  
Phase 4: Reinforcement Learning  

---

## Phase 1: 1‑DOF Vertical Landing ✅ COMPLETE

### Achievements
| Metric | Gain Scheduling (1a) | Cascade PID (1b) |
|--------|--------|--------|
| Landing Velocity | -1.54 m/s | **-0.02 m/s** |
| Control Quality | Good | Excellent |
| Tuning Method | Manual | Grid Search (48 combos) |

### Key Learnings
- Cascade control separates timescales (position vs velocity)
- Outer loop frequency critically affects control quality (5s >> 1s)
- Systematic tuning essential for multi-loop systems
- Feedforward (hover throttle) improves tracking

### Stability Analysis (TODO - Quick Version)
1. Linearize at hover point (altitude=100m, velocity=0)
2. Compute Jacobians A, B for inner and outer loops
3. Use Python `control` library for gain/phase margins

---

## Phase 1.5: 3‑DOF Translation (No Rotation)

### Objective
Extend cascade control to 3D space WITHOUT rotation dynamics. Focus on trajectory planning, wind disturbances, and moving target tracking.

### Why This Phase?
- Natural extension of cascade architecture
- Avoids rotation complexity initially
- Focuses on path planning (relevant for Optimus)
- Wind rejection (real-world disturbance handling)

### State Space

```cpp
// State vector: 7 dimensions
State x = [x, y, z, vx, vy, vz, fuel]

// Control vector: 3 dimensions (direct thrust command)
Control u = [Fx, Fy, Fz]

// Assumption: Can command thurst in any direction instantly
// (Rotation dynamics handled in Phase 2)
```

### Dynamics

```cpp
// 3-DOF Translation Dynamics
dx/dt = vx
dy/dt = vy
dz/dt = vz

dvx/dt = Fx/m - Dx(vx)/m + wind_x
dvy/dt = Fy/m - Dy(vy)/m + wind_y
dvz/dt = Fz/m - g - Dz(vz)/m

dm/dt = -||F|| / (Isp * g0)  // Fuel consumption
```

### Control Architecture

Desired Position [x, y, z] = [0, 0, 0]  (landing pad)

```
        |
        v
+--------------------------------------+
| Outer Loop: 3D Position PID          |   (0.2 Hz)
| Error: [x_err, y_err, z_err]         |
+--------------------------------------+
        |
        v
Reference Velocity [vx_ref, vy_ref, vz_ref]
        |
        v
+--------------------------------------+
| Inner Loop: 3D Velocity PID          |   (10 Hz)
| Error: [vx_err, vy_err, vz_err]      |
+--------------------------------------+
        |
        v
Thrust Command [Fx, Fy, Fz]
        |
        v
+--------------------------------------+
| Thrust Allocation                    |
| Saturate: ||F|| <= F_max             |
+--------------------------------------+
        |
        v
Applied to Rocket Dynamics
```


### New Features to Implement

1. **3D State and Control Vectors**
   - Extend state from 3 → 7 dimensions
   - Extend control from scalar throttle → 3D thrust vector

2. **Thrust Magnitude Constraint**
    ```cpp
    // Can't exceed max thrust
    double F_magnitude = sqrt(Fx*Fx + Fy*Fy + Fz*Fz)
    if (F_magnitude > MAX_THRUST) {
        double scale = MAX_THRUST / F_magnitude;
        Fx *= scale; Fy *= scale; Fz *= scale;
    }
    ```

3. **Wind Disturbance Model**
    ```cpp
    // Constant winds + gusts
    double wind_x = WIND_BASE_X + gust_amplitude * sin(omega * t);
    double wind_y = WIND_BASE_Y + gust_amplitude * cos(omega * t);
    ```

4. Moving Landing Pad
    ```cpp
    // Landing pad moves (e.g., drone ship)
    double pad_x = pad_amplitude * sin(pad_omega * t);
    double pad_y = pad_amplitude * cos(pad_omega * t);

    setpoint = [pad_x, pad_y, 0];
    ```

### Deliverables
- [] Extended dynamics in src/dynamics/
- [] 3D cascade controller in src/control/
- [] Wind disturbance model
- [] Moving landing pad tracking
- [] 3D visualization tools in tools/
- [] Grid search tuning for 3D gains
- [] Documentation: docs/3dof_translation.md

### Success Criteria
- Land within 1m of moving target
- Handle 5 m/s constant wind + gusts
- Fuel efficiency within 20% of optimal
- Smooth control (no saturation oscillations)

---

## Phase 2: Add Rotation (Euler Angles)

### Objective
Add rotational dynamics using Euler angles. Now thrust direction is controlled by rocket orientation, not magic.

### Why Euler Angles First?
- Easier to visualize (roll, pitch, yaw are intuitive)
- Simpler math than quaternions
- Learn rotation dynamics before dealing with gimbal lock
- Can demonstrate understanding of the limitation

### State Space

```cpp
// State vector: 13 dimensions
State x = [
  x, y, z,  // position (3)
  vx, vy, vz,  // velocity (3)
  roll, pitch, yaw,  // euler angles (3)
  wx, wy, wz,  // angular velocity (3)
  fuel  // fuel mass (1)
]

// Control vector: 2 dimensions
Control u = [thrust_magnitude, gimbal_angle]
```

### Dynamics

**Translational:**
```cpp
// Thurst in body frame (along z-axis)
F_body = [0, 0, thrust]

// Transform to intertial frame using rotation matrix R(roll, pitch, yaw)
F_intertial = R * F_body

// Accelerations
dvx/dt = F_inertial[0] / m
dvy/dt = F_inertial[1] / m
dvz/dt = F_inertial[2] / m - g
```

**Rotational:**
```cpp
// Euler angle rates (kinematic relationship)
d(roll)/dt  = wx + wy*sin(roll)*tan(pitch) + wz*cos(roll)*tan(pitch)
d(pitch)/dt = wy*cos(roll) - wz*sin(roll)
d(yaw)/dt   = wy*sin(roll)/cos(pitch) + wz*cos(roll)/cos(pitch)

// Angular acceleration (Euler's equations)
I * dw/dt = torque - w × (I * w)

// Torque from thrust offset (gimbal)
torque = r_thrust × F_body  // r_thrust is lever arm to thrust point
```

**Gimbal Lock Issue**
```cpp
// When pitch = ±90°, these terms blow up:
tan(pitch) → ±∞
1/cos(pitch) → ±∞

// This is gimbal lock - loss of one degree of freedom
```

Solution: avoid ±90° or use quaternions.

### Control Architecture

Position [x, y, z] → Velocity [vx, vy, vz] → Desired Thrust Direction

```
                             |
                             v
        ---------------------------------------------
        |                                           |
        v                                           v

Desired Attitude                          Thrust Magnitude
[roll_d, pitch_d, yaw_d]                        |
        |                                       |
        v                                       |
Attitude Controller                             |
(PID on angle errors)                           |
        |                                       |
        v                                       |
Angular Rate Controller                         |
(PID on rate errors)                            |
        |                                       |
        v                                       v
Gimbal Angles -----------------------------> Thrust Vector
```


### Deliverables
- [] Euler rotation implementation
- [] Inertia tensor & torque calculations
- [] Attitude controller
- [] TVC logic
- [] Visualization tools
- [] Demonstrate gimbal lock scenario
- [] docs/6dof_euler.md

### Success Criteria
- Stable attitude control
- Land upright (roll, pitch < 5°)
- Handle disturbances
- Document gimbal lock limitations

---

## Phase 2.5: Upgrade to Quaternions

### Objective
Replace Euler angles with quaternions to eliminate gimbal lock.

### Why Quaternions?
- No gimbal lock (singularity-free)
- Computationally efficient (4 params vs 9 for rotation matrix)
- Industry standard
- Smooth interpolation (SLERP)

### State Space Change (14D)

```cpp
// Replace Euler angles with quaternion
State x = [
  x,y,z,
  vx,vy,vz,
  q0,q1,q2,q3,  // unit constraint
  wx,wy,wz,
  fuel
]
```

### Key Math

**Quaternion Kinematics**
```cpp
// Quaternion derivative from angular velocity
dq/dt = 0.5 * q ⊗ [0, wx, wy, wz]

// Where ⊗ is quaternion multiplication
```

**Rotation Using Quaternion**
```cpp
// Rotate vector v by quaternion q
v_rotated = q ⊗ [0, v] ⊗ q*

// Or convert to rotation matrix
R = quaternion_to_matrix(q)
v_rotated = R * v
```

**Quaternion Normalization**
```cpp
// Must maintain unit norm
double norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
q0/=norm; q1/=norm; q2/=norm; q3/=norm;
```

### Deliverables
- [] Quaternion math library
- [] Replace Euler dynamics
- [] Quaternion attitude controller
- [] Compare Euler vs Quaternion
- [] Demonstrate gimbal lock avoidance
- [] docs/quaternions.md

### Success Criteria
- All Phase 2 scenarios work
- No gimbal lock in any orientation
- 6‑DOF maneuvers stable
- Performance comparison documented

---

## Phase 3: Model Predictive Control (MPC)

### Objective
Replace PID cascade with optimal control using MPC and successive convexification.

### Why MPC?
- Optimal trajectories (fuel, time or custom cost)
- Handles constraints explicitly (thrust limits, glide slope)
- Predictive control
- Industry standard for rockets, autonomous vehicles, robotics

### Phase 3a: LTV‑MPC (Linear Time-Varying)

Approach:
1. Use cascade PID trajectory as reference
2. Linearize dynamics along reference
3. Get time-varying A(t), B(t) matrices
4. Solve QP each timestep

Solver Options:
- OSQP (recommended)
- qpOASES
- CVXPY

Deliverables:
- [] Jacobian computation
- [] LTV system formulation
- [] OSQP integration
- [] Compare LTV‑MPC vs PID
- [] docs/ltv_mpc.md

---

## Phase 3b: Successive Convexification (SCvx)

### SpaceX Approach

1. Initial straight-line trajectory
2. Iterate until convergence:
   - Linearize dynamics
   - Convexify constraints
   - Solve convex optimization
   - Update trajectory
   - Apply trust region
3. Result: locally optimal trajectory

### Key Insight: Lossless Convexification

Original:
thrust_min ≤ ||u|| ≤ thrust_max

Relaxed:
||u|| ≤ thrust_max

Optimal solution satisfies original constraint.

### Algorithm (Python)

```python
def successive_convexification(x0, xf, params):
    traj = initial_guess(x0, xf)
    for iteration in range(max_iters):
        A, B, c = linearize_along_trajectory(traj, dynamics)
        prob = ConvexProblem(
            objective=fuel_cost(u),
            constraints=[
                dynamics_constraint(x, u, A, B, c),
                thrust_cone_constraint(u),
                boundary_conditions(x0, xf),
                trust_region(traj, delta)
            ]
        )
        new_traj = prob.solve()
        if norm(new_traj - traj) < tolerance:
            return new_traj
        traj = update_with_trust_region(traj, new_traj, delta)
    return traj
```

Solver Options:
- ECOS
- CVXGEN
- CVXPYgen

Deliverables:
- [] Convex formulation
- [] Trust region implementation
- [] Solver integration
- [] Convegence analysis
- [] Compare SCvx vs MPC vs PID
- [] docs/successive_convexification.md

---

## Reading List

**Foundations**

1. Boyd & Vandenberghe — Convex Optimization  
   - https://web.stanford.edu/~boyd/cvxbook/

**CVXGEN**

2. Mattingley & Boyd — CVXGEN (2012)  
   - Understand embedded convex optimization code generation

**Rocket Landing (SpaceX Foundation)**

3. Acikmese & Ploen — Powered Desecent Guidance for Mars Landing
   - The foundational paper
4. Acikmese et al. — Lossless Convexification of Nonconvex Control Constraints  
    - How to make rocket landing convex
5. Blackmore et al. — Autonomous Precision Landing of Space Rockets (2016)
    - SpaceX overview, practical considerations

**6‑DOF Extension**

6. Szmuk & Acikmese — Successive Convexification for 6‑DOF Rocket Powered Landing (2018)
    - Full 6-DOF formulation

---

## Phase 4: Reinforcement Learning

### Objective
Train RL agent for rocket landing, compare to MPC, explore sim‑to‑real considerations.

### Why RL After MPC?
- MPC provides optimal baseline
- Understand what RL learns
- RL may discover robust solutions
- Useful in uncertain environments

---

### Algorithms
- PPO — stable, widely used
- SAC — sample efficient, good for continuous control

---

### Training Example

```python
from stable_baselines3 import PPO

env = RocketLandingEnv()
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=1_000_000)
model.save("rocket_landing_ppo")
```

---

### RL Environment

```python
class RocketLandingEnv(gym.Env):

    def __init__(self):
        # State: [x, y, z, vx, vy, vz, quat(4), omega(3), fuel]
        self.observation_space = spaces.Box(...)

        # Action: [thrust, gimbal_pitch, gimbal_yaw]
        self.action_space = spaces.Box(...)

    def step(self, action):
        # Apply action, integrate dynamics
        # Return: obs, reward, done, info

    def reset(self):
        # Randomize initial conditions
```

---

### Reward Design (Critical)

```python
def compute_reward(state, action, next_state):
    reward = 0

    # progress toward pad
    reward += progress_toward_pad(state, next_state)

    # fuel efficiency
    reward -= fuel_penalty * fuel_used(action)

    # smoothness
    reward -= jerk_penalty * control_jerk(action)

    # terminal rewards
    if landed_successfully(next_state):
        reward += 1000
    elif crashed(next_state):
        reward -= 1000

    return reward
```

---

### Comparisons

| Aspect | PID Cascade | MPC (SCvx) | RL (PPO) |
|--------|------------|-----------|---------|
| Optimality | Suboptimal | Locally optimal | Learned |
| Robustness | Moderate | Handles constraints | Can be very robust |
| Computation | Trival | ~50ms solve | ~1ms inference |
| Tuning | Manual gains | Cost weights | Reward shaping |
| Adaptability | Fixed | Re-solve | Generalizes |

---

### Deliverables

- Gym environment for 6‑DOF rocket
- PPO training pipeline
- Reward engineering experiments
- Domain randomization (mass, wind, thrust uncertainty)
- Compare RL vs MPC vs PID
- Sim‑to‑real considerations document
- TensorRT deployment for inference (optional)
- Documentation: docs/reinforcement_learning.md

### Success Criteria

- RL achieves >95% landing success rate
- Handles domain randomization
- Inference time < 5 ms
- Documented comparison with MPC

---

## Interview Preparation Alignment

### Tesla Optimus (Primary Target)

**Most Relevant Phases**
- Phase 1.5: 3D trajectory planning (manipulation paths)
- Phase 2–2.5: Attitude control & quaternions (joint control)
- Phase 3: MPC (standard in legged robotics)

**Key Talking Points**
- "Implemented cascade control with systematic tuning"
- "Extended to 3D with wind disturbance rejection"
- "Compared PID vs MPC — understand trade‑offs"
- "Can discuss gimbal lock and why quaternions solve it"

### SpaceX (Secondary Target)

**Most Relevant Phases**
- Phase 3b: Successive convexification (actual approach)
- Phase 2.5: Quaternions (aerospace standard)
- Phase 4: RL comparison (future directions)

**Key Talking Points**
- "Implemented lossless convexification from Acikmese’s papers"
- "Understand why single‑point linearization fails"
- "Can discuss CVXGEN vs OSQP trade‑offs"
- "Compared optimal control vs learned policies"

---