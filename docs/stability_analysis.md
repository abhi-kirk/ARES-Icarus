# 3DOF Stability Analysis

## Open-Loop
The 3DOF state and control vectors are: 

$$
\begin{aligned}
x &= [p_x, p_y, p_z, v_x, v_y, v_z, m]\\
u &= [F_x, F_y, F_z]
\end{aligned}
$$

System dynamics: 

$$
\dot{x} = f(x, u)
$$

$$
\begin{gathered}
\dot{p}_x = v_x, \space\dot{p}_y = v_y, \space\dot{p}_z = v_z\\
\dot{v}_x = F_x/m, \space\dot{v}_y = F_y/m, \space\dot{v}_z = (F_z - mg)/m\\
\dot{m} = \frac{-||F||}{F_{max}}\cdot {\dot{m}_{max}}
\end{gathered}
$$

>[!NOTE]
> We will treat $m$ as a slowly varying parameter, not a state.
> We will analyze stability at a fixed $m$, then check how stability margins change across the mass range (a.k.a robustness analysis). 

Hence, we have the revised state vector:

$$
x = [p_x, p_y, p_z, v_x, v_y, v_z]
$$

### Equilibrium
At equilibrium we have: $\dot{x} = 0$. 

Hence, we get:

$$
\begin{gathered}
v_x = v_y = v_z = 0\\
F_x = F_y = 0\\
F_z = mg
\end{gathered}
$$

>[!IMPORTANT]
> We also have $p_z = p_z^*$ which means altitude (position in z-direction) can take any reference value.

>[!Tip]
> The equilibrium condition is also called the _Trim_ condition, especially in aerospace literature.

>[!NOTE]
> Vertical and horizontal dynamics are decoupled in open-loop; i.e., $(p_i, v_i)$ only depend on $F_i$, $\forall i\in (x, y, z)$. 
> - Hence we can analyze 3 independent 2D subsytems instead of a single 6D system.
> - Even in closed-loop, there are 3 decoupled 2D loops. 
> - Moreover, since $x$ and $y$ are structurally identical, we only need to analyze 2 independent 2D subsytems. 
>   - Vertical loop: different gains, different timescales, and with feedforward.
>   - Horizontal loop: representative of both $x$ and $y$.

> [!IMPORTANT]
> Stability analysis is for linear systems only.
> 
> Strictly speaking, our system is nonlinear because:
> - $m$ varies,
> - PID has saturation and a rate-limiter on the outer loop.


### Perturbation Analysis
Perturbation around Trim (subtract equilibtrium condition from current condition to get perturbation):

$$
\begin{gathered}
\delta p_z = p_z - p_z^*\\
\delta v_z = v_z - 0\\
\delta F_z = F_z - mg
\end{gathered}
$$

Substituting in system dynamics, we get the perturbation dynamics:

$$
\begin{aligned}
\delta\dot{p}_z &= v_z = \delta v_z\\
\delta\dot{v}_z &= F_z/m - g = \delta F_z/m
\end{aligned}
$$

This gives us a _double integrator_.

>[!NOTE]
> Double Integrator:
>   - A system where the input is integrated twice to produce the output.
>   - Transfer function: $1/s^2$, i.e., two poles at the origin. 
>   - A double integrator is _marginally stable_ open-loop (i.e., without control), i.e., poles on the imaginary axis and not in the left-half plane.
> 
> For rockets, this means that the control input force should be the second derivative of the output position: 
>   - $p \xrightarrow{d/dt} v \xrightarrow{d/dt} a = F/m$.

>[!Tip]
> - _Asymptotically stable_: perturbations decay back to 0.
> - _Marginally stable_: drifts forever, perturbation neither grows or decays.
> - _Unstable_: perturbation grows without bound.

### Jacobian
For vertical subsystem, $x = [\delta p_z, \delta v_z]$ and $u = \delta F_z$.

Linearized dynamics should be: $\dot{x} = Ax + Bu$, where $A = \frac{\partial f}{\partial x}$, $B = \frac{\partial f}{\partial u}$.

Perturbed dynamics become:

$$
\begin{bmatrix}
\delta\dot{p}_z\\
\delta\dot{v}_z
\end{bmatrix} =
\begin{bmatrix}
0 & 1\\
0 & 0
\end{bmatrix}
\begin{bmatrix}
\delta p_z\\
\delta v_z
\end{bmatrix}
+
\begin{bmatrix}
0\\
1/m
\end{bmatrix}
\delta F_z
$$

## Closed-Loop

### Vertical Subsystem
Controller contributions in Phase 1.5:

$$
\begin{aligned}
v_z^{ref} &= K_p^o \space \delta p_z \space\space\rightarrowtail \text{outer loop}\\
\delta F_z &= K_p^i(v_z^{ref} - \delta v_z) + K_i^i\int (v_z^{ref} - \delta v_z)\,dt \space\space \rightarrowtail \text{inner loop}
\end{aligned}
$$

>[!NOTE]
> $\delta p_z$: deviation of position from trim $p_z=0$, which is also what the outer loop wants.
> 
> $\delta v_z$: deviation of velocity from trim $v_z=0$, but inner loop wants to track to reference velocity not make it zero.

Substituting, we get:

$$
\delta F_z = K_p^i(K_p^o\delta p_z - \delta v_z) + K_i^i\int (K_p^o\delta p_z - \delta v_z)\,dt
$$

Because of integrator memory, we need to define a new state $x = [\delta p_z, \delta v_z, \xi]$, where the integrator state $\xi$ is:

$$
\xi = \int (v_z^{ref} - \delta v_z)\,dt = \int (K_p^o\delta p_z - \delta v_z)\,dt
$$

with dynamics:

$$
\dot{\xi} = K_p^o\delta p_z - \delta v_z
$$

Closed-loop dynamics (vertical) become $\dot{x} = A_{cl}x$:

$$
\begin{bmatrix}
\delta\dot{p}_z\\\\
\delta\dot{v}_z\\\\
\dot{\xi}
\end{bmatrix} =
\begin{bmatrix}
0 & 1 & 0\\\\
K_p^iK_p^o/m & -K_p^i/m & K_i^i/m\\\\
K_p^o & -1 & 0
\end{bmatrix}
\begin{bmatrix}
\delta p_z\\\\
\delta v_z\\\\
\xi
\end{bmatrix}
$$

>[!IMPORTANT]
> Eigenvalues of $A_{cl}$ determine closed-loop stability, i.e., all 3 must have negative real part.


### Horizontal Subsystem
State $x = [\delta p_x, \delta v_x]$. No feedforrwad $\delta F_x=0$.

Controller contributions in Phase 1.5:

$$
\begin{aligned}
v_x^{ref} &= K_p^o \delta p_x\\
\delta F_x &= K_p^i(v_x^{ref} - \delta v_x) = K_p^i(K_p^o\delta p_x - \delta v_x)
\end{aligned}
$$

Closed loop dynamics (horizontal) become $\dot{x} = A_{cl}x$:

$$
\begin{bmatrix}
\delta \dot{p}_x\\\\
\delta \dot{v}_x
\end{bmatrix} =
\begin{bmatrix}
0 & 1\\\\
K_p^iK_p^o/m & -K_p^i/m
\end{bmatrix}
\begin{bmatrix}
\delta p_x\\\\
\delta v_x
\end{bmatrix}
$$

>[!NOTE]
> Because of no integrator term, the horizontal loop cannot reject steady-state disturbances like constant wind. This is expected and acceptable in Phase 1.5 since wind is treated as a disturbance.