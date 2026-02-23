# Classical Mechanics Notes
Based on _The Theoretical Minimum_ by Leonard Susskind.

## Important Identities

- _Derivative_:

$$\frac{df(t)}{dt} = \lim_{\Delta t \rightarrow 0}\frac{\Delta f}{\Delta t} = \lim_{\Delta t \rightarrow 0}\frac{f(t+\Delta t) - f(t)}{\Delta t}$$

- _Potential Energy Principle_:
  - Intuition: The force is always directed in a way that pushes the particle toward lower potential energy $V$.

$$F(x) = -\frac{d}{dx}V(x)$$

- _Lagrangian_:
  - $T - V$, where $T$ is kinetic energy.

- _Action_:
  - For a trajectory $x(t)$:

$$A = \int_{t_0}^{t_1} \big(T(x(t)) - V(x(t))\big)\,dt = \int_{t_0}^{t_1} L(x(t), \dot{x}(t))\,dt$$

- _Euler–Lagrange Equation_:
  - From the _Principle of Least Action_ ($\delta A(x) = 0$), which gives:

$$\frac{d}{dt}\Big(\frac{\partial L}{\partial \dot{x}}\Big) - \frac{\partial L}{\partial x} = 0$$

- _Hamiltonian_:
  - $T + V$, or total energy, which gives:

$$\frac{dH}{dt} = -\frac{\partial L}{\partial t}$$

- _Phase Space Hamiltonian Equation_:
  - For conjugate momentum $p$ and generalized coordinates $q$:

$$\dot{p} = -\frac{\partial H}{\partial q}$$

$$\dot{q} = \frac{\partial H}{\partial p}$$


## Circular Motion

- _Angular Frequency_: Number of radians that the angle advances in unit time.
- The most general (counterclockwise) uniform circular motion about the origin has the mathematical form:

$$x(t) = R\cos(\omega t)$$

$$y(t) = R\sin(\omega t)$$

where $\omega$ is the _angular frequency_, i.e. $\omega = 2\pi/T$.
- Velocity and acceleration components can be easily calculated, taking the derivatives.
- Acceleration of a circular orbit is parallel to the position vector, but it is oppositely directed.
  - In other words, the acceleration vector points inward toward the origin.
- Position and velocity vectors are orthogonal.


## Minimization

For a local minima for functions of single variable:
- A necessary condition is that the point is _stationary_, i.e. its derivative wrt to the independent variable is zero.
- A sufficient condition is that the second derivative of the stationary point is positive.
  - If second derivative is zero, then that is called an _inflection point_ where the derivative changes from positive to negative or vice versa.

For a local minima for multi-variate functions:
- If the _determinant_ and the _trace_ of the **Hessian** is positive, then the point is a local minima.
- If the determinant is positive and trace is negative, then the point is a local maxima.
- If the determinant is negative, then irrespective of the trace, the point is a _saddle point_.

Hessian:

$$\begin{pmatrix}
\frac{\partial^2 f}{\partial x^2} & \frac{\partial^2 f}{\partial x \partial y} \\
\frac{\partial^2 f}{\partial y \partial x} & \frac{\partial^2 f}{\partial y^2}
\end{pmatrix}$$


## Principle of Least/Stationary Action

- _Action_:
  - Since action depends on the $x$’s _trajectory_, it is not simply an ordinary function of a few variables. It depends on an infinity of variables, i.e., all possible forms the trajectory can take.
  - Action is the function of the entire trajectory (i.e., a function of a function, a _functional_), and simply assigns a single value to each trajectory.
  - Minimizing a functional is the subject of a branch of mathematics called the _calculus of variations_.

- _Principle_:
  - When the physical system moves from one state to another, it chooses the path that minimizes (makes stationary) the Action.
  - Action is the accumulation of Lagrangian over the time duration of motion.
  - Instead of thinking about forces pushing and pulling an object moment-by-moment (Newtonian view), the Principle of Least Action looks at the motion as a whole, optimizing the balance between kinetic and potential energy over the entire journey.
  - The resulting Euler–Lagrange equation is just another form of Newton's second law $F=ma$.

## Hamiltonion Phase-Space Formulation
If at any time we know the exact values of all the coordinates and momenta, and we know the form of the Hamiltonion, Hamilton's equations will tell us the corresponding quantities an infinitesimal time later. By a process of successive updating, we can determine a trajectory through phase space. 