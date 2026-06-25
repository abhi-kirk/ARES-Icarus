import numpy as np


# Full trajectory integrators

def integrate_euler(deriv, x0, t_span, dt):
    """Forward Euler integration. Simple but energy-drifting."""
    t = np.arange(t_span[0], t_span[1], dt)
    x = np.zeros((len(t), len(x0)))
    x[0] = x0
    for i in range(len(t)-1):
        x[i+1] = x[i] + dt * deriv(t[i], x[i])
    return t, x


def integrate_rk4(deriv, x0, t_span, dt):
    """Classic 4th-order Runge-Kutta. The standard for a reason."""
    t = np.arange(t_span[0], t_span[1], dt)
    x = np.zeros((len(t), len(x0)))
    x[0] = x0
    for i in range(len(t)-1):
        k1 = deriv(t[i],             x[i])
        k2 = deriv(t[i] + 0.5 * dt, x[i] + 0.5 * dt * k1)
        k3 = deriv(t[i] + 0.5 * dt, x[i] + 0.5 * dt * k2)
        k4 = deriv(t[i] + dt,        x[i] + dt * k3)
        x[i+1] = x[i] + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    return t, x