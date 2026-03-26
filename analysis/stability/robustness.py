import json
from pathlib import Path
import numpy as np
import control
import matplotlib.pyplot as plt

BASE_DIR = Path(__file__).parent.parent.parent
JSON_PATH = BASE_DIR / "config.json"
PLOT_DIR = Path(__file__).parent

MASS_DRY = 1000.0  # kg, no fuel

def load_config():
    with open(JSON_PATH, "r") as f:
        config = json.load(f)
    return config

def build_closed_loop_matrices(mass, Kp_outer_v, Ki_inner_v, Kp_inner_v,
                                Kp_outer_h, Kp_inner_h):
    """Return A_vertical (3x3) and A_horizontal (2x2) for a given mass."""
    A_vertical = [
        [0,                              1,               0              ],
        [-Kp_inner_v * Kp_outer_v / mass, -Kp_inner_v / mass, Ki_inner_v / mass],
        [-Kp_outer_v,                    -1,               0              ],
    ]
    A_horizontal = [
        [0,                               1               ],
        [-Kp_inner_h * Kp_outer_h / mass, -Kp_inner_h / mass],
    ]
    return A_vertical, A_horizontal

def build_outer_loop_tfs(mass, Kp_outer_v, Ki_inner_v, Kp_inner_v,
                          Kp_outer_h, Kp_inner_h):
    """Return the outer open-loop TFs L_outer_v and L_outer_h for a given mass."""
    integrator = control.tf([1], [1, 0])

    L_inner_v = control.tf([Kp_inner_v, Ki_inner_v], [mass, 0, 0])
    T_inner_v = control.feedback(L_inner_v, 1)
    L_outer_v = Kp_outer_v * T_inner_v * integrator

    L_inner_h = control.tf([Kp_inner_h], [mass, 0])
    T_inner_h = control.feedback(L_inner_h, 1)
    L_outer_h = Kp_outer_h * T_inner_h * integrator

    return L_outer_v, L_outer_h

def get_phase_margin(L):
    """Return phase margin in degrees, or NaN if no gain crossover exists."""
    try:
        gm, pm, wgc, wpc = control.margin(L)
        # control.margin returns inf or nan when crossover doesn't exist
        if pm is None or np.isnan(pm) or np.isinf(pm):
            return float("nan")
        return pm
    except Exception:
        return float("nan")

if __name__ == "__main__":
    config = load_config()
    Kp_outer_v = config["outer_pid_vertical_position"]["gains"]["kp"]
    Ki_inner_v = config["inner_pid_vertical_velocity"]["gains"]["ki"]
    Kp_inner_v = config["inner_pid_vertical_velocity"]["gains"]["kp"]
    Kp_outer_h = config["outer_pid_horizontal_position"]["gains"]["kp"]
    Kp_inner_h = config["inner_pid_horizontal_velocity"]["gains"]["kp"]
    initial_fuel = config["simulation"]["initial_fuel"]

    mass_full = MASS_DRY + initial_fuel   # full fuel
    mass_mid  = MASS_DRY + initial_fuel / 2.0
    mass_dry  = MASS_DRY                  # no fuel

    # Sweep mass from dry to full fuel in 50 steps
    mass_array = np.linspace(mass_dry, mass_full, 50)

    # Storage arrays for results
    eig_v_real_max  = np.zeros(len(mass_array))  # most unstable vertical eigenvalue (max real part)
    eig_h_real_max  = np.zeros(len(mass_array))  # most unstable horizontal eigenvalue (max real part)
    pm_v_outer      = np.zeros(len(mass_array))  # vertical outer phase margin
    pm_h_outer      = np.zeros(len(mass_array))  # horizontal outer phase margin

    for i, mass in enumerate(mass_array):
        # Eigenvalues
        A_v, A_h = build_closed_loop_matrices(
            mass, Kp_outer_v, Ki_inner_v, Kp_inner_v, Kp_outer_h, Kp_inner_h
        )
        eigs_v = np.linalg.eigvals(A_v)
        eigs_h = np.linalg.eigvals(A_h)

        # Track the eigenvalue with the largest real part (most dangerous)
        eig_v_real_max[i] = np.max(np.real(eigs_v))
        eig_h_real_max[i] = np.max(np.real(eigs_h))

        # Phase margins
        L_outer_v, L_outer_h = build_outer_loop_tfs(
            mass, Kp_outer_v, Ki_inner_v, Kp_inner_v, Kp_outer_h, Kp_inner_h
        )
        pm_v_outer[i] = get_phase_margin(L_outer_v)
        pm_h_outer[i] = get_phase_margin(L_outer_h)

    # ----------------------------------------------------------------
    # Plot 1: Max eigenvalue real part vs mass
    # ----------------------------------------------------------------
    fig1, ax1 = plt.subplots(figsize=(9, 5))

    ax1.plot(mass_array, eig_v_real_max, label="Vertical (max Re[λ])", color="tomato")
    ax1.plot(mass_array, eig_h_real_max, label="Horizontal (max Re[λ])", color="steelblue")
    ax1.axhline(0, color="black", linestyle="--", linewidth=0.9, label="Stability boundary")

    # Mark the three operating points
    for m_label, m_val in [("dry", mass_dry), ("mid", mass_mid), ("full fuel", mass_full)]:
        ax1.axvline(m_val, color="gray", linestyle=":", linewidth=0.8)
        ax1.text(m_val + 5, ax1.get_ylim()[0] if ax1.get_ylim()[0] != 0 else -0.001,
                 m_label, fontsize=8, color="gray")

    ax1.set_title("Robustness vs Mass — Max Eigenvalue Real Part")
    ax1.set_xlabel("Mass (kg)")
    ax1.set_ylabel("Max Re[λ]  (< 0 = stable)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    fig1.tight_layout()
    fig1.savefig(PLOT_DIR / "robustness_eigenvalues.png", dpi=150)

    # ----------------------------------------------------------------
    # Plot 2: Phase margin vs mass
    # ----------------------------------------------------------------
    fig2, ax2 = plt.subplots(figsize=(9, 5))

    ax2.plot(mass_array, pm_v_outer, label="Vertical outer PM", color="tomato")
    ax2.plot(mass_array, pm_h_outer, label="Horizontal outer PM", color="steelblue")
    ax2.axhline(45, color="black", linestyle="--", linewidth=0.9, label="45° rule of thumb")
    ax2.axhline(0,  color="black", linestyle="-",  linewidth=0.5)

    for m_label, m_val in [("dry", mass_dry), ("mid", mass_mid), ("full fuel", mass_full)]:
        ax2.axvline(m_val, color="gray", linestyle=":", linewidth=0.8)
        ax2.text(m_val + 5, 5, m_label, fontsize=8, color="gray")

    ax2.set_title("Robustness vs Mass — Outer Loop Phase Margins")
    ax2.set_xlabel("Mass (kg)")
    ax2.set_ylabel("Phase Margin (deg)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    fig2.tight_layout()
    fig2.savefig(PLOT_DIR / "robustness_phase_margins.png", dpi=150)

    print("Robustness plots saved to", PLOT_DIR)
    print(f"\nMass range: {mass_dry:.0f} kg (dry) → {mass_full:.0f} kg (full fuel)")
    print(f"\n{'Mass':>10}  {'Re[λ]_v':>12}  {'Re[λ]_h':>12}  {'PM_v (deg)':>12}  {'PM_h (deg)':>12}")
    print("-" * 65)
    for m_label, m_val in [("dry", mass_dry), ("mid", mass_mid), ("full fuel", mass_full)]:
        idx = np.argmin(np.abs(mass_array - m_val))
        print(f"{m_label:>10}  {eig_v_real_max[idx]:>12.5f}  {eig_h_real_max[idx]:>12.5f}"
              f"  {pm_v_outer[idx]:>12.2f}  {pm_h_outer[idx]:>12.2f}")
