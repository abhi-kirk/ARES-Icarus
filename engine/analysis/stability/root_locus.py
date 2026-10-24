import json
from pathlib import Path
import numpy as np
import control
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

BASE_DIR = Path(__file__).parent.parent.parent
JSON_PATH = BASE_DIR / "config.json"
PLOT_DIR = Path(__file__).parent

def load_config():
    with open(JSON_PATH, "r") as f:
        config = json.load(f)
    return config

if __name__ == "__main__":
    config = load_config()
    Kp_outer_v = config["outer_pid_vertical_position"]["gains"]["kp"]
    Ki_inner_v = config["inner_pid_vertical_velocity"]["gains"]["ki"]
    Kp_inner_v = config["inner_pid_vertical_velocity"]["gains"]["kp"]
    Kp_outer_h = config["outer_pid_horizontal_position"]["gains"]["kp"]
    Kp_inner_h = config["inner_pid_horizontal_velocity"]["gains"]["kp"]
    initial_fuel = config["simulation"]["initial_fuel"]

    mass = 1000.0 + initial_fuel / 2

    # ----------------------------------------------------------------
    # Build G(s) for each outer loop: L_outer = K * G(s)
    # Factor out Kp_outer so K is the variable and G captures everything else.
    #
    # Vertical:
    #   L_inner_v = (Kp_inner_v * s + Ki_inner_v) / (mass * s^2)
    #   T_inner_v = L_inner_v / (1 + L_inner_v)
    #   G_v(s) = T_inner_v(s) / s        (Kp_outer_v factored out)
    #
    # Horizontal:
    #   L_inner_h = Kp_inner_h / (mass * s)
    #   T_inner_h = L_inner_h / (1 + L_inner_h)
    #   G_h(s) = T_inner_h(s) / s        (Kp_outer_h factored out)
    # ----------------------------------------------------------------

    integrator = control.tf([1], [1, 0])

    # Vertical inner loop (fixed — not the parameter being varied)
    L_inner_v = control.tf([Kp_inner_v, Ki_inner_v], [mass, 0, 0])
    T_inner_v = control.feedback(L_inner_v, 1)
    G_v = T_inner_v * integrator  # Kp_outer_v factored out

    # Horizontal inner loop (fixed)
    L_inner_h = control.tf([Kp_inner_h], [mass, 0])
    T_inner_h = control.feedback(L_inner_h, 1)
    G_h = T_inner_h * integrator  # Kp_outer_h factored out

    # ----------------------------------------------------------------
    # Compute closed-loop poles at the actual operating gain,
    # so we can mark them on the root locus.
    #
    # Closed-loop poles of K*G / (1 + K*G) at K = Kp_outer are the
    # roots of the characteristic polynomial of the full outer loop.
    # ----------------------------------------------------------------
    CL_v = control.feedback(Kp_outer_v * G_v, 1)
    CL_h = control.feedback(Kp_outer_h * G_h, 1)

    actual_poles_v = control.poles(CL_v)
    actual_poles_h = control.poles(CL_h)

    print("Vertical closed-loop poles at actual Kp_outer_v =", Kp_outer_v)
    for p in actual_poles_v:
        print(f"  {p:.6f}")

    print("\nHorizontal closed-loop poles at actual Kp_outer_h =", Kp_outer_h)
    for p in actual_poles_h:
        print(f"  {p:.6f}")

    # ----------------------------------------------------------------
    # Root locus plots
    # ----------------------------------------------------------------
    fig, (ax_v, ax_h) = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(f"Root Locus — Varying $K_p^o$ (mass = {mass:.0f} kg)", fontsize=13)

    for ax, G, actual_poles, Kp_actual, title in [
        (ax_v, G_v, actual_poles_v, Kp_outer_v, "Vertical outer loop"),
        (ax_h, G_h, actual_poles_h, Kp_outer_h, "Horizontal outer loop"),
    ]:
        # control.root_locus returns (roots array, gains array)
        roots, gains = control.root_locus(G, plot=False)

        # roots shape: (n_gains, n_poles) — plot each branch
        for branch in range(roots.shape[1]):
            branch_real = np.real(roots[:, branch])
            branch_imag = np.imag(roots[:, branch])
            ax.plot(branch_real, branch_imag, color="steelblue", linewidth=1.2)

        # Mark open-loop poles (K=0) with x
        ol_poles = control.poles(G)
        ax.plot(np.real(ol_poles), np.imag(ol_poles),
                "rx", markersize=10, markeredgewidth=2, label="Open-loop poles (K=0)")

        # Mark open-loop zeros (K→∞) with o
        ol_zeros = control.zeros(G)
        if len(ol_zeros) > 0:
            ax.plot(np.real(ol_zeros), np.imag(ol_zeros),
                    "go", markersize=8, markeredgewidth=2, fillstyle="none", label="Open-loop zeros (K→∞)")

        # Mark closed-loop poles at actual operating gain with filled square
        ax.plot(np.real(actual_poles), np.imag(actual_poles),
                "k^", markersize=9, label=f"Actual poles ($K_p^o$ = {Kp_actual})")

        # Stability boundary
        ax.axvline(0, color="gray", linestyle="--", linewidth=0.8)

        ax.set_title(title)
        ax.set_xlabel("Real axis")
        ax.set_ylabel("Imaginary axis")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plot_path = PLOT_DIR / "root_locus.png"
    plt.savefig(plot_path, dpi=150)
    print(f"\nRoot locus plot saved to {plot_path}")
