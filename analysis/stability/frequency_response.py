import json
from pathlib import Path
import numpy as np
import control
import matplotlib.pyplot as plt

BASE_DIR = Path(__file__).parent.parent.parent
JSON_PATH = BASE_DIR / "config.json"
PLOT_DIR = Path(__file__).parent

def load_config():
    with open(JSON_PATH, "r") as f:
        config = json.load(f)
    return config

def print_margins(label, L):
    """Compute and print gain margin, phase margin, and crossover frequencies."""
    gm, pm, wgc, wpc = control.margin(L)
    gm_db = 20 * np.log10(gm) if gm > 0 else float("nan")
    print(f"\n{label}")
    print(f"  Gain margin:            {gm_db:.2f} dB  (at phase crossover {wpc:.4f} rad/s)")
    print(f"  Phase margin:           {pm:.2f} deg  (at gain crossover {wgc:.4f} rad/s)")

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
    # VERTICAL LOOP
    # ----------------------------------------------------------------

    # Inner open-loop: L_inner_v(s) = (Kp_i*s + Ki_i) / (m*s^2)
    # num coefficients: highest power first → [Kp_i, Ki_i]
    # den coefficients:                     → [m, 0, 0]
    L_inner_v_num = [Kp_inner_v, Ki_inner_v]
    L_inner_v_den = [mass, 0, 0]
    L_inner_v = control.tf(L_inner_v_num, L_inner_v_den)

    # Inner closed-loop: T_inner_v = L_inner_v / (1 + L_inner_v)
    T_inner_v = control.feedback(L_inner_v, 1)

    # Outer open-loop: L_outer_v(s) = Kp_o * T_inner_v(s) / s
    # control.tf([1], [1, 0]) is the 1/s integrator
    integrator = control.tf([1], [1, 0])
    L_outer_v = Kp_outer_v * T_inner_v * integrator

    # ----------------------------------------------------------------
    # HORIZONTAL LOOP
    # ----------------------------------------------------------------

    # Inner open-loop: L_inner_h(s) = Kp_i / (m*s)
    L_inner_h_num = [Kp_inner_h]
    L_inner_h_den = [mass, 0]
    L_inner_h = control.tf(L_inner_h_num, L_inner_h_den)

    # Inner closed-loop: T_inner_h = L_inner_h / (1 + L_inner_h)
    T_inner_h = control.feedback(L_inner_h, 1)

    # Outer open-loop: L_outer_h(s) = Kp_o * T_inner_h(s) / s
    L_outer_h = Kp_outer_h * T_inner_h * integrator

    # ----------------------------------------------------------------
    # PRINT MARGINS
    # ----------------------------------------------------------------
    print("=" * 55)
    print(f"Stability margins at mid-flight mass = {mass:.0f} kg")
    print("=" * 55)
    print_margins("Vertical   — inner loop", L_inner_v)
    print_margins("Vertical   — outer loop", L_outer_v)
    print_margins("Horizontal — inner loop", L_inner_h)
    print_margins("Horizontal — outer loop", L_outer_h)

    # ----------------------------------------------------------------
    # BODE PLOTS
    # ----------------------------------------------------------------
    omega = np.logspace(-4, 2, 1000)  # frequency range: 0.0001 to 100 rad/s

    fig, axes = plt.subplots(4, 2, figsize=(14, 18))
    fig.suptitle(f"Bode Plots — Open-Loop TFs (mass = {mass:.0f} kg)", fontsize=14)

    loops = [
        (L_inner_v, "Vertical inner (PI)",   axes[0]),
        (L_outer_v, "Vertical outer (P+PI)",  axes[1]),
        (L_inner_h, "Horizontal inner (P)",   axes[2]),
        (L_outer_h, "Horizontal outer (P+P)", axes[3]),
    ]

    for L, title, (ax_mag, ax_phase) in loops:
        mag, phase, omega_out = control.bode(L, omega, plot=False)
        mag_db = 20 * np.log10(mag)
        phase_deg = np.degrees(phase)

        ax_mag.semilogx(omega_out, mag_db)
        ax_mag.axhline(0, color="gray", linestyle="--", linewidth=0.8)  # 0 dB line
        ax_mag.set_title(title)
        ax_mag.set_ylabel("Magnitude (dB)")
        ax_mag.set_xlabel("Frequency (rad/s)")
        ax_mag.grid(True, which="both", alpha=0.3)

        ax_phase.semilogx(omega_out, phase_deg)
        ax_phase.axhline(-180, color="gray", linestyle="--", linewidth=0.8)  # -180 deg line
        ax_phase.set_ylabel("Phase (deg)")
        ax_phase.set_xlabel("Frequency (rad/s)")
        ax_phase.grid(True, which="both", alpha=0.3)

    plt.tight_layout()
    plot_path = PLOT_DIR / "bode_plots.png"
    plt.savefig(plot_path, dpi=150)
    print(f"\nBode plots saved to {plot_path}")