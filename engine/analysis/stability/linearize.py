import json
from pathlib import Path
import numpy as np

BASE_DIR = Path(__file__).parent.parent.parent
JSON_PATH = BASE_DIR / "config.json"

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

    # Closed-loop system matrix for vertical dynamics
    A_vertical = [
        [0, 1, 0], 
        [-Kp_inner_v * Kp_outer_v / mass, -Kp_inner_v / mass, Ki_inner_v / mass ],
        [-Kp_outer_v, -1, 0],
    ]

    # Closed-loop system matrix for horizontal dynamics
    A_horizontal = [
        [0, 1], 
        [-Kp_inner_h * Kp_outer_h / mass, -Kp_inner_h / mass],
    ]

    # Get eigenvalues for vertical and horizontal dynamics
    eigenvalues_vertical = np.linalg.eigvals(A_vertical)
    eigenvalues_horizontal = np.linalg.eigvals(A_horizontal)

    # Check for negative real parts to determine stability
    is_stable_vertical = np.all(np.real(eigenvalues_vertical) < 0)
    is_stable_horizontal = np.all(np.real(eigenvalues_horizontal) < 0)

    print("Vertical Dynamics Eigenvalues:", eigenvalues_vertical)
    print("Horizontal Dynamics Eigenvalues:", eigenvalues_horizontal)
    print("Vertical Dynamics Stable:", is_stable_vertical)
    print("Horizontal Dynamics Stable:", is_stable_horizontal)