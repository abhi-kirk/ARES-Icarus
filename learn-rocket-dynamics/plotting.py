import numpy as np
import matplotlib.pyplot as plt

# Color palettes
C_BLUE = "#2176AE"
C_ORANGE = "#E8651A"
C_GREEN_ACCENT = "#27AE60"
C_RED = "#C0392B"
C_PURPLE = "#8E44AD"
C_DARK = "#2C3E50"

C_BODY = "#2E86AB"
C_GHOST = "#CCCCCC"
C_ACCENT = "#E8430C"
C_ACCENT2 = "#8338EC"
C_GREEN = "#06D6A0"


# Style setup

def setup_style():
    """Apply shared rcParams. Notebooks add their own overrides after this."""
    plt.style.use("default")
    plt.rcParams.update({
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "axes.grid": True,
        "grid.alpha": 0.3,
        "font.size": 12,
        "figure.dpi": 100,
        "animation.html": "html5",
    })


# 3D helpers

def draw_ground_plane(ax, center, size, alpha=0.15, color="green"):
    """Draw a flat ground plane at z=0 on a 3D axes."""
    gx = np.array([-size, size, size, -size]) + center[0]
    gy = np.array([-size, -size, size, size]) + center[1]
    gz = np.zeros(4)
    ax.plot_surface(
        gx.reshape(2, 2), gy.reshape(2, 2), gz.reshape(2, 2),
        alpha=alpha, color=color, zorder=0
    )