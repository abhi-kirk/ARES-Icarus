"""
Generate publication-quality plots of the success trajectory of a model over time.
"""

import argparse
import json
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd


def compute_metrics(df):
    """Compute key metrics from the trajectory data."""
    metrics = {
        "flight_time": df["Time".iloc[-1]],
        "landing_velocity": df["Velocity"].iloc[-1],
        "initial_altitude": df["Altitude"].iloc[0],
        "fuel_used": df["Fuel"].iloc[0] - df["Fuel"].iloc[-1],
        "max_velocity": df["Velocity"].min(),
        "avg_throttle": df["Throttle"].mean(),
        "total_samples": len(df),
    }

    max_vel_idx = df["Velocity"].idxmin()
    metrics["time_at_max_velocity"] = df["Time"].iloc[max_vel_idx]

    return metrics

def identify_phases(df, alt_high=500, alt_med=100, alt_landing=10):
    """Identify different phases of the flight based on altitude."""
    phases = []
    for i, row in df.iterrows():
        alt = row["Altitude"]
        if alt > alt_high:
            phases.append("High Descent")
        elif alt > alt_med:
            phases.append("Medium Descent")
        elif alt > alt_landing:
            phases.append("Hover Phase")
        else:
            phases.append("Final Landing")
    return phases

def main():
    parser = argparse.ArgumentParser(description="Plot success trajectory of a model.")
    parser.add_argument("--input", "-i", default="src/results/trajectory.csv", help="Input CSV file")
    parser.add_argument("--output", "-o", default="src/results/trajectory_success_final.png", help="Output plot file")
    parser.add_argument("--dpi", type=int, default=300, help="DPI for the output plot")
    args = parser.parse_args()

    # Read data
    df = pd.read_csv(args.input)
    metrics = compute_metrics(df)

    # Compute additional quantities
    df["Acceleration"] = df["Velocity"].diff() / df["Time"].diff()
    df["Acceleration"] = df["Acceleration"].fillna(0)

    # Smooth acceleration for cleaner viz
    window = 5
    df["Acceleration_smooth"] = df["Acceleration"].rolling(window=window, center=True).mean()
    df["Acceleration_smooth"] = df["Acceleration_smooth"].fillna(method="bfill").fillna(method="ffill")

    # Identify phases
    df["Phase"] = identify_phases(df)

    # Calculate thrust-to-weight ratio over time
    DRY_MASS = 1000  # kg
    GRAVITY = 9.81  # m/s^2
    MAX_THRUST = 25000  # N

    df["Total_Mass"] = DRY_MASS + df["Fuel"]
    df["Weight"] = df["Total_Mass"] * GRAVITY
    df["Thrust"] = df["Throttle"] * MAX_THRUST
    df["TW_Ratio"] = df["Thrust"] / df["Weight"]

    # Create figure with improved layout
    fig = plt.figure(figsize=(18, 11))
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.28)

    # Color scheme
    colors = {
        "altitude": "#1f77b4",
        "velocity": "#ff7f0e",
        "acceleration": "#2ca02c",
        "throttle": "#d62728",
        "tw_ratio": "#9467bd",
    }

    # Phase colors for backgrounds
    phase_colors = {
        "High Descent": "#e0f7fa",
        "Medium Descent": "#b2ebf2",
        "Hover Phase": "#80deea",
        "Final Landing": "#4dd0e1",
    }

    # Helper function to add phase backgrounds
    def add_phase_background(ax, df, alpha=0.15):
        current_phase = df["Phase"].iloc[0]
        start_time = df["Time"].iloc[0]
        for i in range(1, len(df)):
            if df["Phase"].iloc[i] != current_phase or i == len(df) - 1:
                end_time = df["Time"].iloc[i]
                ax.axvspan(start_time, end_time, facecolor=phase_colors.get(current_phase, "white"), alpha=alpha, zorder=0)
                current_phase = df["Phase"].iloc[i]
                start_time = end_time


    # 1. Altitude vs Time
    ax1 = fig.add_subplot(gs[0, :2])
    add_phase_background(ax1, df)
    ax1.plot(df["Time"], df["Altitude"], color=colors["altitude"], linewidth=2, zorder=3)
    ax1.fill_between(df["Time"], 0, df["Altitude"], color=colors["altitude"], alpha=0.2, zorder=2)
    ax1.axhline(y=0, color="red", linestyle="-", linewidth=2, zorder=1, alpha=0.7, label="Ground")

    # Annotatate phase transitions
    for alt, label in [(500, "500m"), (100, "100m"), (10, "10m")]:
        idx = (df["Altitude"] - alt).abs().idxmin()
        if idx < len(df) - 1:
            ax1.axvline(x=df["Time"].iloc[idx], color="gray", linestyle="--", linewidth=1, alpha=0.5)
            ax1.text(df["Time"].iloc[idx] + 0.5, alt + 20, label, color="gray", fontsize=10, verticalalignment="bottom")
    
    ax1.set_ylabel("Altitude (m)", fontsize=13, fontweight="bold")
    ax1.set_xlabel("Time (s)", fontsize=13, fontweight="bold")
    ax1.set_title("Altitude Profile", fontsize=14, fontweight="bold", pad=10)
    ax1.grid(True, alpha=0.3, linestyle="--", linewidth=0.8)
    ax1.set_xlim(0, df["Time"].max())

    # 2. Velocity vs Time with phase labels
    ax2 = fig.add_subplot(gs[1, :2])
    add_phase_background(ax2, df)
    ax2.plot(df["Time"], df["Velocity"], color=colors["velocity"], linewidth=2.5, zorder=3)
    ax2.axhline(y=0, color="red", linestyle="--", linewidth=1.5, alpha=0.5, label="Zero Velocity")

    # Add reference velocity times
    for vel, label in [(-15, '-15 m/s (High)'), (-8, '-8 m/s (Medium)'), (-2, '-2 m/s (Hover)')]:
        ax2.axhline(y=vel, color="gray", linestyle=":", linewidth=1, alpha=0.4)
        ax2.text(5, vel, label, fontsize=8, va="bottom", alpha=0.6)

    ax2.set_ylabel("Velocity (m/s)", fontsize=13, fontweight="bold")
    ax2.set_xlabel("Time (s)", fontsize=13, fontweight="bold")
    ax2.set_title("Velocity Profile", fontsize=14, fontweight="bold", pad=10)
    ax2.grid(True, alpha=0.3, linestyle="--", linewidth=0.8)
    ax2.set_xlim(0, df["Time"].max())

    # 3. Smoothed Acceleration vs Time
    ax3 = fig.add_subplot(gs[2, :2])
    add_phase_background(ax3, df)
    ax3.plot(df["Time"], df["Acceleration_smooth"], color=colors["acceleration"], linewidth=2, zorder=3)
    ax3.fill_between(df["Time"], -10, df["Acceleration_smooth"], color=colors["acceleration"], alpha=0.3, zorder=2)
    ax3.axhline(y=0, color="red", linestyle="--", linewidth=1.5, alpha=0.5, label="Zero Acceleration")
    ax3.axhline(y=GRAVITY, color="green", linestyle="--", linewidth=1.5, alpha=0.5, label="Gravity (9.81 m/s²)")
    ax3.set_ylabel("Acceleration (m/s²)", fontsize=13, fontweight="bold")
    ax3.set_xlabel("Time (s)", fontsize=13, fontweight="bold")
    ax3.set_title("Acceleration Profile", fontsize=14, fontweight="bold", pad=10)
    ax3.grid(True, alpha=0.3, linestyle="--", linewidth=0.8)
    ax3.set_xlim(0, df["Time"].max())
    ax3.set_ylim(-12, 8)

    # 4. Fuel Mass
    ax4 = fig.add_subplot(gs[0, 2])
    ax4.plot(df["Time"], df["Fuel"], color=colors["fuel"], linewidth=2, zorder=3)
    ax4.fill_between(df["Time"], 0, df["Fuel"], color=colors["fuel"], alpha=0.2, zorder=2)

    # Add text box with fuel info
    fuel_text = f"Initial Fuel: {df['Fuel'].iloc[0]:.2f} kg\nFinal Fuel: {df['Fuel'].iloc[-1]:.2f} kg\nFuel Used: {metrics['fuel_used']:.2f} kg"
    ax4.text(0.5, 0.5, fuel_text, transform=ax4.transAxes, fontsize=10, verticalalignment="center", horizontalalignment="center", bbox=dict(facecolor='white', alpha=0.8, edgecolor='gray'))
    ax4.set_ylabel("Fuel Mass (kg)", fontsize=13, fontweight="bold")
    ax4.set_xlabel("Time (s)", fontsize=13, fontweight="bold")
    ax4.set_title("Fuel Consumption", fontsize=14, fontweight="bold", pad=10)
    ax4.grid(True, alpha=0.3, linestyle="--", linewidth=0.8)
    ax4.set_xlim(0, df["Time"].max())