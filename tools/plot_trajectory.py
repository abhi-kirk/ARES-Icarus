import argparse
import json
from datetime import datetime
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
import pandas as pd


def compute_metrics(df):
    """Compute key metrics from trajectory data"""
    metrics = {
        'flight_time': df['Time'].iloc[-1],
        'landing_velocity': df['Velocity'].iloc[-1],
        'initial_altitude': df['Altitude'].iloc[0],
        'fuel_used': df['Fuel'].iloc[0] - df['Fuel'].iloc[-1],
        'max_velocity': df['Velocity'].min(),  # Most negative = fastest downward
        'avg_throttle': df['Throttle'].mean(),
        'total_samples': len(df)
    }

    # Find time of max velocity
    max_vel_idx = df['Velocity'].idxmin()
    metrics['time_at_max_velocity'] = df['Time'].iloc[max_vel_idx]

    return metrics


def main():
    parser = argparse.ArgumentParser(
        description='Plot Icarus trajectory data with enhanced visualization'
    )
    parser.add_argument(
        '--controller', '-c',
        default='unknown',
        help='Controller type (e.g., pid, hoverslam, mpc). Default: latest'
    )
    parser.add_argument(
        '--input', '-i',
        default='src/results/trajectory.csv',
        help='Input CSV file path. Default: src/results/trajectory.csv'
    )
    parser.add_argument(
        '--output-dir', '-d',
        default='src/results',
        help='Output directory for plots. Default: src/results'
    )
    parser.add_argument(
        '--version', '-v',
        help='Controller version (e.g., v1, v2, test). Default: auto (timestamp)'
    )
    parser.add_argument(
        '--no-show',
        action='store_true',
        help='Do not display plot window (just save)'
    )
    parser.add_argument(
        '--dpi',
        type=int,
        default=150,
        help='DPI for saved plot. Default: 150'
    )

    args = parser.parse_args()

    # Read the CSV
    try:
        df = pd.read_csv(args.input)
    except FileNotFoundError:
        print(f"Error: Could not find trajectory file at {args.input}")
        print("Make sure you've run the simulation first!")
        return 1

    # Compute metrics
    metrics = compute_metrics(df)

    # Compute derived quantities
    # Use a central-difference gradient so acceleration doesn't start with an artificial
    # first-sample step (diff()/fillna(0) makes Acceleration start at 0.0).
    t = df['Time'].to_numpy(dtype=float)
    v = df['Velocity'].to_numpy(dtype=float)
    acc = np.gradient(v, t)
    # Avoid boundary artifacts at the first/last sample (one-sided differences amplify noise,
    # and the last point may be an interpolated touchdown sample).
    if len(acc) >= 2:
        acc[0] = acc[1]
        acc[-1] = acc[-2]
    df['Acceleration'] = acc

    # Generate timestamped filename
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    if args.version is None or args.version == 'auto':
        version_str = timestamp
    else:
        version_str = args.version

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    plot_file = output_dir / f"trajectory_{args.controller}_{version_str}.png"
    metrics_file = output_dir / f"metrics_{args.controller}_{version_str}.json"

    # Create figure with 6 subplots
    fig = plt.figure(figsize=(16, 10))
    gs = fig.add_gridspec(3, 3, hspace=0.3, wspace=0.3)

    # Main title with key metrics
    title = f'Icarus Trajectory Analysis - {args.controller.upper()} Controller\n'
    title += f'Flight Time: {metrics["flight_time"]:.1f}s | '
    title += f'Landing Velocity: {metrics["landing_velocity"]:.2f} m/s | '
    title += f'Fuel Used: {metrics["fuel_used"]:.1f} kg'

    fig.suptitle(title, fontsize=14, fontweight='bold', y=0.98)

    # Color scheme
    colors = {
        'altitude': '#2E86AB',
        'velocity': '#A23B72',
        'fuel': '#F18F01',
        'throttle': '#9D7263',
        'acceleration': '#6A4C93',
        'phase': '#6A4C93'
    }

    # 1. Altitude vs Time (large plot, top-left)
    ax1 = fig.add_subplot(gs[0, :2])
    ax1.plot(df['Time'], df['Altitude'], color=colors['altitude'], linewidth=2.5)
    ax1.set_title('Altitude vs Time')
    ax1.fill_between(df['Time'], 0, df['Altitude'], alpha=0.2, color=colors['altitude'])
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Altitude (m)')
    ax1.grid(True, alpha=0.3, linestyle='--')
    ax1.axhline(y=0, color='red', linestyle='--', linewidth=0.3, label='Ground')

    # Annotate landing
    ax1.annotate(
        f'Landing\n{metrics["flight_time"]:.1f}s\n{df["Altitude"].iloc[-1]:.1f} m',
        xy=(df['Time'].iloc[-1], df['Altitude'].iloc[-1]),
        xytext=(-50, 30),
        textcoords='offset points',
        arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0', color='black'),
        bbox=dict(boxstyle='round', pad=0.5, facecolor='yellow', alpha=0.7),
    )

    # 2. Velocity vs Time
    ax2 = fig.add_subplot(gs[0, 2])
    ax2.plot(df['Time'], df['Velocity'], color=colors['velocity'], linewidth=1.5)
    ax2.set_title('Velocity vs Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (m/s)')
    ax2.grid(True, alpha=0.3, linestyle='--')
    ax2.axhline(y=0, color='green', alpha=0.3, linestyle='--', linewidth=3)

    # Annotate max velocity
    max_vel_idx = df['Velocity'].idxmin()
    ax2.annotate(
        f'Max Descent\n{metrics["max_velocity"]:.2f} m/s',
        xy=(df['Time'].iloc[max_vel_idx], df['Velocity'].iloc[max_vel_idx]),
        xytext=(20, 20),
        textcoords='offset points',
        arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0.3', color='black'),
        bbox=dict(boxstyle='round', facecolor='orange', alpha=0.5),
        fontsize=12, fontweight='bold'
    )
    ax2.annotate(
        'Zero velocity',
        xy=(df['Time'].iloc[-1], 0),
        xytext=(-60, -20),
        textcoords='offset points',
        arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0.3', color='black'),
        fontsize=12
    )

    # 3. Acceleration vs Time
    ax3 = fig.add_subplot(gs[1, 0])
    ax3.plot(df['Time'], df['Acceleration'], color=colors['acceleration'], linewidth=2)
    ax3.set_title('Acceleration vs Time')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Acceleration (m/s²)')
    ax3.grid(True, alpha=0.3, linestyle='--')
    grav_line = ax3.axhline(y=-9.81, color='black', linestyle='--', linewidth=0.8, alpha=0.5, label='Gravity')
    ax3.axhline(y=0, color='black', linestyle='--', linewidth=1, alpha=0.8)

    # Tighten y-limits to reduce whitespace (keep 0 visible for sign reference).
    try:
        acc_min = float(np.nanmin(df['Acceleration'].to_numpy(dtype=float)))
        acc_max = float(np.nanmax(df['Acceleration'].to_numpy(dtype=float)))
        span = max(1e-6, acc_max - acc_min)
        pad = 0.15 * span
        y0 = min(acc_min - pad, -0.2)
        y1 = max(acc_max + pad, 0.2)
        ax3.set_ylim(y0, y1)
    except Exception:
        pass

    # Only show the gravity reference in the legend if it's within view.
    try:
        y0, y1 = ax3.get_ylim()
        if not (y0 <= -9.81 <= y1):
            grav_line.set_visible(False)
            grav_line.set_label('_nolegend_')
    except Exception:
        pass

    try:
        if grav_line.get_visible() and grav_line.get_label() != '_nolegend_':
            ax3.legend(loc='best', fontsize=8)
    except Exception:
        pass

    # Make the initial capture transient explicit (helps README readers).
    try:
        cap_idx = int(np.argmax(np.abs(v) > 5.0))
        if cap_idx > 0:
            ax3.annotate(
                'Acquire descent rate',
                xy=(t[cap_idx], df['Acceleration'].iat[cap_idx]),
                xytext=(t[cap_idx] + 15.0, df['Acceleration'].iat[cap_idx] - 2.0),
                arrowprops=dict(arrowstyle='->', color='black', alpha=0.7),
                fontsize=9,
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.7),
            )
    except Exception:
        pass

    # 4. Fuel vs Time
    ax4 = fig.add_subplot(gs[1, 1])
    ax4.plot(df['Time'], df['Fuel'], color=colors['fuel'], linewidth=2.5)
    ax4.fill_between(df['Time'], 0, df['Fuel'], alpha=0.3, color=colors['fuel'])
    ax4.set_title('Fuel Mass')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Fuel (kg)')
    ax4.grid(True, alpha=0.3, linestyle='--')

    # Add text showing fuel consumption
    ax4.text(
        0.95, 0.95,
        f'Used: {metrics["fuel_used"]:.1f} kg\nRemaining: {df["Fuel"].iloc[-1]:.1f} kg',
        transform=ax4.transAxes,
        verticalalignment='top',
        horizontalalignment='right',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
        fontsize=11
    )

    # 5. Throttle vs Time
    ax5 = fig.add_subplot(gs[1, 2])
    ax5.plot(df['Time'], df['Throttle'], color=colors['throttle'], linewidth=2.5)
    ax5.fill_between(df['Time'], 0, df['Throttle'], alpha=0.3, color=colors['throttle'])
    ax5.set_title('Throttle')
    ax5.set_xlabel('Time (s)')
    ax5.set_ylabel('Throttle (0-1)')
    ax5.grid(True, alpha=0.3, linestyle='--')
    ax5.axhline(y=1, alpha=0.5, linestyle='--', linewidth=1.65, color='red', label='Max')
    ax5.axhline(y=0, alpha=0.5, linestyle='--', linewidth=1, label='Min')

    # Add average throttle text
    ax5.text(
        0.95, 0.95,
        f'Avg: {metrics["avg_throttle"]:.3f}',
        transform=ax5.transAxes,
        horizontalalignment='right',
        bbox=dict(boxstyle='round', facecolor='lightyellow', alpha=0.8),
        fontsize=9
    )

    # Tighten y-limits to reduce whitespace while keeping the throttle range clear.
    try:
        thr = df['Throttle'].to_numpy(dtype=float)
        thr_min = float(np.nanmin(thr))
        thr_max = float(np.nanmax(thr))
        span = max(1e-6, thr_max - thr_min)
        pad = 0.20 * span
        y0 = max(0.0, thr_min - pad)
        y1 = min(1.0, thr_max + pad)
        # Ensure the window isn't unreasonably tight for readability.
        if (y1 - y0) < 0.15:
            mid = 0.5 * (y0 + y1)
            y0 = max(0.0, mid - 0.075)
            y1 = min(1.0, mid + 0.075)
        ax5.set_ylim(y0, y1)
    except Exception:
        pass

    # Annotate the main mid-flight transient (start of braking/flare).
    try:
        t = df['Time'].to_numpy(dtype=float)
        thr = df['Throttle'].to_numpy(dtype=float)
        baseline = float(np.nanmedian(thr[t <= min(1.0, t.max())]))
        dthr = np.zeros_like(thr)
        dthr[1:] = np.diff(thr) / np.diff(t)
        idx_candidates = np.where((thr > baseline + 0.01) & (dthr > 0.2))[0]
        if len(idx_candidates) > 0:
            i = int(idx_candidates[0])
            ax5.annotate(
                'Begin braking (flare)',
                xy=(t[i], thr[i]),
                xytext=(t[i] + 15.0, thr[i] + 0.03),
                arrowprops=dict(arrowstyle='->', color='black', alpha=0.7),
                fontsize=9,
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.75),
            )
    except Exception:
        pass

    # 6. Phase Diagram (Altitude vs Velocity)
    ax6 = fig.add_subplot(gs[2, :])

    # Phase portrait: time-colored trajectory (line + points) to avoid the appearance of
    # an "instantaneous" velocity jump in README screenshots.
    x = df['Velocity'].to_numpy(dtype=float)
    y = df['Altitude'].to_numpy(dtype=float)
    tt = df['Time'].to_numpy(dtype=float)

    points = np.column_stack([x, y])
    segments = np.stack([points[:-1], points[1:]], axis=1)
    seg_time = 0.5 * (tt[:-1] + tt[1:])

    norm = plt.Normalize(tt.min(), tt.max())
    lc = LineCollection(segments, cmap='viridis', norm=norm, linewidth=2.0, alpha=0.9)
    lc.set_array(seg_time)
    ax6.add_collection(lc)

    scatter = ax6.scatter(x, y, c=tt, cmap='viridis', s=10, alpha=0.35, linewidths=0)

    # Mark start and end
    ax6.scatter(df['Velocity'].iloc[0], df['Altitude'].iloc[0],
                color='green', s=200, marker='o', label='Start', zorder=5, edgecolors='black')
    ax6.scatter(df['Velocity'].iloc[-1], df['Altitude'].iloc[-1],
                color='red', s=200, marker='X', label='Landing', zorder=5, edgecolors='black')

    ax6.set_title('Phase Diagram', fontsize=11, fontweight='bold')
    ax6.set_xlabel('Velocity (m/s)')
    ax6.set_ylabel('Altitude (m)')
    ax6.legend(loc='lower left', fontsize=8)
    ax6.grid(True, alpha=0.3, linestyle='--')

    # Add colorbar for time
    cbar = plt.colorbar(lc, ax=ax6)
    cbar.set_label('Time (s)', fontsize=8)

    # Ensure the collection is fully visible
    ax6.set_xlim(x.min() - 0.5, x.max() + 0.5)
    ax6.set_ylim(min(0.0, y.min()) - 10.0, y.max() + 10.0)

    # Explicitly annotate the initial ramp segment.
    try:
        idx = int(np.argmax(np.abs(x) > 5.0))
        if idx > 0:
            ax6.annotate(
                'Acquire descent rate',
                xy=(x[idx], y[idx]),
                xytext=(x[idx] + 3.0, y[idx] - 120.0),
                textcoords='data',
                arrowprops=dict(arrowstyle='->', color='black', alpha=0.7),
                fontsize=9,
                bbox=dict(boxstyle='round', facecolor='white', alpha=0.7),
            )
    except Exception:
        pass

    # Add a few direction arrows along the phase trajectory (time flows Start -> Landing).
    try:
        n = len(x)
        if n > 50:
            arrow_idx = np.linspace(5, n - 10, 10, dtype=int)
            for i in arrow_idx:
                ax6.annotate(
                    '',
                    xy=(x[i + 1], y[i + 1]),
                    xytext=(x[i], y[i]),
                    arrowprops=dict(arrowstyle='->', color='black', alpha=0.25, lw=1.0),
                )
    except Exception:
        pass

    # Save plot
    plt.savefig(plot_file, dpi=args.dpi, bbox_inches='tight')
    print(f"Plot saved to: {plot_file}")

    # Save metrics
    metrics['controller'] = args.controller
    metrics['version'] = version_str
    metrics['timestamp'] = timestamp
    metrics['input_file'] = args.input

    with open(metrics_file, 'w') as f:
        json.dump(metrics, f, indent=2)
    print(f"Metrics saved to: {metrics_file}")

    print(f"\nKey Metrics:")
    print(f"  Flight Time: {metrics['flight_time']:.2f} s")
    print(f"  Landing Velocity: {metrics['landing_velocity']:.2f} m/s")
    print(f"  Max Descent Speed: {metrics['max_velocity']:.2f} m/s")
    print(f"  Fuel Used: {metrics['fuel_used']:.1f} kg")
    print(f"  Average Throttle: {metrics['avg_throttle']:.3f}")

    if not args.no_show:
        plt.show()
    plt.close()
    return 0


if __name__ == '__main__':
    exit(main())
