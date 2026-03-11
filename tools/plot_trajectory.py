import argparse
import json
from datetime import datetime
from pathlib import Path
import matplotlib.pyplot as plt
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
        description='Plot ARES trajectory data with enhanced visualization'
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
    df['Acceleration'] = df['Velocity'].diff() / df['Time'].diff()
    df['Acceleration'] = df['Acceleration'].fillna(0)  # First value is NaN

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
    title = f'ARES Trajectory Analysis - {args.controller.upper()} Controller\n'
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
    ax3.axhline(y=-9.81, color='black', linestyle='--', linewidth=0.8, alpha=0.5, label='Gravity')
    ax3.axhline(y=0, color='black', linestyle='--', linewidth=1, alpha=0.8)
    ax3.legend(loc='best', fontsize=8)

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

    # 6. Phase Diagram (Altitude vs Velocity)
    ax6 = fig.add_subplot(gs[2, :])

    # Color-code by time for phase diagram
    scatter = ax6.scatter(
        df['Velocity'],
        df['Altitude'],
        c=df['Time'],
        cmap='viridis',
        s=20,
        alpha=0.6
    )

    # Draw trajectory line
    ax6.plot(df['Velocity'], df['Altitude'], color=colors['phase'], linewidth=1.5, alpha=0.4)

    # Mark start and end
    ax6.scatter(df['Velocity'].iloc[0], df['Altitude'].iloc[0],
                color='green', s=200, marker='o', label='Start', zorder=5, edgecolors='black')
    ax6.scatter(df['Velocity'].iloc[-1], df['Altitude'].iloc[-1],
                color='red', s=200, marker='X', label='Landing', zorder=5, edgecolors='black')

    ax6.set_title('Phase Diagram', fontsize=11, fontweight='bold')
    ax6.set_xlabel('Velocity (m/s)')
    ax6.set_ylabel('Altitude (m)')
    ax6.legend(loc='best', fontsize=8)
    ax6.grid(True, alpha=0.3, linestyle='--')

    # Add colorbar for time
    cbar = plt.colorbar(scatter, ax=ax6)
    cbar.set_label('Time (s)', fontsize=8)

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
