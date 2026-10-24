import argparse
import json
from datetime import datetime
from pathlib import Path
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import pandas as pd


def compute_metrics(df, pad_x_final, pad_y_final):
    land_x = df['PosX'].iloc[-1]
    land_y = df['PosY'].iloc[-1]
    landing_error = np.sqrt((land_x - pad_x_final)**2 + (land_y - pad_y_final)**2)
    return {
        'flight_time':      df['Time'].iloc[-1],
        'landing_vel_z':    df['VelZ'].iloc[-1],
        'landing_vel_x':    df['VelX'].iloc[-1],
        'landing_vel_y':    df['VelY'].iloc[-1],
        'landing_pos_x':    land_x,
        'landing_pos_y':    land_y,
        'initial_altitude': df['PosZ'].iloc[0],
        'fuel_used':        df['Fuel'].iloc[0] - df['Fuel'].iloc[-1],
        'max_descent_speed':df['VelZ'].min(),
        'landing_error_m':  landing_error,
        'total_samples':    len(df),
    }


def main():
    parser = argparse.ArgumentParser(description='Plot Icarus 3DOF trajectory')
    parser.add_argument('--controller', '-c', default='cascade_pid')
    parser.add_argument('--input',      '-i', default='src/results/trajectory.csv')
    parser.add_argument('--config',           default='config.json',
                        help='Path to config.json for pad parameters')
    parser.add_argument('--output-dir', '-d', default='src/results')
    parser.add_argument('--version',    '-v')
    parser.add_argument('--no-show', action='store_true')
    parser.add_argument('--dpi', type=int, default=150)
    args = parser.parse_args()

    try:
        df = pd.read_csv(args.input)
    except FileNotFoundError:
        print(f"Error: Could not find trajectory file at {args.input}")
        return 1

    # Read pad parameters from config if available
    pad_x0, pad_y0, pad_vx, pad_vy = 0.0, 0.0, 0.0, 0.0
    try:
        with open(args.config) as f:
            cfg = json.load(f)
        sim = cfg.get('simulation', {})
        pad_x0 = sim.get('initial_pad_x_pos', 0.0)
        pad_y0 = sim.get('initial_pad_y_pos', 0.0)
        pad_vx = sim.get('initial_pad_x_velocity', 0.0)
        pad_vy = sim.get('initial_pad_y_velocity', 0.0)
    except Exception:
        pass

    t_land    = df['Time'].iloc[-1]
    pad_x_end = pad_x0 + pad_vx * t_land
    pad_y_end = pad_y0 + pad_vy * t_land

    metrics = compute_metrics(df, pad_x_end, pad_y_end)

    timestamp  = datetime.now().strftime("%Y%m%d_%H%M%S")
    version_str = args.version if (args.version and args.version != 'auto') else timestamp

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    plot_file    = output_dir / f"trajectory_{args.controller}_{version_str}.png"
    metrics_file = output_dir / f"metrics_{args.controller}_{version_str}.json"

    t = df['Time'].to_numpy(dtype=float)

    # ── Figure + grid ──────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(18, 13))
    # 3 rows × 3 cols. 3D spans gs[0:2, 0:2] (2×2 top-left).
    # Right strip: gs[0,2]=Alt, gs[1,2]=VelZ.
    # Bottom row:  gs[2,0]=VelXY, gs[2,1]=FxFy, gs[2,2]=Fz+Fuel dual-axis.
    gs = gridspec.GridSpec(3, 3, figure=fig,
                           hspace=0.65, wspace=0.38,
                           top=0.90, bottom=0.06,
                           height_ratios=[1, 1, 0.8])

    title  = f"Icarus 3DOF — {args.controller.upper()}  |  "
    title += f"t={metrics['flight_time']:.1f}s  |  "
    title += f"Vz={metrics['landing_vel_z']:.2f} m/s  |  "
    title += f"Fuel used={metrics['fuel_used']:.0f} kg  |  "
    title += f"Pad miss={metrics['landing_error_m']:.1f} m"
    fig.suptitle(title, fontsize=12, fontweight='bold', y=0.995)

    colors = dict(alt='#2E86AB', vel_z='#A23B72', vx='#2E86AB', vy='#C73E1D',
                  fz='#9D7263', fx='#2E86AB', fy='#C73E1D', fuel='#F18F01',
                  traj='#4FC3F7', pad='#1a6b2e')

    # ── Rows 0-1, cols 0-1: 3D trajectory (2×2) ──────────────────────────────
    ax_3d = fig.add_subplot(gs[0:2, 0:2], projection='3d')
    ax_3d.tick_params(labelsize=7)

    # Pre-compute pad path and data extents
    pad_times = np.linspace(0, t_land, 300)
    pad_xs = pad_x0 + pad_vx * pad_times
    pad_ys = pad_y0 + pad_vy * pad_times

    all_x = np.concatenate([df['PosX'].to_numpy(), pad_xs])
    all_y = np.concatenate([df['PosY'].to_numpy(), pad_ys])
    x_min, x_max = all_x.min(), all_x.max()
    y_min, y_max = all_y.min(), all_y.max()
    z_max = df['PosZ'].max()

    # Equalise horizontal ranges: both X and Y get the same span (the larger
    # of the two), centred on their midpoints, so neither axis has dead space.
    h_range = max(x_max - x_min, y_max - y_min)
    x_c = (x_min + x_max) / 2
    y_c = (y_min + y_max) / 2
    ax_3d.set_xlim(x_c - h_range / 2, x_c + h_range / 2)
    ax_3d.set_ylim(y_c - h_range / 2, y_c + h_range / 2)
    ax_3d.set_zlim(0, z_max * 1.04)

    # aspect=(1,1,1.8): Z gets 1.8× the visual height of X/Y — a compromise
    # between data-proportional (2.5×, near-vertical drop) and equal (1×,
    # horizontal axes look empty). Trajectory fills the cube well in all axes.
    ax_3d.set_box_aspect((1, 1, 1.8), zoom=1.5)

    # Rocket ground shadow
    ax_3d.plot(df['PosX'], df['PosY'], np.zeros(len(df)),
               color=colors['traj'], linewidth=0.8, alpha=0.25, linestyle='--', zorder=1)

    # Pad trajectory — magenta is bold and visible on white background
    pad_color = '#C2185B'
    ax_3d.plot(pad_xs, pad_ys, np.zeros(300),
               color=pad_color, linewidth=2.5, alpha=0.9, label='Pad trajectory', zorder=4)
    pole_h = z_max * 0.06
    ax_3d.plot([pad_x0, pad_x0], [pad_y0, pad_y0], [0, pole_h],
               color=pad_color, linewidth=1.2, linestyle='--', alpha=0.6)
    ax_3d.plot([pad_x_end, pad_x_end], [pad_y_end, pad_y_end], [0, pole_h],
               color=pad_color, linewidth=1.8, alpha=0.85)
    ax_3d.scatter(pad_x0, pad_y0, 0, color=pad_color, s=80, marker='o',
                  alpha=0.55, edgecolors='black', linewidths=0.6,
                  label='Pad (initial)', zorder=5)
    ax_3d.scatter(pad_x_end, pad_y_end, 0, color=pad_color, s=220, marker='s',
                  alpha=0.6, edgecolors='black', linewidths=0.8,
                  label='Pad (at landing)', zorder=6)

    # Rocket trajectory
    ax_3d.plot(df['PosX'], df['PosY'], df['PosZ'],
               color=colors['traj'], linewidth=2.5, alpha=0.95, label='Rocket', zorder=5)
    ax_3d.scatter(df['PosX'].iloc[0], df['PosY'].iloc[0], df['PosZ'].iloc[0],
                  color='green', s=100, marker='o', label='Start',
                  edgecolors='darkgreen', linewidths=0.8, zorder=7)
    ax_3d.scatter(df['PosX'].iloc[-1], df['PosY'].iloc[-1], 0,
                  color='red', s=160, marker='X', label='Landing',
                  edgecolors='black', linewidths=1.8, zorder=8)
    ax_3d.plot([df['PosX'].iloc[-1], pad_x_end],
               [df['PosY'].iloc[-1], pad_y_end], [0, 0],
               color='red', linewidth=1.2, linestyle=':', alpha=0.7)

    ax_3d.set_xlabel('X (m)', labelpad=4, fontsize=9)
    ax_3d.set_ylabel('Y (m)', labelpad=4, fontsize=9)
    ax_3d.set_zlabel('Alt (m)', labelpad=4, fontsize=9)
    ax_3d.set_title('3D Trajectory', fontsize=11, pad=8)
    ax_3d.legend(loc='upper left', fontsize=8, framealpha=0.8)
    ax_3d.view_init(elev=22, azim=-55)

    # Expand left/right into whitespace; keep top/bottom fixed so we don't
    # clip the figure title above or overlap the bottom-row charts below.
    pos = ax_3d.get_position()
    ax_3d.set_position([pos.x0 - 0.02, pos.y0 + 0.01,
                        pos.width + 0.06, pos.height - 0.01])

    # ── Row 0, col 2: Altitude ────────────────────────────────────────────────
    ax_alt = fig.add_subplot(gs[0, 2])
    ax_alt.plot(t, df['PosZ'], color=colors['alt'], linewidth=2)
    ax_alt.fill_between(t, 0, df['PosZ'], alpha=0.12, color=colors['alt'])
    ax_alt.set_xlabel('Time (s)', fontsize=9)
    ax_alt.set_ylabel('Altitude (m)', fontsize=9)
    ax_alt.set_title('Altitude vs Time', fontsize=10)
    ax_alt.grid(True, alpha=0.3, linestyle='--')
    ax_alt.axhline(0, color='red', linestyle='--', linewidth=0.6, alpha=0.7)

    # ── Row 1, col 2: Vertical Velocity ──────────────────────────────────────
    ax_vz = fig.add_subplot(gs[1, 2])
    ax_vz.plot(t, df['VelZ'], color=colors['vel_z'], linewidth=2)
    ax_vz.set_xlabel('Time (s)', fontsize=9)
    ax_vz.set_ylabel('Vel Z (m/s)', fontsize=9)
    ax_vz.set_title('Vertical Velocity', fontsize=10)
    ax_vz.grid(True, alpha=0.3, linestyle='--')
    ax_vz.axhline(0, color='green', alpha=0.35, linestyle='--', linewidth=1.8)

    # ── Row 2, col 0: Horizontal Velocities ──────────────────────────────────
    ax_vxy = fig.add_subplot(gs[2, 0])
    ax_vxy.plot(t, df['VelX'], color=colors['vx'], linewidth=1.5, label='Vx')
    ax_vxy.plot(t, df['VelY'], color=colors['vy'], linewidth=1.5, label='Vy')
    ax_vxy.set_xlabel('Time (s)', fontsize=9)
    ax_vxy.set_ylabel('Velocity (m/s)', fontsize=9)
    ax_vxy.set_title('Horizontal Velocities', fontsize=10)
    ax_vxy.legend(fontsize=8)
    ax_vxy.grid(True, alpha=0.3, linestyle='--')
    ax_vxy.axhline(0, color='black', alpha=0.2, linestyle='--', linewidth=1)

    # ── Row 2, col 1: Horizontal Thrust ──────────────────────────────────────
    ax_fxy = fig.add_subplot(gs[2, 1])
    ax_fxy.plot(t, df['Fx'], color=colors['fx'], linewidth=1.5, label='Fx')
    ax_fxy.plot(t, df['Fy'], color=colors['fy'], linewidth=1.5, label='Fy')
    ax_fxy.set_xlabel('Time (s)', fontsize=9)
    ax_fxy.set_ylabel('Force (N)', fontsize=9)
    ax_fxy.set_title('Horizontal Thrust (Fx, Fy)', fontsize=10)
    ax_fxy.legend(fontsize=8)
    ax_fxy.grid(True, alpha=0.3, linestyle='--')
    ax_fxy.axhline(0, color='black', alpha=0.2, linestyle='--', linewidth=1)
    trim_mask = t > 2.0
    if trim_mask.any():
        fxy_vals = np.concatenate([df['Fx'][trim_mask], df['Fy'][trim_mask]])
        lim = np.abs(fxy_vals).max() * 1.15
        ax_fxy.set_ylim(-lim, lim)
    ax_fxy.annotate('(initial spike clipped)', xy=(0.02, 0.96),
                    xycoords='axes fraction', fontsize=7, color='grey', va='top')

    # ── Row 2, col 2: Vertical Thrust + Fuel (dual y-axis) ───────────────────
    ax_fz = fig.add_subplot(gs[2, 2])
    ax_fz.plot(t, df['Fz'], color=colors['fz'], linewidth=1.5, label='Fz')
    hover_f = (1000.0 + df['Fuel'].iloc[0]) * 9.81
    ax_fz.axhline(hover_f, color=colors['fz'], linestyle='--', linewidth=0.8,
                  alpha=0.5, label=f'Hover ({hover_f:.0f} N)')
    ax_fz.set_xlabel('Time (s)', fontsize=9)
    ax_fz.set_ylabel('Fz (N)', fontsize=9, color=colors['fz'])
    ax_fz.tick_params(axis='y', labelcolor=colors['fz'])
    ax_fz.set_title('Vertical Thrust & Fuel', fontsize=10)
    ax_fz.grid(True, alpha=0.3, linestyle='--')

    ax_fuel = ax_fz.twinx()
    ax_fuel.plot(t, df['Fuel'], color=colors['fuel'], linewidth=1.5,
                 linestyle='--', label='Fuel')
    ax_fuel.fill_between(t, 0, df['Fuel'], alpha=0.08, color=colors['fuel'])
    ax_fuel.set_ylabel('Fuel (kg)', fontsize=9, color=colors['fuel'])
    ax_fuel.tick_params(axis='y', labelcolor=colors['fuel'])

    # Merge both legends into one
    lines_fz,  labels_fz  = ax_fz.get_legend_handles_labels()
    lines_fl,  labels_fl  = ax_fuel.get_legend_handles_labels()
    ax_fz.legend(lines_fz + lines_fl, labels_fz + labels_fl, fontsize=7, loc='upper right')

    # ── Save ──────────────────────────────────────────────────────────────────
    plt.savefig(plot_file, dpi=args.dpi, bbox_inches='tight')
    print(f"Plot saved to: {plot_file}")

    metrics.update(controller=args.controller, version=version_str,
                   timestamp=timestamp, input_file=args.input)
    with open(metrics_file, 'w') as f:
        json.dump(metrics, f, indent=2)
    print(f"Metrics saved to: {metrics_file}")

    print(f"\nKey Metrics:")
    print(f"  Flight Time:        {metrics['flight_time']:.2f} s")
    print(f"  Landing Vel Z:      {metrics['landing_vel_z']:.2f} m/s")
    print(f"  Landing Vel X/Y:    ({metrics['landing_vel_x']:.2f}, {metrics['landing_vel_y']:.2f}) m/s")
    print(f"  Landing Pos (X,Y):  ({metrics['landing_pos_x']:.1f}, {metrics['landing_pos_y']:.1f}) m")
    print(f"  Pad Pos at landing: ({pad_x_end:.1f}, {pad_y_end:.1f}) m")
    print(f"  Pad miss:           {metrics['landing_error_m']:.1f} m")
    print(f"  Max Descent Speed:  {metrics['max_descent_speed']:.2f} m/s")
    print(f"  Fuel Used:          {metrics['fuel_used']:.1f} kg")

    if not args.no_show:
        plt.show()
    plt.close()
    return 0


if __name__ == '__main__':
    exit(main())
