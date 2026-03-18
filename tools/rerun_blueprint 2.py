"""
Apply a Rerun blueprint to trajectory.rrd so the viewer opens with the same
layout as the matplotlib plot_trajectory.py output.

Layout (mirrors the matplotlib figure):
  ┌─────────────────────┬──────────────────┐
  │                     │  Altitude        │
  │   3D Trajectory     ├──────────────────┤
  │                     │  Vertical Vel    │
  ├──────────┬──────────┼──────────────────┤
  │ Horiz    │ Horiz    │ Vert Thrust      │
  │ Vel      │ Thrust   │ & Fuel           │
  └──────────┴──────────┴──────────────────┘

Usage:
    pip install rerun-sdk          # system python3, not the plotting venv
    python3 tools/rerun_blueprint.py [--rrd src/results/trajectory.rrd]

The script opens the .rrd in the Rerun viewer with the blueprint already applied.
To save a release .rrd with the blueprint baked in, pass --save.
"""

import argparse
import rerun as rr
import rerun.blueprint as rrb


def build_blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                # 3D trajectory — 2 column-shares wide
                rrb.Spatial3DView(
                    name="3D Trajectory",
                    origin="/",
                    contents=["rocket/**", "world"],
                ),
                # Right column: altitude + vertical velocity stacked
                rrb.Vertical(
                    rrb.TimeSeriesView(
                        name="Altitude",
                        contents=["/telemetry/altitude"],
                    ),
                    rrb.TimeSeriesView(
                        name="Vertical Velocity",
                        contents=["/telemetry/vel_z", "/telemetry/ref_vel_cmd"],
                    ),
                ),
                column_shares=[2, 1],
            ),
            # Bottom row: 3 equal panels
            rrb.Horizontal(
                rrb.TimeSeriesView(
                    name="Horizontal Velocities",
                    contents=["/telemetry/vel_x", "/telemetry/vel_y"],
                ),
                rrb.TimeSeriesView(
                    name="Horizontal Thrust (Fx, Fy)",
                    contents=["/telemetry/Fx", "/telemetry/Fy"],
                ),
                rrb.TimeSeriesView(
                    name="Vertical Thrust & Fuel",
                    contents=["/telemetry/Fz", "/telemetry/fuel"],
                ),
            ),
            row_shares=[2, 1],
        ),
        collapse_panels=True,
    )


def main():
    parser = argparse.ArgumentParser(description="Apply Rerun blueprint to Icarus .rrd")
    parser.add_argument("--rrd", default="src/results/trajectory.rrd",
                        help="Path to the .rrd recording")
    parser.add_argument("--save", metavar="OUTPUT",
                        help="Save a new .rrd with the blueprint baked in instead of opening viewer")
    args = parser.parse_args()

    blueprint = build_blueprint()

    if args.save:
        rr.init("icarus", recording_id="blueprint_release")
        rr.save(args.save, default_blueprint=blueprint)
        # Load the existing recording into the saved file
        rr.log_file_from_path(args.rrd)
        print(f"Saved: {args.save}")
    else:
        rr.init("icarus")
        rr.spawn(default_blueprint=blueprint)
        rr.log_file_from_path(args.rrd)
        print(f"Opened in Rerun viewer: {args.rrd}")


if __name__ == "__main__":
    main()
