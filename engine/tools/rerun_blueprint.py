#!/usr/bin/env python3
"""
Launch the Rerun viewer with the Icarus layout blueprint and trajectory recording.

Usage:
    python3 tools/rerun_blueprint.py
    python3 tools/rerun_blueprint.py --rrd src/results/trajectory.rrd

This saves the blueprint to docs/rerun_layout.rbl and opens it alongside the
recording via the Rerun CLI. You can also do this manually:
    rerun docs/rerun_layout.rbl src/results/trajectory.rrd

Layout:
  ┌─────────────────────┬──────────────────┐
  │                     │  Altitude        │
  │   3D Trajectory     ├──────────────────┤
  │                     │  Vertical Vel    │
  ├──────────┬──────────┼──────────────────┤
  │ Horiz    │ Horiz    │ Vert Thrust      │
  │ Vel      │ Thrust   │ & Fuel           │
  └──────────┴──────────┴──────────────────┘
"""

import argparse
import subprocess

import rerun.blueprint as rrb


def build_blueprint() -> rrb.Blueprint:
    return rrb.Blueprint(
        rrb.Vertical(
            rrb.Horizontal(
                rrb.Spatial3DView(
                    name="3D Trajectory",
                    origin="/",
                    contents=["rocket/**", "world", "world/pad"],
                ),
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
                    name="Vertical Thrust (Fz)",
                    contents=["/telemetry/Fz"],
                ),
                rrb.TimeSeriesView(
                    name="Fuel",
                    contents=["/telemetry/fuel"],
                ),
            ),
            row_shares=[2, 1],
        ),
        collapse_panels=False,
    )


def main():
    parser = argparse.ArgumentParser(description="Launch Rerun viewer for Icarus")
    parser.add_argument("--rrd", default="src/results/trajectory.rrd",
                        help="Path to .rrd recording to load")
    parser.add_argument("--save-only", action="store_true",
                        help="Save .rbl file without launching viewer")
    parser.add_argument("--publish", action="store_true",
                        help="Embed blueprint into a self-contained .rrd for web publishing")
    parser.add_argument("--out", default="src/results/trajectory_published.rrd",
                        help="Output path for --publish (default: src/results/trajectory_published.rrd)")
    args = parser.parse_args()

    blueprint = build_blueprint()
    rbl_path = "docs/rerun_layout.rbl"

    # Save the blueprint to a .rbl file
    blueprint.save("icarus", path=rbl_path)
    print(f"Blueprint saved to {rbl_path}")

    if args.publish:
        # Embed the blueprint directly into the rrd so the web viewer picks it up
        import rerun as rr
        rr.init("icarus")
        rr.save(args.out)
        rr.send_blueprint(blueprint)
        rr.log_file_from_path(args.rrd)
        print(f"Published rrd with embedded blueprint: {args.out}")
        return

    if not args.save_only:
        # Launch the viewer with both the blueprint and the recording
        print(f"Opening: rerun {rbl_path} {args.rrd}")
        subprocess.run(["/opt/homebrew/bin/rerun", rbl_path, args.rrd])


if __name__ == "__main__":
    main()
