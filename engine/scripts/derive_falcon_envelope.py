#!/usr/bin/env python3
"""Regenerate scripts/data/falcon_landing_envelope.json from a local corpus.

The corpus lives outside this repo (default: ~/Documents/flight-telemetry).
Committed numbers are the bench; re-run only when the catalog grows.

  python3 scripts/derive_falcon_envelope.py
  python3 scripts/derive_falcon_envelope.py --corpus /path/to/flight-telemetry
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import Counter
from pathlib import Path


def pct(xs: list[float], p: float) -> float:
    xs = sorted(xs)
    i = min(len(xs) - 1, max(0, int(round((p / 100.0) * (len(xs) - 1)))))
    return xs[i]


def summarize(vals: list[float]) -> dict | None:
    vals = [v for v in vals if v is not None and math.isfinite(v)]
    if not vals:
        return None
    return {
        "p10": round(pct(vals, 10), 2),
        "p25": round(pct(vals, 25), 2),
        "p50": round(pct(vals, 50), 2),
        "p75": round(pct(vals, 75), 2),
        "p90": round(pct(vals, 90), 2),
        "mean": round(sum(vals) / len(vals), 2),
    }


def fbool(x: str) -> bool:
    return str(x).lower() in ("true", "1", "yes")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--corpus",
        default=str(Path.home() / "Documents" / "flight-telemetry"),
    )
    ap.add_argument(
        "--out",
        default=str(
            Path(__file__).resolve().parent / "data" / "falcon_landing_envelope.json"
        ),
    )
    args = ap.parse_args()
    root = Path(args.corpus)
    cat = list(csv.DictReader((root / "catalog.csv").open()))
    rows = [
        r
        for r in cat
        if r.get("vehicle") == "falcon"
        and fbool(r.get("booster_touchdown_tracked"))
        and r.get("status") == "processed"
    ]

    landing_alts: list[float] = []
    pre_speeds: list[float] = []
    taus: list[float] = []
    durations: list[float] = []
    shapes: dict[str, list[float]] = {
        "0.00": [],
        "0.25": [],
        "0.50": [],
        "0.75": [],
        "1.00": [],
    }
    landing_types: Counter[str] = Counter()

    for r in rows:
        touch_raw = (r.get("booster_touchdown_t_s") or "").strip()
        if not touch_raw:
            continue
        touch_t = float(touch_raw)
        folder = root / r["folder"]
        ts_path = folder / "timeseries.csv"
        if not ts_path.exists():
            continue
        with ts_path.open() as f:
            series = list(csv.DictReader(f))

        samples = []
        for s in series:
            try:
                t = float(s["t_mission_s"])
            except (KeyError, ValueError):
                continue
            if t < touch_t - 400 or t > touch_t + 1:
                continue
            alt = (s.get("booster_alt_m") or "").strip()
            spd = (s.get("booster_speed_mps") or "").strip()
            eng = (s.get("booster_engines_lit") or "").strip()
            a = float(alt) if alt else None
            v = float(spd) if spd else None
            e = int(float(eng)) if eng else None
            samples.append((t, a, v, e))

        edges = []
        prev = 0
        for t, a, v, e in samples:
            if e is None:
                continue
            if prev == 0 and e >= 1 and a is not None and a > 20:
                edges.append((t, a))
            prev = e
        if not edges:
            continue

        burn_t, burn_a = edges[-1]
        last_v = None
        last_a = None
        for t, a, v, e in samples:
            if t < burn_t - 30 or t >= burn_t:
                continue
            if v is not None:
                last_v = v
            if a is not None:
                last_a = a
        if last_v is None or last_v <= 1.0:
            continue
        alt_pre = last_a if last_a is not None else burn_a
        landing_alts.append(burn_a)
        pre_speeds.append(last_v)
        taus.append(alt_pre / last_v)
        durations.append(touch_t - burn_t)
        landing_types[r.get("landing_type") or "unknown"] += 1

        pts = []
        for t, a, v, e in samples:
            if t < burn_t or t > touch_t or a is None or burn_a <= 0:
                continue
            frac = (t - burn_t) / (touch_t - burn_t) if touch_t > burn_t else 0.0
            pts.append((frac, a / burn_a))
        for key, target in [
            ("0.00", 0.0),
            ("0.25", 0.25),
            ("0.50", 0.5),
            ("0.75", 0.75),
            ("1.00", 1.0),
        ]:
            if not pts:
                continue
            best = min(pts, key=lambda p: abs(p[0] - target))
            shapes[key].append(best[1])

    h_norm = {
        k: round(pct(vs, 50), 3) if vs else None for k, vs in shapes.items()
    }
    nominal_z = 280.0
    falcon_ref = summarize(landing_alts)["p50"]
    tau_p50 = summarize(taus)["p50"]

    payload = {
        "version": 1,
        "source": {
            "corpus": str(root),
            "filter": "vehicle=falcon, booster_touchdown_tracked, status=processed, touchdown_t present",
            "n_landings": len(landing_alts),
            "landing_types": dict(landing_types),
            "method": (
                "Landing burn = last booster_engines_lit rising edge before touchdown "
                "(entry burn ~60 km excluded). Speed during the lit window is usually blank "
                "on the webcast engine diagram; pre-burn speed is the last valid "
                "booster_speed_mps in the 30 s before that edge. tau = alt / speed at that "
                "pre-burn sample."
            ),
        },
        "falcon": {
            "landing_burn_alt_m": summarize(landing_alts),
            "preburn_speed_mps": summarize(pre_speeds),
            "tau_s": summarize(taus),
            "burn_duration_s": summarize(durations),
            "h_norm_vs_burn_frac": h_norm,
            "missing": [
                "mass / thrust / throttle",
                "attitude / gimbal",
                "horizontal position and velocity",
            ],
        },
        "plant_scale": {
            "method": "Keep Falcon tau = h/|v|; map altitude into the Icarus hop height. Do not replay raw km.",
            "nominal_z_m": nominal_z,
            "falcon_alt_ref_m": falcon_ref,
            "nominal_vz_mps": round(-nominal_z / tau_p50, 2),
        },
        "academic_walls": {
            "glide_cone_deg": 20.0,
            "pointing_cone_deg": 25.0,
            "thrust_max_N": 18000.0,
            "thrust_min_N": 7200.0,
            "note": "Horizontal offset and wind have no Falcon webcast field; walls match the constrained MC envelope.",
        },
    }

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(payload, indent=2) + "\n")
    print(f"wrote {out}  n={len(landing_alts)}")


if __name__ == "__main__":
    main()
