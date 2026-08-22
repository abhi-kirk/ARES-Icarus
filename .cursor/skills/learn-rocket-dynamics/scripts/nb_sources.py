#!/usr/bin/env python3
"""Print notebook cell sources without outputs. Stdlib only.

Usage:
  python nb_sources.py path/to/notebook.ipynb              # outline
  python nb_sources.py path/to/notebook.ipynb --cells 3,8-10
  python nb_sources.py path/to/notebook.ipynb --grep gimbal
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path


def cell_source(cell: dict) -> str:
    src = cell.get("source", "")
    if isinstance(src, list):
        return "".join(src)
    return src if isinstance(src, str) else ""


def first_line(text: str) -> str:
    for line in text.splitlines():
        stripped = line.strip()
        if stripped:
            return stripped[:100]
    return "(empty)"


def parse_cells(spec: str) -> list[int]:
    indices: list[int] = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            start_s, end_s = part.split("-", 1)
            start, end = int(start_s), int(end_s)
            if end < start:
                raise ValueError(f"bad range: {part}")
            indices.extend(range(start, end + 1))
        else:
            indices.append(int(part))
    return indices


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("notebook", type=Path)
    parser.add_argument(
        "--cells",
        help="Comma-separated indices and ranges, e.g. 0,3,8-10",
    )
    parser.add_argument(
        "--grep",
        help="Print outline rows whose source matches this regex (case-insensitive)",
    )
    args = parser.parse_args()

    path = args.notebook
    if not path.exists():
        print(f"not found: {path}", file=sys.stderr)
        return 1

    nb = json.loads(path.read_text())
    cells = nb.get("cells", [])

    if args.cells:
        for i in parse_cells(args.cells):
            if i < 0 or i >= len(cells):
                print(f"# cell {i}: out of range (0..{len(cells) - 1})", file=sys.stderr)
                continue
            cell = cells[i]
            kind = cell.get("cell_type", "?")
            print(f"# --- cell {i} ({kind}) ---")
            print(cell_source(cell).rstrip())
            print()
        return 0

    pattern = re.compile(args.grep, re.I) if args.grep else None
    print(f"# {path}  ({len(cells)} cells)")
    print(f"# {'idx':>4}  {'type':<8}  {'chars':>6}  first line")
    for i, cell in enumerate(cells):
        src = cell_source(cell)
        if pattern and not pattern.search(src):
            continue
        kind = cell.get("cell_type", "?")
        print(f"{i:4d}  {kind:<8}  {len(src):6d}  {first_line(src)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
