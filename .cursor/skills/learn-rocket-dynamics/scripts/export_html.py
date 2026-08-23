#!/usr/bin/env python3
"""Export learn-rocket-dynamics notebooks to gitignored dark HTML (no code).

Usage:
  python3 export_html.py                         # all notebooks in the folder
  python3 export_html.py learn-rocket-dynamics/02_rotations.ipynb
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[4]
NOTEBOOK_DIR = REPO / "learn-rocket-dynamics"
HTML_DIR = NOTEBOOK_DIR / "html"
VENV_JUPYTER = REPO / ".venv" / "bin" / "jupyter"


def jupyter() -> str:
    if VENV_JUPYTER.exists():
        return str(VENV_JUPYTER)
    return "jupyter"


def notebooks_from_args(args: list[str]) -> list[Path]:
    if not args:
        return sorted(
            p
            for p in NOTEBOOK_DIR.glob("*.ipynb")
            if ".ipynb_checkpoints" not in p.parts
        )
    out: list[Path] = []
    for raw in args:
        path = Path(raw)
        if not path.is_absolute():
            path = (Path.cwd() / path).resolve()
        if path.suffix != ".ipynb":
            print(f"skip (not a notebook): {path}", file=sys.stderr)
            continue
        if not path.exists():
            print(f"not found: {path}", file=sys.stderr)
            sys.exit(1)
        out.append(path)
    return out


def export(nb: Path) -> None:
    HTML_DIR.mkdir(parents=True, exist_ok=True)
    cmd = [
        jupyter(),
        "nbconvert",
        "--to",
        "html",
        "--no-input",
        "--template",
        "lab",
        "--theme",
        "dark",
        "--output-dir",
        str(HTML_DIR),
        str(nb),
    ]
    print(" ".join(cmd))
    subprocess.run(cmd, check=True, cwd=REPO)


def main() -> int:
    nbs = notebooks_from_args(sys.argv[1:])
    if not nbs:
        print("no notebooks to export", file=sys.stderr)
        return 1
    for nb in nbs:
        export(nb)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
