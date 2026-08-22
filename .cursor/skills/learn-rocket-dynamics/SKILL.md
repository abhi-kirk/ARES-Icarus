---
name: learn-rocket-dynamics
description: >-
  Edits and reviews Karpathy-style rocket dynamics Jupyter notebooks in
  learn-rocket-dynamics/. Use when working on .ipynb files, 01_pendulum,
  02_rotations, Euler's equations, quaternions, attitude kinematics, gimbal
  lock, or the notebook series that prepares Icarus Phase 4 C++.
---

# Learn rocket dynamics notebooks

Goal: Abhi understands rotational rigid-body dynamics deeply enough to implement 6-DOF attitude + gimbal TVC in C++. Not a Python tutorial. Derive, implement, plot, then write C++.

Full status and conventions: `context/notebooks.md`. C++ mapping: [cpp-mapping.md](cpp-mapping.md).

## Collaboration

Abhi writes the notebooks. Fix bugs, typos, and broken cells directly. Propose new sections or experiment prompts **before** inserting them.

## Token-cheap editing (mandatory)

`.ipynb` JSON with outputs is expensive. Never `Read` a full notebook.

1. Outline (no outputs):

```bash
python3 .cursor/skills/learn-rocket-dynamics/scripts/nb_sources.py learn-rocket-dynamics/02_rotations.ipynb
```

2. Dump only the cells you need:

```bash
python3 .cursor/skills/learn-rocket-dynamics/scripts/nb_sources.py learn-rocket-dynamics/02_rotations.ipynb --cells 3,8-10
python3 .cursor/skills/learn-rocket-dynamics/scripts/nb_sources.py learn-rocket-dynamics/02_rotations.ipynb --grep gimbal
```

3. Edit with `EditNotebook` (cell source only). Match `old_string` exactly. Batch independent cell edits. Do not rewrite the `.ipynb` file as JSON.

4. Shared code lives in `learn-rocket-dynamics/utils.py` and `plotting.py`. Prefer adding a helper over duplicating integrator or quaternion code in a cell.

5. Animations: GIFs in `learn-rocket-dynamics/animations/`, embedded in markdown (GitHub-renderable). Do not store large plot outputs in the notebook.

If Abhi points at a section by name, `--grep` then `--cells`; do not ingest the whole file.

## Pedagogy

- One idea per section: short markdown derivation, then a small code cell, then a plot that makes the claim visible.
- Named intermediates; readable over clever. Same bar as the C++ engine.
- Show the failure (Euler energy drift, gimbal lock) before the fix (RK4, quaternions).
- Integrator API: `f(t, state)` so it matches C++ `SystemDynamics`.
- Quaternions `[w, x, y, z]`; Euler ZYX. Reuse `utils.py`.
- Use `plotting.setup_style()` and existing palette colors.

## Review checklist

- Physics and numerics are correct; plots match the claim in the markdown.
- Conventions match [cpp-mapping.md](cpp-mapping.md) (frames, quaternion order, `f(t, state)`).
- No unexplained new helpers that belong in `utils.py`.
- Markdown cells teach; code cells are short enough to follow by hand.
