# Building Icarus

This repo builds a single C++17 executable (`Icarus`) using CMake. It depends on:

- A C++17-capable compiler (macOS: Apple Clang via Command Line Tools)
- CMake (>= 3.10)
- Eigen3 (headers-only; discovered via `find_package(Eigen3 REQUIRED)`)

## macOS (Apple Silicon / Intel)

### 1) Install toolchain

Install Xcode Command Line Tools (provides `clang++`, SDK headers, libc++):

```bash
xcode-select --install
```

Verify:

```bash
clang++ --version
xcrun --show-sdk-path
```

### 2) Install dependencies

Using Homebrew:

```bash
brew install cmake eigen
```

Notes:
- On Apple Silicon, Homebrew typically installs under `/opt/homebrew`.
- CMake’s `find_package(Eigen3 REQUIRED)` should succeed after `brew install eigen`.

### 3) Configure and build

From the repo root:

```bash
make build
```

This creates `build/` and produces:
- `build/Icarus` (the executable)
- `build/compile_commands.json` (used by IDE tooling)

### 4) Run

```bash
make run
```

Or:

```bash
./build/Icarus
```

## Linux (Ubuntu/Debian-style)

Install deps:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev
```

Build:

```bash
make build
```

## Clean rebuild

```bash
make rebuild
```

If you want a truly clean slate:

```bash
rm -rf build
make build
```

## VS Code / IntelliSense

This repo includes workspace settings in `.vscode/` that point cpptools at
`build/compile_commands.json`. After cloning onto a new machine:

1. Run `make build` once (generates `build/compile_commands.json`).
2. In VS Code run `CMake: Configure` (if using CMake Tools).
3. If you see `cannot open source file <...>` errors:
   - Run `C/C++: Reset IntelliSense Database`
   - Run `Developer: Reload Window`

### If cpptools still can’t find standard headers on macOS

cpptools sometimes needs explicit system include paths when a global/user VS Code
setting overrides them. The workspace config sets `C_Cpp.default.systemIncludePath`
to known-good macOS SDK locations.

If you upgrade Xcode/CommandLineTools and IntelliSense breaks again, update the
Clang builtin include path using:

```bash
clang++ -print-resource-dir
```

Then ensure `<resource-dir>/include` is present in `C_Cpp.default.systemIncludePath`
in `.vscode/settings.json`.

### Alternative: clangd (often more robust)

There is a `.clangd` file in the repo configured to use `build/` as the
compilation database. If you install the `clangd` VS Code extension, it will
use `build/compile_commands.json` directly.

