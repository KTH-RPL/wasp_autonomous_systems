# Native Windows build investigation notes

Internal notes on why `pixi run build` doesn't work natively on `win-64`, kept here
instead of in `installation-windows.md` so students following the install docs don't
need to wade through it. Not linked from anywhere else in the repo.

## Summary

Native Windows builds hit a genuine upstream bug in the `pixi-build-ros` backend, not
anything fixable from this repo's `pixi.toml`. Building inside WSL2 (`linux-64`) works
instead.

## What was tried, in order

1. **Missing MSVC compiler** - `pixi run build` failed at CMake configure with
   `Could not find the compiler specified in the environment variable CC: cl.exe`.
   Fixed by installing Visual Studio 2022 Build Tools with the C++ workload.
2. **Windows path length** - after the compiler fix, builds failed with
   `fatal error C1083: Cannot open compiler generated file: '': Invalid argument` on
   packages with long generated object paths (300+ chars, over the classic 260 limit).
   Enabling `LongPathsEnabled` in the registry got past some of these but not all -
   some generated paths for longer package names (`wasp_autonomous_systems_interfaces`)
   still exceeded what MSVC's PDB writer handles even with the registry flag set.
3. **The actual blocker** - even from a fresh clone at a short path (`C:\wsbuild`,
   ruling out path length entirely), every package that generates rosidl Python
   message bindings (`wasp_as_interfaces`, `wasp_autonomous_systems_interfaces`,
   `webots_ros2_msgs`) failed during install with:
   ```
   CMake Error at cmake_install.cmake:152 (execute_process):
     Syntax error in cmake code at
       .../build/cmake_install.cmake:154
     when parsing string
       C:\<...>\host\python.exe
     Invalid character escape '\w'.
   ```
   `pixi-build-ros` embeds the Windows Python executable path using backslashes
   directly into a generated `execute_process(...)` call in `cmake_install.cmake`
   (used to byte-compile the rosidl Python bindings). CMake's own script parser
   treats backslashes as escape characters, and `\w` (from `C:\wsbuild`, or whatever
   `\<letter>` sequence appears first in the path) isn't a valid one, so parsing the
   install script itself fails - every other path in the same generated file
   correctly uses forward slashes, so this looks like one specific variable
   substitution in `pixi-build-ros`'s Windows CMake templating that was never
   converted. This matches `pixi-build-ros`'s own README, which lists "Add support
   for Windows" as an open TODO.

   Issue drafted against [prefix-dev/pixi](https://github.com/prefix-dev/pixi/issues)
   (the successor to the now-archived `prefix-dev/pixi-build-backends`).

## What works instead: WSL2

Building and running the whole stack (pixi, ROS2, Webots) inside WSL2 got further,
with two more fixable issues along the way (both are just normal Linux/ROS2
prerequisites, not Windows/WSL2-specific bugs):

- Missing system C toolchain - Webots' vendored controller library Makefile calls
  `gcc`/`make` directly rather than the conda-provided compiler. Fixed with
  `sudo apt install build-essential` (see `installation-linux.md`).
- Linker couldn't find `liblttng-ust`/`libyaml-cpp` at build time, even though both
  exist in the pixi env - same class of issue already worked around for macOS in
  `pixi.toml` (see the comment above `[target.osx-arm64.tasks]`). Fixed the same way
  for `linux-64`: an explicit `LIBRARY_PATH` pointing at the conda env's lib dir in
  the `build` task.

With both of those in place, `pixi run build` completed with no failures, and running
`ass_1_1` + `ass_1_1_teleop` produced a Webots window (via WSLg) with a robot that
could actually be driven - checked once, on one machine, not across a matrix of
Windows/WSL versions.
