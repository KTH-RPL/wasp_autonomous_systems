# WASP Autonomous Systems Course

Please see the [Wiki](https://github.com/KTH-RPL/wasp_autonomous_systems/wiki).

## Known issues on macOS

- The Gazebo GUI (`-g` / combined server+GUI process) does not work on macOS
  at all — see [gazebosim/gz-sim#44](https://github.com/gazebosim/gz-sim/issues/44),
  open since 2019. All Gazebo launches in this repo run headless (`-s`) on
  every platform as a result; visualization is via RViz/rqt only, no native
  Gazebo 3D window.
- Beyond that, Gazebo (`gz-sim`) crashes with a segfault on macOS whenever
  the simulated robot has a camera/RGBD sensor (e.g. `turtlebot3_waffle_rgbd`,
  used in assignments 1 and 2), even fully headless. This is a separate,
  unresolved upstream limitation in how gz-sim's sensors system initializes
  Ogre2 rendering off the main thread, which macOS/Cocoa does not allow — see
  [gazebosim/gz-sim#960](https://github.com/gazebosim/gz-sim/issues/960) and
  [gazebosim/gz-sim#2877](https://github.com/gazebosim/gz-sim/issues/2877).
  Not fixable from this repo. Students on macOS should expect assignments 1
  and 2's Gazebo simulations to crash on spawn.
- Assignment 4 (quadrotor) has no camera sensor, so it is unaffected by the
  above and runs fine headless on macOS.
- `rcutils` unconditionally links `-latomic` in its exported CMake config,
  which doesn't exist on macOS (atomics are compiler builtins there). This
  breaks `pixi run build` for any package that transitively depends on
  `rcutils` (e.g. `wasp_as_interfaces`) with `ld: library 'atomic' not found`.
  Workaround used during setup: create a stub `libatomic.dylib` in the pixi
  environment's `lib/` and export `LIBRARY_PATH` to include it before
  building. This is not yet automated in `pixi.toml`.
