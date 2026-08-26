# WASP Autonomous Systems Course

Please see the [Wiki](https://github.com/KTH-RPL/wasp_autonomous_systems/wiki).

## Known issues on macOS

- Assignments 1, 2, and 4 use Webots as their simulator, not Gazebo. This
  was a deliberate switch: Gazebo (`gz-sim`) crashes with a segfault on
  macOS whenever the simulated robot has a camera/RGBD sensor (e.g.
  `turtlebot3_waffle_rgbd`, used by assignments 1 and 2), even fully
  headless — a `SIGSEGV` in `Ogre2RenderEngine::InitImpl()`, because
  gz-sim's sensors system initializes Ogre2 rendering off the main thread,
  which macOS/Cocoa does not allow. Unresolved upstream, not fixable from
  this repo — see [gazebosim/gz-sim#960](https://github.com/gazebosim/gz-sim/issues/960)
  and [gazebosim/gz-sim#2877](https://github.com/gazebosim/gz-sim/issues/2877).
  Webots' own rendering doesn't go through that code path. **Only verified
  on macOS**: the patched Webots ROS2 driver's build/rpath/dyld fixes are
  `if(APPLE)`-guarded, so Linux gets the unmodified upstream driver, but
  the `pixi-build-ros` packaging of this driver hasn't actually been
  exercised on Linux/Windows.
- `rcutils` unconditionally links `-latomic` in its exported CMake config,
  which doesn't exist on macOS (atomics are compiler builtins there). This
  breaks `pixi run build` for any package that transitively depends on
  `rcutils` (e.g. `wasp_as_interfaces`) with `ld: library 'atomic' not found`.
  Workaround used during setup: create a stub `libatomic.dylib` in the pixi
  environment's `lib/` and export `LIBRARY_PATH` to include it before
  building. This is not yet automated in `pixi.toml`.
