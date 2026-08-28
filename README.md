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
  which doesn't exist as a standalone library on macOS (atomics are
  compiler builtins there) - and separately, `webots_ros2_driver`'s C++
  plugins need `yaml-cpp`. Both break `pixi run build` with
  `ld: library 'X' not found` unless the linker is told where to find them
  in the pixi environment. **Already handled**: the `build` task exports
  `LIBRARY_PATH` for this on macOS (see `[target.osx-arm64.tasks]` in
  `pixi.toml`) - just run `pixi run build`, no manual steps needed.
