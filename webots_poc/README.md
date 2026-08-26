# Webots-on-macOS proof of concept (exploratory, not decided)

This branch preserves an exploratory investigation into running **Webots
natively on macOS** (via Pixi/RoboStack, no VM) as a candidate simulator for
the course, done as an alternative to `ht26-alpha`'s Gazebo-based port.

**This is not a decided direction.** The only thing actually committed to
for HT26 is using Pixi. Both Gazebo and native Webots have real, unresolved
problems on macOS; this branch exists so the Webots side of that comparison
isn't lost, not because a decision has been made.

## Why this exists

Gazebo segfaults on macOS for any robot with a camera or GPU-lidar sensor —
a `SIGSEGV` in `Ogre2RenderEngine::InitImpl()` ("`NSWindow should only be
instantiated on the main thread!`"), because Gazebo's Sensors system spins
up a render thread off the main thread, which macOS forbids outright. This
is a genuine, unfixable-from-the-repo engine bug (see `ht26-alpha`'s
`README.md`, "Known issues on macOS"). It blocks:

- **Assignment 1** (RGB-D camera) completely — the assignment's whole point
  is comparing simulated RGB-D data to real recorded data, so a
  camera-less workaround defeats it.
- **Assignment 2** (collision detection) in its default launch config
  (spawns a camera-equipped robot), though the actual exercise only needs
  `/imu` — a camera-and-lidar-less Turtlebot variant would also fix this
  *without* Webots, and is a cheaper fix if Gazebo is kept (not yet
  implemented in `ht26-alpha`).

Webots' own camera/depth rendering doesn't go through that crashing code
path at all. This branch tests whether that holds up in practice.

## Status: Assignments 1, 2, and 4 all confirmed working AND soak-tested

All three were validated with two kinds of test, both passing 8/8:

1. **Full restart-cycle soak test** (GUI rendering on): repeatedly launch
   the whole stack from cold, verify sensors/actuators come up correctly,
   fully tear down, relaunch. Tests that Webots itself starts up reliably.
2. **Restart-just-the-code test** (the one that actually matters for
   students): kill and restart only the student-facing code, while Webots
   and its driver process are left running and never touched. Confirmed
   Webots never needs restarting just because a student's script crashed
   or was edited and rerun — verified via identical process IDs
   before/after every cycle.

Assignment 3 (path planning) has no simulator dependency either way and
isn't part of this branch.

## Layout

- `ws/src/` — a colcon workspace. `webots_ros2_driver`,`webots_ros2_msgs`,
  `webots_ros2_importer`, `webots_ros2_control`, `webots_ros2_tests` are
  `cyberbotics/webots_ros2` @ tag `2025.0.1`, with macOS build/rpath/dyld
  patches applied to `webots_ros2_driver` (`if(APPLE)`-guarded, Linux path
  untouched) plus two new files: `Ros2Motor.cpp/.hpp` (unused by Mavic, see
  below) and `Ros2MavicController.cpp/.hpp` (see Assignment 4 below).
  - `webots_ros2_mavic/` — reused as the catch-all package for world/
    resource/launch files across all three assignments (nothing
    Mavic-specific about most of it despite the name):
    - `worlds/course_mavic_world.wbt` + `resource/{Mavic2Pro.proto,
      course_mavic_webots.urdf}` — the real course Mavic world (Assignment 4)
    - `worlds/turtlebot_collision_detection.wbt` +
      `resource/{TurtleBot3Burger.proto, turtlebot_webots.urdf}` — the real
      course Turtlebot collision-detection world (Assignment 2)
    - `worlds/turtlebot_apartment.wbt` + `resource/{Kinect.proto,
      turtlebot_webots_rgbd.urdf}` — the real course apartment world with an
      RGB-D Kinect-equipped Turtlebot (Assignment 1)
    - `launch/turtlebot_launch.py`, `launch/course_world_launch.py` — launch
      files for the above
  - `wasp_autonomous_systems_interfaces/` — copied from the real course repo
    (`Collision.msg`, `Encoders.msg`, etc.) so the packages below build
    against the real message types.
  - `assignment_1_solution/` — `encoders.py` (ported verbatim from the real
    course repo) + `launch/apartment_rgbd_launch.py`.
  - `assignment_2_solution/` — a filled-in `collision_detection.py`
    (threshold on IMU deviation-from-gravity magnitude) + `cleaning_robot.py`
    (a Roomba-style bump-and-turn demo built on top of it) +
    `launch/cleaning_robot_launch.py`.
- `assignment_4_skeleton/{altitude_manual,altitude_hold,altitude_pid}.py` —
  the real student skeleton files with two safety fixes added: a working
  Ctrl+C handler (`rclpy.init()`'s built-in SIGINT handler invalidates the
  context before a plain `except KeyboardInterrupt:` block can publish —
  fixed with a custom `signal.signal()` handler registered before the spin
  loop) and a braking landing (`land()` estimates vertical velocity from
  consecutive GPS readings and brakes before cutting to zero thrust, instead
  of free-falling into a hard landing that could flip the drone). **Not yet
  wired into `ws/src/` or applied to the real course repo's assignment_4
  package** — still standalone files.
- `testing/` — the soak-test and reliability-test scripts used to validate
  all of the above (restart-cycle scripts, sensor-rate checks, an
  Assignment-3-style repeated-run harness). Ephemeral/ad hoc, not polished
  tooling, kept for reproducibility.

## Fixed: `Ros2Supervisor` crash-loop on macOS

This used to crash-loop under `ros2 launch` (worked fine standalone via
`ros2 run webots_ros2_driver ros2_supervisor.py`), which is why every
launch file in this branch used `ros2_supervisor=False` /
`use_sim_time: False`. Root-caused and fixed — two independent bugs,
confirmed via live instrumentation and a 5-cycle restart soak test
(0 crashes, `/clock` publishing every cycle):

1. **The actual crash.** `ros2_supervisor.py` is installed as a raw script
   with a `#!/usr/bin/env python3` shebang. `launch_ros`'s `Node` action
   execs that script path directly, which routes the exec through macOS's
   SIP-protected `/usr/bin/env` — dyld strips `DYLD_*`/`LD_*` environment
   variables when loading a SIP-protected binary, so the eventual `python3`
   process never sees `DYLD_LIBRARY_PATH`, regardless of what
   `additional_env`/the parent `ros2 launch` process had (confirmed: `ros2
   launch`'s own process has it set correctly; `/usr/bin/env`'s child does
   not). The compiled `driver` binary and Webots' own `webots-controller`
   binary aren't scripts, so they're exec'd directly with no shebang hop —
   that's why only the supervisor ever hit this. **Fix:** `prefix=sys.
   executable` on `darwin` in `Ros2SupervisorLauncher`'s `Node(...)` call
   (`webots_launcher.py`) — forces the exec through our already-correct
   `python3` directly, bypassing the shebang/`/usr/bin/env` hop entirely.
2. **A separate, silent no-op.** `WebotsLauncher` always makes a temp copy
   of the world file and (when `ros2_supervisor=True`) appends a `Robot {
   name "Ros2Supervisor" controller "<extern>" supervisor TRUE }` node to
   that copy — but only actually *loads* the temp copy if the `world=`
   argument is a bare `LaunchConfiguration` reference; a literal string or
   a `PathJoinSubstitution` (what every launch file in this repo used)
   silently bypasses the indirection and loads the original world file with
   no supervisor robot in it. No error, no warning — the supervisor just
   retries forever ("not in the list of robots with `<extern>` controllers")
   and `/clock` never publishes. **Fix:** in `course_world_launch.py`,
   route the filename through `world = LaunchConfiguration('world',
   default='course_mavic_world.wbt')` first (matching `ht25`'s own working
   launch file pattern, which does exactly this), not a literal path.
   **Any other launch file in this branch that wants `ros2_supervisor=True`
   needs the same treatment** — `turtlebot_launch.py`/
   `apartment_rgbd_launch.py` still use a literal `PathJoinSubstitution`
   and `ros2_supervisor=False`, untouched by this fix (never needed it —
   see "What this does and doesn't get you" below).

**What this gets you:** with both fixed, `Ros2Supervisor` runs stably and
`/clock` publishes real simulation time, so `use_sim_time: True` is now safe
(`course_world_launch.py` sets it) — a genuine correctness improvement over
the previous wall-clock-timestamp workaround. `ros2_supervisor.py`'s ROS
node itself only exposes `spawn_urdf_robot`/`spawn_node_from_string`/
`animation_start_recording`/`animation_stop_recording` services — no
reset/reload ROS service exists, even now that the node runs cleanly.

**But in-place reset-on-flip doesn't need a ROS service at all — tested and
confirmed working via Webots' own native GUI:** `File > Reset Simulation`
(**⇧⌘T** / Shift+Cmd+T — see `docs/images/webots-file-menu-reset-shortcuts.png`
for the actual File menu, confirmed from a screenshot; `File > Reload World`
is **⇧⌘R**, a slower full disk reload, not needed for this). Tested both by
clicking the menu item and by sending the ⇧⌘T keystroke directly (as a
student actually would) — both via macOS UI scripting (`osascript`/System
Events — the Accessibility permission needs to be granted to whichever app
owns the actual terminal process tree, e.g. iTerm2, not a nested child
process), both give the identical result. Verified with an actual
displacement test, not just a clock check: flew the drone to `z≈28-393m`
via `/thrust` (tried multiple altitudes across the menu-click and keystroke
tests), triggered reset each way, GPS snapped back to the exact spawn pose
(`x≈0.005, y≈0, z≈0.086`) every single time regardless of how far away it
was. Also confirmed, with `ros2_supervisor=True` running: Webots' own
process PID never changed (a genuine in-place reset, not a disguised
restart); sim time actually reset (GPS header stamp e.g. `sec: 124` →
`sec: 8`); the driver auto-reconnected within under a second
(`respawn=True`); `Ros2Supervisor` itself briefly died and auto-respawned
too, handled automatically; zero orphaned processes after.
This is core Webots functionality independent of ROS/the supervisor fix
above — it would very likely have worked even before today's fixes, it just
hadn't been tested. **This is now the recommended flip-recovery path** —
faster than the `pkill`+relaunch fallback, doesn't interrupt the `ros2
launch` process tree at all, and is a single keyboard shortcut a student can
just be told about directly. Test scripts:
`testing/supervisor_soak_cycles.sh` + the `osascript` snippet above
(not scripted into a reusable file yet).

## Packaging: `pixi-build-ros` can build this from source automatically

Investigated whether `ht26-alpha`'s existing `pixi-build-ros` mechanism
(already used for `src/wasp_as_ass_1..4` etc. via `path = "..."` dependencies
in the root `pixi.toml`) could build this patched `webots_ros2_driver` the
same way — eliminating the need to manually `export LIBRARY_PATH=...`/
`export DYLD_LIBRARY_PATH=...` before every build/run, which is how this
whole investigation was actually tested (see `wasp-as-webots-macos-
investigation` memory / "If picking this up" below). **Result: yes, with
three fixes, all applied in this branch's `ws/src/`:**

1. **Each locally-built package needs its own tiny companion `pixi.toml`**
   declaring the build backend (already present alongside `package.xml` in
   `webots_ros2_driver/`, `webots_ros2_msgs/`, `webots_ros2_importer/` in
   this branch):
   ```toml
   [package.build.backend]
   channels = [
     "https://prefix.dev/pixi-build-backends",
     "https://prefix.dev/conda-forge",
   ]
   name = "pixi-build-ros"
   version = "*"
   ```
   Without this, `pixi install` fails with "does not contain a supported
   manifest" even though `package.xml` is right there — a `package.xml`
   alone isn't sufficient, despite what the error's own hint text implies.
   The referenced source directories also need to be `git add`ed (tracked,
   not necessarily committed) — `pixi-build-ros`'s source resolution
   silently ignores untracked files.
2. **A genuine upstream bug, unrelated to any macOS patch**, blocks
   compilation: `Ros2LightSensor.cpp` has a C++ chained comparison (`X < Y <
   Z`, which doesn't mean what it looks like) that plain `colcon build` only
   warns about, but `pixi-build-ros`'s stricter build flags treat as a hard
   error. Fixed (rewritten as `X < Y && Y < Z`) — this file isn't even used
   by any of the three assignments here, so it was never hit before.
3. **`DYLD_LIBRARY_PATH` still can't be eliminated at the linking level.**
   The driver embeds `@rpath/Contents/lib/controller/libController.dylib`
   (baked in by Cyberbotics' own Webots.app build), and `CMakeLists.txt`
   already tries to add `/Applications/Webots.app` as a matching absolute
   rpath entry — but conda/pixi-build's binary-relocation step strips any
   rpath pointing outside the build prefix, so that never survives into the
   installed binary (confirmed via `otool -l`: only `@loader_path/..` made
   it through). Practical fix: automate the *export* instead of trying to
   eliminate it, via the root `pixi.toml`:
   ```toml
   [target.unix.activation.env]
   DYLD_LIBRARY_PATH = "/Applications/Webots.app/Contents/lib/controller"
   ```
   **This works for `pixi shell`/`pixi shell-hook`, but NOT for `pixi run
   <task>`** — confirmed via direct testing (`pixi run bash -c 'echo
   $DYLD_LIBRARY_PATH'` comes back empty even with `--force-activate`; this
   looks like a genuine `pixi` limitation, not something specific to this
   package). Since `ht26-alpha`'s actual usage pattern is `pixi run
   ass_4_manual`-style tasks, that gap matters. Workaround, confirmed
   working: declare the env var directly on each relevant task instead:
   ```toml
   ass_4_manual = { cmd = "ros2 launch ...", env = { DYLD_LIBRARY_PATH = "/Applications/Webots.app/Contents/lib/controller" } }
   ```
   Not yet done: adding this to real task definitions (this was only proven
   in an isolated throwaway clone, not wired into any launch file/task
   here). **This is Mac-only** — `LD_LIBRARY_PATH`/Linux doesn't need it;
   the non-Apple `CMakeLists.txt` branch compiles its own self-contained
   controller library from the bundled `webots/` source instead of linking
   against an external Webots.app install, so it's a different code path
   entirely that shouldn't hit this.

**Not needed at all, contrary to earlier assumption:** `LIBRARY_PATH`
(the `-latomic`/`yaml-cpp` linker workaround documented below and in
`ht26-alpha`'s own README) — that was only ever a plain-`colcon-build`
problem. `pixi-build-ros`'s isolated build already injects correct
LDFLAGS/rpath, confirmed by the build succeeding without setting it.

**Bottom line:** a fresh `pixi install` on macOS, with these three fixes and
the `webots_ros2_driver`/`webots_ros2_msgs`/`webots_ros2_importer` family
added as `path = "..."` dependencies (same pattern as the course's own
`wasp_as_ass_*` packages), builds the patched driver end-to-end with zero
manual intervention beyond the one-time `[target.unix.activation.env]`
addition (for shell users) or per-task `env` (for `pixi run` users).
Verified by launching the built `driver` binary directly and confirming no
`dyld` errors, in a clean shell with no prior manual exports. Not yet
verified: an actual end-to-end Webots launch (world + URDF + full
`ros2 launch`) using *this* pixi-built package rather than the
plain-`colcon-build` scratch workspace everything else in this branch was
tested against — the mechanism is proven, but hasn't replaced the
day-to-day testing setup.

## Known unresolved issues

- Embedded Python plugins inside the C++ driver (`PythonPlugin.cpp`) fail
  silently on this build (a swallowed exception, `PyErr_Print()` missing on
  that code path) — root cause not found. Not a blocker: worked around
  entirely by using a C++ plugin (`Ros2MavicController`) instead of the
  upstream `MavicDriver` Python plugin for Assignment 4.
- None of this has been soak-tested for anything beyond restart-cycle
  reliability — no multi-hour sessions, no adversarial student-style rapid
  parameter tuning via `rqt`/Foxglove.

## If picking this up

- Re-clone `github.com/cyberbotics/webots_ros2` @ tag `2025.0.1` and diff
  against `ws/src/webots_ros2_driver` if you need to re-verify exactly what
  changed, or just trust this checked-in copy.
- The macOS-only build gotchas that aren't fixed by this branch's patches:
  plain `colcon build` needs `LIBRARY_PATH=<pixi-env>/lib` exported (`ld:
  library 'yaml-cpp'/'atomic' not found` otherwise) and the built driver
  needs `DYLD_LIBRARY_PATH` including `<pixi-env>/lib` at runtime
  (conda-forge/RoboStack binaries use `@loader_path`-relative rpaths that
  only resolve inside the conda prefix).
- Use `colcon build --symlink-install` — a plain `colcon build` copies
  files, so editing a `.py` file and re-testing without rebuilding will
  silently run the stale version.
