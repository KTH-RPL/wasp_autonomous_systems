# Verifying the Webots migration on Linux

Assignments 1, 2, and 4 were migrated from Gazebo to Webots (see commits
`a969193`, `259e068`, `09a593f` on `ht26-alpha`) and, so far, only tested on
macOS. All macOS-specific patches to the vendored `webots_ros2_driver` are
`if(APPLE)`-guarded, so Linux gets the unmodified upstream driver — the
underlying combination is well-established — but the `pixi-build-ros`
packaging of this patched driver has not actually been exercised on Linux.
This is what that verification pass is for.

## 1. Install Webots

Download the `.deb` (or tarball) from
[cyberbotics.com](https://cyberbotics.com/#download) and install it, e.g.:

```
sudo apt install ./webots_*.deb
```

As long as `webots` ends up on `PATH` (typical for the `.deb` installer),
you're set. `packaging/webots_env_activation.sh` in this repo is a no-op on
Linux (it's guarded to macOS only, where a separate fix was needed to find a
non-`PATH` Webots.app install) — Linux relies entirely on
`webots_ros2_driver`'s own upstream discovery logic. If `webots` isn't
findable on `PATH` for some reason, set `WEBOTS_HOME` to the install
directory manually.

## 2. Install pixi

```
curl -fsSL https://pixi.sh/install.sh | sh
```

## 3. Get the repo on `ht26-alpha`

```
git clone git@github.com:KTH-RPL/wasp_autonomous_systems.git
cd wasp_autonomous_systems
git checkout ht26-alpha
git pull
```

## 4. `pixi install` — the actual test

```
pixi install
```

This is the step that matters most: it exercises whether `pixi-build-ros`
can build the patched `webots_ros2_driver`, `webots_ros2_msgs`,
`webots_ros2_importer`, `webots_ros2_control`, and `wasp_as_webots`
packages on Linux. If it fails, the error output itself is the useful
signal to bring back for debugging.

## 5. Launch each assignment and eyeball it

```
pixi run ass_1_1        # Ctrl+C to stop
pixi run ass_2_2
pixi run ass_4_manual
```

Each should open Webots with the GUI, connect the robot/drone controller,
and start streaming sensor data:

- **`ass_1_1`** (Turtlebot + RGB-D apartment): camera/depth images should
  start flowing (`ros2 topic hz /camera/color/image_raw` in another
  terminal should show a nonzero rate).
- **`ass_2_2`** (Turtlebot collision detection): `/imu` should be
  publishing, and `diffdrive_controller`/`joint_state_broadcaster` should
  both log "Configured and activated" in the launch output within a few
  seconds.
- **`ass_4_manual`** (Mavic drone): `/mavic_2_pro/gps` and `/thrust` should
  both be live; setting the `thrust` ROS parameter above ~68.5 (the hover
  value) should make the drone visibly climb in the Webots GUI.

This mirrors exactly what was verified on macOS — same commands, same
things to check.

## Things that are macOS-specific and shouldn't come up on Linux

- The `-latomic` linker issue documented in the main `README.md`'s "Known
  issues" section is macOS-only (atomics are compiler builtins there, but
  not on glibc/Linux) — it shouldn't appear at all here.
- Every dependency fix made during the migration (the missing
  `diff_drive_controller`/`joint_state_broadcaster` deps,
  `webots_ros2_control`'s missing `pixi.toml`) was a plain package
  declaration, not macOS-specific code — those fixes apply identically on
  Linux since they're already pushed to `ht26-alpha`.

## If something breaks

Bring back the exact error output from whichever step failed (`pixi
install` output, or the `pixi run ass_*` launch log) — that's the fastest
path to a fix.
