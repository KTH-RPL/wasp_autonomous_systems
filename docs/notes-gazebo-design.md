# Gazebo setup design notes

Internal notes on how `gazebo/` is put together and why, kept here instead of in
`gazebo/pixi.toml` so the manifest stays readable. Student-facing instructions are in
[gazebo-alternative.md](gazebo-alternative.md); this file is for whoever maintains the
setup.

## Why it exists

Some employer-managed machines only allow software from the App Store or a company
software centre, and Webots is not distributed that way. On macOS this blocks those
students completely rather than only at runtime: `src/webots_ros2_driver/CMakeLists.txt`
links against an installed `Webots.app` at build time, so even `pixi install` of the
normal setup fails. Nothing under `gazebo/` refers to Webots.

## Why a separate workspace, not a second environment

Two Pixi environments cannot share a task name. With `ass_1_1` defined in both, `pixi run
ass_1_1` is ambiguous and Pixi stops to ask which environment you meant - which breaks the
plain commands the Webots setup has always used, and would force every task here to be
renamed (`gazebo_ass_3` and so on), including the many that involve no simulator at all.

A separate manifest gives every task its normal name back. Students `cd gazebo` once and
then follow the assignment text verbatim.

## Why a folder, not a branch

Considered and rejected, 2026-09-20. A branch would spare students the `cd`, but:

- The Gazebo manifest would become the root `pixi.toml`, so its lock would be the root
  `pixi.lock` - the same path as `ht26`'s with different content. Every merge forward
  would conflict in a 1.4 MB generated file. As a folder the two locks are different
  paths and never collide.
- The assignment code under `src/` would exist on two branches, so every fix to it would
  need merging across. The failure mode when that is forgotten is silent: Gazebo students
  quietly running stale exercise code, which hits the students already having the worst
  time.
- Students must edit the exercise files, so they always have local modifications to
  tracked files. Branch switching and pulling with those in hand puts merge conflicts in
  their own solutions.

The cost of the folder is that a task run from the repository root silently starts the
Webots version instead of erroring, since the names are identical. That is documented in
the student-facing page.

## Shared code and paths

The assignment packages are shared with the parent workspace, referenced as
`../src/<pkg>/package.xml`. The `package.xml` suffix is required: a bare directory path is
rejected for ROS source packages outside the manifest's own tree ("does not contain a
supported manifest").

Only `wasp_as_gazebo` lives in `gazebo/src/`. Keeping it out of `../src/` matters - the
normal `pixi run build` crawls `src/` and would otherwise build it into the Webots
workspace.

Rosbags download to the repository root (`tar -C ..`) and are read back as `../rosbags`,
so both setups share one copy rather than a few GB each.

## Dependency notes

- `ros-jazzy-wasp-as-interfaces` and `ros-jazzy-wasp-autonomous-systems-interfaces` are
  the only assignment packages built as conda packages here. Their generated typesupport
  libraries are `dlopen`'d by name at runtime, and on macOS only `$CONDA_PREFIX/lib` is
  searched - a colcon-built copy under `install/` is never found, and every node touching
  a `Collision` message dies with `Could not load library
  libwasp_as_interfaces__rosidl_typesupport_fastrtps_c.dylib`. Neither package depends on
  anything Webots-related, so both build without Webots installed.
- The `ultralytics`/`torch`/`transformers`/... block is the non-ROS run-dependencies of
  the assignment packages. In the parent workspace those arrive via `pixi-build-ros` from
  each `package.xml`; here the packages are colcon-built, so the dependencies are listed
  directly.
- `ros-jazzy-turtlebot3-gazebo` supplies the house and the robot meshes used by Task 1.1,
  which is why no large model assets are vendored into this repo.
- `ros-jazzy-compressed-image-transport` is not pulled in by `ros-jazzy-desktop` on this
  channel; without it RViz's Image display cannot load the `compressed` transport plugin.
  Same note applies to the root `pixi.toml`.
- `xacro` is *not* in this environment, which is why the Task 1.1 robot description is a
  plain `.urdf` rather than a `.xacro`.

## The build task

The packages to build are listed explicitly via `--paths` rather than crawled, so that
`../src/webots_ros2_*` and `../src/wasp_as_webots` are left alone - on macOS
`webots_ros2_driver` does not compile without an installed `Webots.app`, which is the
whole reason this setup exists. (`--packages-ignore` would read better but comes from the
`colcon-package-selection` extension, which is not installed here.)

Build and install output land in `gazebo/build` and `gazebo/install`, separate from the
parent workspace's, so the two never hand each other libraries built against the wrong
conda prefix.

## Task 1.1

It is the one task that may not work everywhere: it needs an RGB-D camera, and Gazebo's
rendering-backed sensors have a history of trouble on macOS. It is offered on every
platform anyway and prints a warning at startup on macOS - the whole point of this setup
is to unblock people who cannot run Webots, so refusing to even try would defeat it. See
`gazebo/src/wasp_as_gazebo/launch/ass_1_1_launch.py`.
