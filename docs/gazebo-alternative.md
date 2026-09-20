# Running the course without Webots (Gazebo alternative)

Some employer-managed computers only allow software installed through the
App Store or a company software centre, and Webots is not distributed that
way. If you cannot install it, use this instead. Everything else the course
needs arrives through Pixi, which installs into the folder you cloned and
needs no administrator rights.

Only use this if Webots is genuinely not an option. If it works on your
machine, follow [the README](../README.md) as normal.

## How to use it

Install Pixi as described for your platform
([Mac](installation-mac.md), [Linux](installation-linux.md),
[Windows](installation-windows.md)), but **skip the Webots section**. Clone
the repo as the README describes. Then work from the `gazebo` folder inside
it:

```
cd wasp_autonomous_systems/gazebo
pixi run build
pixi run download_rosbags
```

**That is the whole difference.** From there, do everything exactly as the
assignment text says. The task names are the same, the commands are the
same, and you edit the same files. The only thing to remember is that your
terminal stays in the `gazebo` folder.

It has to be a separate folder because the Webots packages cannot even be
built on a machine without Webots installed, so the normal `pixi run build`
in the repository root would fail for you. This folder has its own packages
and its own build output, and shares the assignment code with the normal
setup.

### Already tried Webots and it did not work?

Nothing to delete, nothing to clone again. Just `cd gazebo` and
`pixi run build`. A failed attempt leaves several GB behind, though; if you
want the space back, delete these **from the repository root**:

```
rm -rf .pixi build install log
```

They are all generated and none of them are used here. Keep `rosbags` if you
already downloaded it - this setup reads the same copy.

## Things worth knowing

**Task 1.1 may not work on macOS.** It is the only task that needs a
simulated camera, and that is the part of Gazebo with a history of trouble
there. It starts anyway and prints a warning, and it has been seen working,
so try it. If the camera views in RViz stay empty or Gazebo closes by
itself, tell the course staff rather than assuming you did something wrong.
Everything else is unaffected.

**Every terminal needs to be in the `gazebo` folder.** A few tasks are meant
to be run alongside each other, and the assignment text will tell you to open
a second or third terminal. A new terminal starts in your home directory, so
`cd` into `wasp_autonomous_systems/gazebo` in each one, not into
`wasp_autonomous_systems`.

Getting this wrong does not give you a helpful error. The task names are the
same in both places, so running one from the repository root silently starts
the **Webots** version instead - the one you cannot run. Depending on your
machine that either spends a long time trying to build the Webots packages
and then fails, or starts and complains that it cannot find Webots. If a
command behaves nothing like the assignment describes, check which folder
you are in first.

**You get Gazebo's own 3D window**, on every platform, with nothing to pass.

You can turn it off, which is worth doing under WSL2: there it falls back to
software rendering and can be slow enough to be a nuisance. The simulation
itself is unaffected, and RViz and rqt still come up, so you lose only the
third-person view of the scene.

The simulator tasks take a `gui` value of `true` or `false`. Pixi task
arguments are positional, so you pass the value on its own rather than
`gui=false`:

```
pixi run ass_1_1 false
pixi run ass_2_collision false
pixi run ass_4_manual false
```

`ass_4_pid` takes the log file first and `gui` second, so pass `none` for
the log file if you only want to turn the window off:

```
pixi run ass_4_pid none false
pixi run ass_4_pid step_response.csv false
```

Leave the argument out and the window appears, which is what you want most of
the time.

**It will not behave identically to Webots.** Two physics engines never
agree exactly. The robots are set up to match as closely as they can, so
gains and thresholds carry across, but tune to what you actually see rather
than to numbers from someone running Webots.

The Gazebo-specific code is all under `gazebo/src/wasp_as_gazebo/`, and no
assignment package was changed to add it.
