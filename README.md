# WASP Autonomous Systems Course

## Using Windows?
Native Windows builds don't work for this repo yet, so build and run everything -
pixi, ROS2, and Webots - inside WSL2 instead. Run the following in a Windows
PowerShell terminal (requires administrator rights and a reboot):
```
wsl --install -d Ubuntu-24.04
```
Afterwards, open the "Ubuntu 24.04" app once to finish first-run setup
(username/password), then do everything below inside that Ubuntu terminal instead of a
native Windows one. Install 24.04 specifically rather than plain `Ubuntu`, which now
gives you a newer release the course has not been tested on - see
[docs/installation-windows.md](docs/installation-windows.md).

When you get to cloning the repo further down, **clone into WSL2's own filesystem**
(e.g. `~/wasp_autonomous_systems`), not into a Windows checkout accessed via
`/mnt/c/...`. Two reasons: building through the `/mnt/c` passthrough is much slower
(lots of small file I/O), and `git` inside WSL2 sees permission/line-ending
differences on NTFS-mounted files that make every file look "modified" even though
nothing changed.

## Supported platforms 
* Linux: Tested on x86-64 with Ubuntu 24.04.
* Mac: Tested on AArch64 system with macOS 26.6.2 (chip Apple M4 Pro).
* Windows: Net yet tested

## Core components used 
* [Pixi](https://pixi.prefix.dev/latest/): A cross-platform package management system that allows us to run ROS2 Jazzy in a Ubuntu 24.04 in the same way on Mac, Linux and Windows.
* [Webots](https://cyberbotics.com): A simulator with a physics engine. If it was not for our Mac users we could have done it all with Pixi using the Gazebo simulator but there is a bug that makes it impossible to simulate camera like sensors which we need.
* Matlab: Used in one task in a mandatory assignment and two conditionally elective assignments. Available from your university.
  
## Installing Pixi and Webots
* Mac: Instructions [here](docs/installation-mac.md)
* Linux: Instructions [here](docs/installation-linux.md)
* Windows: Instructions [here](docs/installation-windows.md)

### Cannot install Webots on your computer?
Some employer-managed computers only allow software from the App Store or a
company software centre, and Webots is not distributed that way. If that is your
situation, there is an alternative setup that uses the Gazebo simulator instead,
which Pixi installs for you along with everything else. See
[docs/gazebo-alternative.md](docs/gazebo-alternative.md). You work from the
`gazebo` folder in this repository, and everything after that is exactly as the
assignment text describes - same commands, same files to edit. Only use it if
you genuinely cannot install Webots; it is not the recommended path.


## Installing the course specific code
Open a terminal and move to the directory where you want to have the code you work with.

**NOTE:** Make sure to not work in a directory managed by Google Drive, iCloud, Dropbox, OneDrive or some other cloud service. There are somewhere between half a million and one million files created by Pixi to setup large parts of a Ubuntu 24.04 systems. Synching these files will be problematic for the network and your computer will spent a lot of efforts trying to keep everything up to date.

**NOTE:** Avoid non-ASCII characters (e.g. å, ä, ö) anywhere in the path to the folder you clone into. Some of the build tooling does not handle these correctly and will fail with confusing errors.

Run the following to download the course code 
```
git clone -b ht26 https://github.com/KTH-RPL/wasp_autonomous_systems.git
```
Build the software (this will take some time) and download two files with data to use.

**NOTE:** Make sure to **restart the terminal** before you run the Pixi commands below (so that Pixi is in the PATH).

```
cd wasp_autonomous_systems
pixi run build
pixi run download_rosbags
```

## The first time you start a simulation

Expect it to take much longer than you think it should, and much longer than
every later start. Webots fetches the 3D models and textures for a world from
the internet the first time that world is opened - the apartment used in
Assignment 1.1 pulls down more than eighty separate files. They are cached
afterwards, so the second start of the same world is far quicker.

While that is happening, RViz often opens long before the simulator is ready.
An empty RViz with no sensor data is normal during a first start, and does not
mean anything is wrong. Give it a few minutes.

If it seems to be stuck, or RViz still shows nothing once the simulator window
has finished loading, stop everything with `Ctrl+C` in each terminal and start
again. Occasionally one of those downloads stalls part way through; starting
again picks up from whatever was already cached, so it costs you much less the
second time. It is worth trying twice before concluding that something is
broken.


## Uninstalling everything after the course
Most of what this course installs lives inside the `wasp_autonomous_systems` folder
you cloned, so deleting that folder is the main step:
```
rm -rf wasp_autonomous_systems
```

That is not quite everything, though. Pixi keeps its downloaded packages in a cache
outside the folder, shared across all your Pixi projects, and a few tools cache data
in your home directory too. Together these can be tens of GB, so if you want the space
back you need to clear them as well.

**Pixi's package cache.** Run this from anywhere:
```
pixi clean cache --conda
```
This cache is shared by all your Pixi projects, so if you have started using Pixi for
something else of your own, that will re-download its packages the next time you run
it. To look at the cache first, or remove it by hand, it is at
`~/Library/Caches/rattler` on macOS and `~/.cache/rattler` on Linux and WSL2, and
`pixi info` prints the exact path as "Cache dir".

**Model files downloaded by Assignment 2.** The CLIP, DINOv2 and Grounding DINO models
are fetched from Hugging Face on first use and cached in your home directory:
```
rm -rf ~/.cache/huggingface
```

**ROS logs**, written every time you launch something:
```
rm -rf ~/.ros/log
```

**Gazebo's own cache**, only if you used the [Gazebo
alternative](docs/gazebo-alternative.md):
```
rm -rf ~/.gz
```

**Pixi itself**, if you do not want to keep it. On Linux, macOS and WSL2:
```
rm -rf ~/.pixi
```
and remove the line the installer added to your shell startup file (`~/.bashrc`,
`~/.zshrc` or similar) that puts `~/.pixi/bin` on your `PATH`.

## Known issues on macOS
- Closing RViz reliably triggers macOS's crash reporter:

  <img src="docs/images/rviz2-quit-unexpectedly.png" width="200"/>

  This is a known upstream ROS2 issue, not caused by anything in this
  repo: `pluginlib`'s `class_loader` crashes (`abort()` via
  `class_loader::ClassLoader::~ClassLoader()`) during shared-library
  teardown at process exit, a shutdown-ordering bug that's worse on
  macOS than Linux. It happens *after* RViz has already done everything
  it was asked to do - purely cosmetic, nothing is lost. Click **Ignore**
  and move on.
