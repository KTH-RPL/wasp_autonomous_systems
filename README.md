# WASP Autonomous Systems Course

## Using Windows?
Native Windows builds don't work for this repo yet, so build and run everything -
pixi, ROS2, and Webots - inside WSL2 instead. Run the following in a Windows
PowerShell terminal (requires administrator rights and a reboot):
```
wsl --install -d Ubuntu
```
Afterwards, open the "Ubuntu" app once to finish first-run setup (username/password),
then do everything below inside that Ubuntu terminal instead of a native Windows one.

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


## Installing the course specific code
Open a terminal and move to the directory where you want to have the code you work with.

**NOTE:** Make sure to not work in a directory managed by Google Drive, iCloud, Dropbox or some other cloud service. There are somewhere between half a million and one million files created by Pixi to setup large parts of a Ubuntu 24.04 systems. Synching these files will be problematic for the network and your computer will spent a lot of efforts trying to keep everything up to date.

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
