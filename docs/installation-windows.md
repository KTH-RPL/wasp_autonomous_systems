## Windows: build and run everything inside WSL2

Native Windows builds don't work for this repo yet, so build and run everything -
pixi, ROS2, and Webots - inside WSL2 instead. This has worked when tried, including
running a full assignment with the Webots GUI and keyboard teleop. Install Webots
inside WSL2 too, not natively on Windows - the ROS2/Webots plugin needs to match
whatever platform the rest of the workspace is built for.

**Everything below runs inside an Ubuntu (WSL2) terminal, not a native Windows one.**

## Install WSL2
Requires administrator rights and a reboot.

Run the following in a Windows PowerShell terminal (WSL2 doesn't exist yet at this point):
```
wsl --install -d Ubuntu-24.04
```
Afterwards, open the "Ubuntu 24.04" app once to finish first-run setup (username/password).
WSLg (GUI app support, needed for the Webots window) is built in on Windows 11 - no
extra setup needed.

**Install 24.04 specifically, not plain `Ubuntu`.** `wsl --install -d Ubuntu` gives you
whatever Microsoft currently ships as the default, which is no longer 24.04. The course
is built and tested against 24.04, and the parts that come from the distro rather than
from Pixi - the C/C++ compiler used to build Webots' controller library, and the Webots
`.deb`'s own dependencies - are where a newer release causes trouble. Check what you
have with:
```
lsb_release -a
```
If it says anything other than 24.04, install the right one alongside it - WSL2 runs
several distributions side by side, so nothing you already have is disturbed:
```
wsl --install -d Ubuntu-24.04
```
and use the "Ubuntu 24.04" app from then on. `wsl -l -v` lists what is installed, and
`wsl -l -o` lists what is available.

If `wsl -l -v` still shows no distributions after the reboot, just run the
`wsl --install -d Ubuntu-24.04` command again - on some machines it takes two rounds
(one to enable the underlying Windows feature, a second to actually install Ubuntu).

No admin rights on your machine? There's no supported no-admin install path for WSL2
either (enabling it is an OS feature toggle). Check whether your organization's IT
provides a self-service install, or use a remote/cloud Linux environment instead.

From here on, open the "Ubuntu 24.04" app and run everything below from inside it.

**Note:** To open more terminals into the same running WSL2 instance (e.g. one for the
simulation, one for teleop), just launch the "Ubuntu 24.04" app again, or open a new tab for
it if you're using Windows Terminal - both land in the same WSL2 session, so anything
already running (like a Webots window) is visible/reachable from either.

**Note:** Closing the Ubuntu terminal window doesn't stop WSL2 - it keeps running in
the background (using whatever CPU/RAM it's currently using) until you either restart
Windows or explicitly shut it down. To fully stop it, run this in a **Windows**
PowerShell terminal (not inside Ubuntu):
```
wsl --shutdown
```

## Update the package lists
A freshly installed distribution ships with no package lists, so every `apt install`
below fails with "Unable to locate package" until you do this once.

Run the following in a WSL2 terminal:
```
sudo apt update
```

## Install a C/C++ compiler
`pixi run build` needs a system C/C++ toolchain - Webots' vendored controller library
Makefile calls `gcc`/`make` directly rather than the conda-provided compiler. A fresh
WSL2 Ubuntu doesn't have one yet.

Run the following in a WSL2 terminal:
```
sudo apt install build-essential
```

## Install Pixi 
Run the following in a WSL2 terminal:
```
curl -fsSL https://pixi.sh/install.sh | sh
```
You might want to set up [autocomplete](https://pixi.prefix.dev/latest/installation/#autocompletion) for smoother operation of Pixi.

**Note:** Restart the terminal (or `source ~/.bashrc`) afterwards.

## Install Webots R2025a 
Run the following in a WSL2 terminal:
```
curl -L -O https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
sudo apt install ./webots_2025a_amd64.deb
```
Do not worry if you see the message:
```
N: Download is performed unsandboxed as root file 
'<SOMETHING>/webots_2025a_amd64.deb' couldn't be 
accessed by user '_apt'. - pkgAcquire::Run
(13: Permission denied)
```
that is expected.

A fresh WSL2 Ubuntu may then fail to start Webots with:
```
error while loading shared libraries: libsndio.so.7: cannot open shared object file
```
Fix:
```
sudo apt install libsndio7.0
```

**Note:** The `.deb` above installs Webots to `/usr/local/webots`, which this repo's
pixi environment already auto-detects - you don't need to set anything extra. If you
installed Webots somewhere else, set `WEBOTS_HOME` to **that install directory itself**
(not `/usr/local/bin`, which only holds a `webots` launcher symlink pointing into it -
setting `WEBOTS_HOME` to that instead will make Webots fail to be found) before running
`pixi run build`/`pixi run ass_*`, e.g.:
```
export WEBOTS_HOME=/path/to/your/webots
```
Without this, WSL2 is wrongly assumed to only ever have Webots installed natively on
Windows, and you'll be prompted to auto-install a Windows copy instead of using the one
you already have.

## Where to clone the repository
**Clone into WSL2's own filesystem** (e.g. `~/wasp_autonomous_systems`), not into a
Windows folder reached through `/mnt/c/...`. Two reasons: building through the `/mnt/c`
passthrough is much slower, because the build is thousands of small file operations;
and `git` inside WSL2 sees permission and line-ending differences on NTFS-mounted
files, which makes every file look modified even though nothing changed.

## If the simulation runs slowly
See [Simulation speed under WSL2](performance-wsl2.md).
