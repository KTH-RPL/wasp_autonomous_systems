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
wsl --install -d Ubuntu
```
Afterwards, open the "Ubuntu" app once to finish first-run setup (username/password).
WSLg (GUI app support, needed for the Webots window) is built in on Windows 11 - no
extra setup needed.

No admin rights on your machine? There's no supported no-admin install path for WSL2
either (enabling it is an OS feature toggle). Check whether your organization's IT
provides a self-service install, or use a remote/cloud Linux environment instead.

From here on, open the "Ubuntu" app and run everything below from inside it.

**Note:** To open more terminals into the same running WSL2 instance (e.g. one for the
simulation, one for teleop), just launch the "Ubuntu" app again, or open a new tab for
it if you're using Windows Terminal - both land in the same WSL2 session, so anything
already running (like a Webots window) is visible/reachable from either.

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
