## Install Pixi 
You find full instructions [here](https://pixi.prefix.dev/latest/installation/).
Run the following in a terminal
```
curl -fsSL https://pixi.sh/install.sh | sh
```
You might want to set up [autocomplete](https://pixi.prefix.dev/latest/installation/#autocompletion) smoother operation of Pixi.

**Note:** Restart the terminal 

## Install Webots R2025a 
Run the following in a terminal
```
curl -L -O https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a.dmg
open webots-R2025a.dmg
```

## `pixi run build` fails with "unknown architecture arm64e.x1-macos"
If you recently updated Xcode's Command Line Tools, `pixi run build` may fail early
(often on the very first package it builds) with something like:
```
ld: warning: ignoring file .../MacOSX.sdk/usr/lib/libSystem.tbd, malformed file
.../MacOSX.sdk/usr/lib/libSystem.tbd:4:20: error: unknown architecture
                   arm64e.x1-macos, arm64e.x1-maccatalyst ]
ld: dynamic executables or dylibs must link with libSystem.dylib for architecture arm64
```
This is a known upstream bug: newer Xcode Command Line Tools (Xcode 27+) ship a macOS
SDK using an `arm64e.x1` architecture variant that Pixi's bundled linker doesn't yet
understand ([conda-forge/cctools-and-ld64-feedstock#112](https://github.com/conda-forge/cctools-and-ld64-feedstock/issues/112)),
not something fixable from this repo.

**Workaround:** check whether an older SDK happens to still be present alongside the
new one (Xcode Command Line Tools updates sometimes leave previous SDK versions in
place):
```
ls /Library/Developer/CommandLineTools/SDKs/
```
If that lists an older SDK (anything before the newest one, e.g. `MacOSX26.5.sdk`),
point just this one command at it - this only affects this single build, nothing else
on your system:
```
SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk CONDA_BUILD_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk pixi run build
```
If no older SDK is listed, there isn't a clean workaround yet - this is waiting on a
fixed linker release upstream. Get in touch if you're stuck here.
