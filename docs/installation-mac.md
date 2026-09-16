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

**Workaround**, if you have an older SDK still installed alongside the new one:
```
ls /Library/Developer/CommandLineTools/SDKs/
```
If that lists an older SDK (e.g. `MacOSX26.5.sdk`, anything before the newest one),
point the build at it:
```
export SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk
export CONDA_BUILD_SYSROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk
pixi run build
```
If only the newest SDK is installed, install an older, stable Xcode Command Line Tools
release from [Apple's developer site](https://developer.apple.com/download/all/) and
switch to it with `xcode-select -s`.
