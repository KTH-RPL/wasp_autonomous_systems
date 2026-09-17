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
If no older SDK is listed, you can get one without touching your current Xcode Command
Line Tools install at all: download **Xcode 26.1.1** (the version this repo is
actually tested against) from [Apple's developer downloads page](https://developer.apple.com/download/all/)
(needs a free Apple ID sign-in) and drop it into `/Applications` under its own name,
e.g. `Xcode_26.1.1.app`. Full Xcode versions can sit side by side as separate apps
without changing your system's default Command Line Tools, and each one carries its
own complete SDK:
```
SDKROOT=/Applications/Xcode_26.1.1.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk CONDA_BUILD_SYSROOT=/Applications/Xcode_26.1.1.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk pixi run build
```
(adjust the `Xcode_26.1.1.app` name/version to whatever you downloaded). This is a large
download (several GB), but it's the most reliable option since it's a real, complete
SDK rather than a workaround.

### Lighter-weight alternative: extract just the SDK, not the whole app
You don't actually need the full Xcode.app installed anywhere - a script in this repo
can pull just the one SDK directory out of the `.xip` you download from Apple, without
ever expanding the whole (40+GB) app. Concretely, from inside your cloned repo (the
path below, `/Users/pelle/wasp_autonomous_systems`, is just an example - replace the
whole thing with wherever you actually cloned this repo, not just the username):
```
cd /Users/pelle/wasp_autonomous_systems
```
Download Xcode from [Apple's developer downloads page](https://developer.apple.com/download/all/)
(needs a free Apple ID sign-in) directly into the repo folder, so it ends up at e.g.
```
/Users/pelle/wasp_autonomous_systems/Xcode_26.1.1.xip
```
Then run:
```
./packaging/extract-xcode-sdk.sh Xcode_26.1.1.xip xcode-sdk-extracted
```
This gives you just the SDK, about 800MB instead of 40+GB, at:
```
/Users/pelle/wasp_autonomous_systems/xcode-sdk-extracted/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk
```
You can now delete the `.xip` itself (it's no longer needed once extracted):
```
rm Xcode_26.1.1.xip
```
And point your build at the extracted SDK:
```
SDKROOT=/Users/pelle/wasp_autonomous_systems/xcode-sdk-extracted/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk CONDA_BUILD_SYSROOT=/Users/pelle/wasp_autonomous_systems/xcode-sdk-extracted/Xcode.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk pixi run build
```
Keeping `xcode-sdk-extracted/` inside the repo (it's already in `.gitignore`) means it's
never committed, and it's automatically cleaned up along with everything else if you
delete the whole `wasp_autonomous_systems` folder at the end of the course.
