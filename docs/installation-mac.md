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
Line Tools install at all: download a full copy of **Xcode 26.6** (confirmed working -
any version before 27 avoids this bug, this is just the one we've tested) from
[Apple's developer downloads page](https://developer.apple.com/download/all/)
(sign in with a free Apple ID **first, at** [developer.apple.com](https://developer.apple.com) -
visiting the downloads page directly while signed out can get stuck reloading).

You don't need to install it anywhere or touch your system's default Xcode/Command Line
Tools - download the `.xip` directly into your cloned repo folder (the path below,
`/Users/pelle/wasp_autonomous_systems`, is just an example - replace the whole thing
with wherever you actually cloned this repo, not just the username):
```
cd /Users/pelle/wasp_autonomous_systems
```
so it ends up at e.g.
```
/Users/pelle/wasp_autonomous_systems/Xcode_26.6.xip
```
Then expand it in place with macOS's own `xip` tool (the same thing Finder does if you
double-click a `.xip`, just from the terminal so it stays in this folder instead of
wherever Finder happens to be pointed):
```
xip --expand Xcode_26.6.xip
```
This creates `Xcode.app` alongside it; rename it so it doesn't collide with anything and
it's clear which version it is:
```
mv Xcode.app Xcode_26.6.app
```
You can now delete the `.xip` itself (it's no longer needed once expanded):
```
rm Xcode_26.6.xip
```
And point your build at this Xcode's SDK:
```
SDKROOT=/Users/pelle/wasp_autonomous_systems/Xcode_26.6.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk CONDA_BUILD_SYSROOT=/Users/pelle/wasp_autonomous_systems/Xcode_26.6.app/Contents/Developer/Platforms/MacOSX.platform/Developer/SDKs/MacOSX.sdk pixi run build
```
This is a large download (several GB) and `Xcode_26.6.app` itself is much bigger once
expanded, but keeping it inside the repo folder (it's already in `.gitignore`) means
it's never committed and gets cleaned up automatically along with everything else if you
delete the whole `wasp_autonomous_systems` folder at the end of the course - and unlike
a partial/reconstructed SDK, this is guaranteed correct since it's just Apple's own
Xcode, expanded by Apple's own tool.
