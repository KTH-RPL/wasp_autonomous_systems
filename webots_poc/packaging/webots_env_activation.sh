#!/bin/sh
# macOS only: webots_ros2_driver links against Webots.app's own controller
# libraries at build time; conda/pixi-build's binary relocation strips the
# rpath entry pointing at it, so DYLD_LIBRARY_PATH has to be exported
# explicitly. Respects a WEBOTS_HOME override first (matching the runtime
# Python-side get_webots_home() convention and the CMakeLists.txt build-time
# lookup), then checks /Applications (admin-rights install) and
# ~/Applications (no-admin-rights install) before giving up quietly.
if [ "$(uname)" = "Darwin" ]; then
  if [ -z "${WEBOTS_HOME:-}" ]; then
    if [ -d "/Applications/Webots.app" ]; then
      WEBOTS_HOME="/Applications/Webots.app"
    elif [ -d "$HOME/Applications/Webots.app" ]; then
      WEBOTS_HOME="$HOME/Applications/Webots.app"
    fi
  fi
  if [ -n "${WEBOTS_HOME:-}" ] && [ -d "$WEBOTS_HOME/Contents/lib/controller" ]; then
    export DYLD_LIBRARY_PATH="$WEBOTS_HOME/Contents/lib/controller:${DYLD_LIBRARY_PATH:-}"
  fi
fi
