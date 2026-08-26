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
  # webots-controller (a compiled binary, not the Python-side utils.py which
  # has its own auto-detection fallback) requires WEBOTS_HOME to actually be
  # in the process environment, not just a local shell variable - it exits
  # immediately with "Set the path to your webots installation folder in
  # WEBOTS_HOME environment variable" otherwise.
  if [ -n "${WEBOTS_HOME:-}" ]; then
    export WEBOTS_HOME
  fi
  if [ -n "${WEBOTS_HOME:-}" ] && [ -d "$WEBOTS_HOME/Contents/lib/controller" ]; then
    export DYLD_LIBRARY_PATH="$WEBOTS_HOME/Contents/lib/controller:${DYLD_LIBRARY_PATH:-}"
  fi

  # Also needed at runtime (macOS only): pluginlib-loaded shared libraries
  # (e.g. webots_ros2_control's controller_manager plugin) pull in message
  # typesupport .dylibs that aren't on the driver binary's own rpath -
  # matches this repo's README ("the built driver needs DYLD_LIBRARY_PATH
  # including <pixi-env>/lib at runtime"). Assignment 4's Mavic controller
  # never hits this (it doesn't use ros2_control), which is why this gap
  # went unnoticed until Assignments 1/2's Turtlebot did.
  if [ -n "${CONDA_PREFIX:-}" ] && [ -d "$CONDA_PREFIX/lib" ]; then
    export DYLD_LIBRARY_PATH="$CONDA_PREFIX/lib:${DYLD_LIBRARY_PATH:-}"
  fi
fi
