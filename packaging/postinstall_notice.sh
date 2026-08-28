#!/bin/sh
# Run as a task dependency (see the postinstall_notice task in pixi.toml),
# not an activation script - activation scripts' stdout is swallowed by
# pixi (only side effects like exported env vars come through), confirmed
# live. Guarded by a marker file under .pixi/ (untracked, recreated by
# every `pixi install`), so this prints once per install, not on every run.
if [ -n "${PIXI_PROJECT_ROOT:-}" ]; then
  NOTICE_MARKER="$PIXI_PROJECT_ROOT/.pixi/.postinstall_notice_shown"
  if [ ! -f "$NOTICE_MARKER" ]; then
    printf '\033[1;33m[wasp-as] Note: if pixi install suggested running `pixi config set --local run-post-link-scripts insecure`, do NOT run it - it re-enables arbitrary script execution on every future install and is not needed here.\033[0m\n'
    mkdir -p "$(dirname "$NOTICE_MARKER")" 2>/dev/null
    touch "$NOTICE_MARKER" 2>/dev/null
  fi
fi
