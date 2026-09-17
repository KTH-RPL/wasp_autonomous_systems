#!/bin/sh
# Extract a single platform SDK (default: MacOSX) from a downloaded Xcode
# .xip, without expanding the whole multi-GB app. Useful for testing against
# a specific Xcode/SDK version (e.g. to reproduce or work around an
# SDK-specific build issue, as with the Xcode 27 / arm64e.x1 linker bug - see
# docs/installation-mac.md) without needing tens of GB of free disk space for
# a full Xcode install.
#
# Usage: extract-xcode-sdk.sh <path-to-Xcode.xip> [output-dir] [platform]
#   output-dir defaults to ./xcode-sdk-extracted
#   platform defaults to MacOSX (matches .../SDKs/<platform>.sdk) - other
#   valid values: iPhoneOS, iPhoneSimulator, WatchOS, AppleTVOS, etc.
#
# How it works: a .xip is a signed XAR archive with a "Content" entry -
# Apple's "pbzx" format: a magic header, a one-time flags field, then a
# repeating sequence of (uncompressed size, compressed size, raw .xz chunk).
# Content is extracted via the system xar tool, decoded chunk-by-chunk with a
# small embedded Python script, and piped directly into cpio with a path
# pattern - only matching entries are ever written to disk, so nothing
# outside the requested SDK is materialized. Confirmed live: this pulls out
# just the MacOSX SDK (~800MB) in about a minute, vs. 40+GB and a much longer
# wait for a full Xcode.app expansion.
#
# A handful of "Hard-link target ... does not exist" warnings from cpio are
# expected and harmless - a few files inside one platform SDK are hardlinked
# to identical files in other platform SDKs (which this script deliberately
# doesn't extract), mostly generic headers/man pages, not SDK-specific
# content.

set -eu

XIP="${1:?Usage: $0 <path-to-Xcode.xip> [output-dir] [platform]}"
OUT="${2:-./xcode-sdk-extracted}"
PLATFORM="${3:-MacOSX}"

case "$XIP" in
  /*) XIP_ABS="$XIP" ;;
  *) XIP_ABS="$(pwd)/$XIP" ;;
esac

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

echo "Extracting Content from $XIP_ABS ..."
(cd "$WORK" && xar -x -f "$XIP_ABS" Content)

cat > "$WORK/pbzx_decode.py" <<'PYEOF'
import sys, struct, lzma

def main():
    inf = sys.stdin.buffer
    outf = sys.stdout.buffer
    magic = inf.read(4)
    assert magic == b'pbzx', f"bad magic: {magic!r}"
    inf.read(8)  # one-time flags field, not part of the per-chunk loop
    while True:
        header = inf.read(16)
        if len(header) < 16:
            break
        _, compressed_size = struct.unpack('>QQ', header)
        data = inf.read(compressed_size)
        if len(data) < compressed_size:
            break
        if data[:6] == b'\xfd7zXZ\x00':
            outf.write(lzma.decompress(data))
        else:
            outf.write(data)
        outf.flush()

if __name__ == '__main__':
    main()
PYEOF

mkdir -p "$OUT"
touch "$OUT/COLCON_IGNORE"  # in case output-dir ever ends up somewhere colcon scans

echo "Decoding pbzx and extracting SDKs/${PLATFORM}.sdk into $OUT ..."
cat "$WORK/Content" | python3 "$WORK/pbzx_decode.py" \
  | (cd "$OUT" && cpio -idm --no-preserve-owner "*/SDKs/${PLATFORM}.sdk*") || true

SDK_PATH="$OUT/Xcode.app/Contents/Developer/Platforms/${PLATFORM}.platform/Developer/SDKs/${PLATFORM}.sdk"
if [ -d "$SDK_PATH" ]; then
  echo "Done. SDK extracted to: $SDK_PATH"
  echo "Point a build at it with, e.g.:"
  echo "  SDKROOT=$SDK_PATH CONDA_BUILD_SYSROOT=$SDK_PATH pixi run build"
else
  echo "Something went wrong - expected SDK directory not found: $SDK_PATH" >&2
  exit 1
fi
