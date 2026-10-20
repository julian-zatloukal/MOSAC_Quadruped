#!/bin/sh
# Launches Webots for training: fast mode and the 3D view not rendered (it can be turned back on from View > Rendering).
# Usage: ./train_fast.sh [world]          (default: worlds/MOSAC_REAL_V4.wbt)
# If webots isn't on the PATH, point WEBOTS to the executable, e.g.
#   WEBOTS=/Applications/Webots.app/Contents/MacOS/webots ./train_fast.sh

WEBOTS="${WEBOTS:-webots}"
WORLD="${1:-$(dirname "$0")/worlds/MOSAC_REAL_V4.wbt}"

exec "$WEBOTS" --mode=fast --no-rendering "$WORLD"
