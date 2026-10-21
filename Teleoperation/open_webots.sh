#!/bin/sh
# Opens the teleoperation world in a new Webots window and returns at once; the robot stands up and waits for the page
# (python3 server.py).
# Usage: ./open_webots.sh [world]          (default: ../Webots/worlds/MOSAC_REAL_V4_teleop.wbt)
# Webots is looked for in WEBOTS_HOME, /Applications and /Volumes/Portable/lb/Software/Webots, e.g.
#   WEBOTS_HOME=/Applications/Webots.app ./open_webots.sh
# On Linux, point WEBOTS to the executable if it isn't on the PATH (as for ../Webots/train_fast.sh).

HERE="$(cd "$(dirname "$0")" && pwd)"
WORLD="${1:-$HERE/../Webots/worlds/MOSAC_REAL_V4_teleop.wbt}"
[ -f "$WORLD" ] || { echo "No such world: $WORLD" >&2; exit 1; }
WORLD="$(cd "$(dirname "$WORLD")" && pwd)/$(basename "$WORLD")"  # absolute: Webots does not start in this folder

# Only one teleoperation world can run at a time: a second robot cannot listen on the same ports.
if command -v lsof >/dev/null && lsof -nP -iUDP:9871 -iTCP:9873 -sTCP:LISTEN >/dev/null 2>&1; then
    echo "The robot's ports (UDP 9871, TCP 9873) are taken, most likely by a Webots window already running this world:" >&2
    lsof -nP -iUDP:9871 -iTCP:9873 -sTCP:LISTEN >&2
    echo "Close that window first." >&2
    exit 1
fi

if [ "$(uname)" = Darwin ]; then
    for APP in "$WEBOTS_HOME" /Applications/Webots.app /Volumes/Portable/lb/Software/Webots/Webots.app; do
        [ -n "$APP" ] && [ -d "$APP" ] && break
        APP=""
    done
    [ -n "$APP" ] || { echo "Webots not found: set WEBOTS_HOME to Webots.app" >&2; exit 1; }
    # open does not pass this shell's environment on, so the teleoperation settings that are set go explicitly
    set --
    [ -n "$TELEOP_BIND" ] && set -- "$@" --env "TELEOP_BIND=$TELEOP_BIND"
    [ -n "$TELEOP_SERVER_HOST" ] && set -- "$@" --env "TELEOP_SERVER_HOST=$TELEOP_SERVER_HOST"
    [ -n "$TELEOP_FFMPEG" ] && set -- "$@" --env "TELEOP_FFMPEG=$TELEOP_FFMPEG"
    exec open -n -a "$APP" "$@" --args "$WORLD"  # -n: a new window even if Webots is already open
fi

nohup "${WEBOTS:-webots}" "$WORLD" >/dev/null 2>&1 &
