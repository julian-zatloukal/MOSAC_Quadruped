#!/bin/sh
# Builds gait_runner in Docker. Arguments go to make:
#   ./build.sh                          firmware -> build/gait_runner.bin (.elf, .hex)
#   ./build.sh RUN_MODE=RUN_STAND       with a setting of Src/config.h overridden
#   ./build.sh test                     C walk against the Python walk
#   ./build.sh report                   the board's dry run of the walk
#   ./build.sh clean
# The whole repository is mounted, because the HAL and the pot tables live in
# quadruped_control_mcu/ and the walk parameters in Webots/controllers/.
set -eu

HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
IMAGE=mosac-gait-runner-build:1

docker build -q -t "$IMAGE" "$HERE/docker" >/dev/null
exec docker run --rm -u "$(id -u):$(id -g)" -v "$REPO":/repo -w /repo/docs/gait_runner "$IMAGE" make "$@"
