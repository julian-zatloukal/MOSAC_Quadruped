# Build environment of gait_runner: the Arm GNU toolchain for the NUCLEO-F412ZG, plus a host compiler and
# Python with numpy for `make test` (the C walk against the Webots Python code). Used by ../build.sh.
FROM debian:bookworm-slim

RUN apt-get update \
 && apt-get install -y --no-install-recommends \
        gcc-arm-none-eabi libnewlib-arm-none-eabi binutils-arm-none-eabi \
        make gcc libc6-dev python3 python3-numpy \
 && rm -rf /var/lib/apt/lists/*

ENV PYTHONDONTWRITEBYTECODE=1
WORKDIR /repo
