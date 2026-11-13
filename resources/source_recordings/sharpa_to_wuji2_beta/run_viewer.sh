#!/usr/bin/env bash
set -eu
bundle_dir="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
bundle_python="${WUJI_REPLAY_PYTHON:-/home/descfly/miniforge3/envs/bench2dex/bin/python}"
if [ ! -x "$bundle_python" ]; then bundle_python=python3; fi
export MUJOCO_GL=glfw
exec "$bundle_python" "$bundle_dir/visualize.py" "$bundle_dir/${1:-episode594}" --viewer
