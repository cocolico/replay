#!/usr/bin/env bash
REPLAY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$REPLAY_ROOT/vendor/pyaubo_sdk_0_26:$REPLAY_ROOT/vendor/wuji_sdk_2026_11_9:$REPLAY_ROOT/vendor/replay_ik/src:$REPLAY_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$REPLAY_ROOT/vendor/wuji_sdk_2026_11_9/wuji_sdk.libs${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
