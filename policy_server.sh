#!/usr/bin/env bash
set -euo pipefail

POLICY_PYTHON="${LEISAAC_POLICY_PYTHON:-/home/rvi/anaconda3/envs/leisaac-policy/bin/python}"

exec "${POLICY_PYTHON}" -m lerobot.async_inference.policy_server \
    --host=127.0.0.1 \
    --port=8080 \
    --fps=30 \
    --inference_latency=0.033 \
    "$@"
