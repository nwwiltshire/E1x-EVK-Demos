#!/usr/bin/env bash
# End-to-end test: builds the firmware simulator and runs e2e_check.py
# against the real binary with synthetic room audio.  Needs python3 +
# numpy (no audio hardware) — CI-friendly; uses ./venv if present.
set -euo pipefail
cd "$(dirname "$0")/.."

PY=python3
[ -x venv/bin/python3 ] && PY=venv/bin/python3

make -C firmware sim
"$PY" tests/e2e_check.py --fw firmware/build/firmware_sim
