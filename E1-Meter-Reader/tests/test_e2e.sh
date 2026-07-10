#!/bin/sh
# Build the simulator firmware and run the end-to-end protocol checks
# against it.  No hardware needed (numpy + opencv for the synthetic
# gauge renderer).
set -e
cd "$(dirname "$0")/.."

make -C firmware sim

PY=python3
if [ -x venv/bin/python ]; then
    PY=venv/bin/python
fi

exec "$PY" tests/e2e_check.py --fw firmware/build/firmware_sim
