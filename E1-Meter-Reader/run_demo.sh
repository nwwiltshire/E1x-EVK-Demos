#!/bin/sh
# One-command, no-hardware demo: build the firmware simulator, start
# it, and open the viewer with the built-in animated gauge.
#
#   ./run_demo.sh                 # synthetic gauge -> sim firmware
#   ./run_demo.sh --source camera # webcam -> sim firmware
#
# For real hardware see README "Going to hardware".
set -e
cd "$(dirname "$0")"

PY=python3
if [ -x venv/bin/python ]; then
    PY=venv/bin/python
fi
if ! "$PY" -c "import cv2, numpy, serial" 2>/dev/null; then
    echo "bootstrapping venv..."
    python3 -m venv venv
    venv/bin/pip install -q -r host/requirements.txt
    PY=venv/bin/python
fi

make -C firmware sim

PORT=${E1_SIM_PORT:-5555}
E1_SIM_PORT=$PORT ./firmware/build/firmware_sim &
FW_PID=$!
trap 'kill $FW_PID 2>/dev/null' EXIT INT TERM
sleep 0.5

if [ $# -gt 0 ]; then
    exec_args="$@"
else
    exec_args="--source synth"
fi
"$PY" host/meter_viewer.py --sim 127.0.0.1:$PORT --units PSI $exec_args
