#!/usr/bin/env bash
set -e

cd "$(dirname "$0")"

# The Jetson image keeps its camera-enabled OpenCV/PyQt packages in the
# system Python.  An activated venv/conda environment can put another
# `python3` first on PATH and make the UI report a misleading missing-cv2
# error.  Keep the production launcher deterministic while still allowing an
# explicit override for development.
PYTHON_BIN="${ARM_MOLE_PYTHON:-/usr/bin/python3}"
if [[ ! -x "$PYTHON_BIN" ]]; then
    PYTHON_BIN="$(command -v python3)"
fi

unset PYTHONHOME
unset PYTHONNOUSERSITE
exec "$PYTHON_BIN" -m app.main "$@"
