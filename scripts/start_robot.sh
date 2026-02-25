#!/usr/bin/env bash
# Startup wrapper for the robot SLAM pipeline.
# Called by the systemd service; activates the venv and runs src.main.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

# Activate virtual environment (venv38 created during setup)
VENV="$HOME/venv38"
if [ -f "$VENV/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$VENV/bin/activate"
else
    echo "WARNING: venv38 not found at $VENV, using system Python" >&2
fi

# Use local_config.yaml if present, otherwise fall back to default
if [ -f "$PROJECT_ROOT/config/local_config.yaml" ]; then
    CONFIG="$PROJECT_ROOT/config/local_config.yaml"
else
    CONFIG="$PROJECT_ROOT/config/default_config.yaml"
fi

echo "[$(date)] Starting robot SLAM pipeline with config: $CONFIG"
exec python -m src.main --config "$CONFIG"
