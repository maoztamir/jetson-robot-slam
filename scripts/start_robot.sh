#!/usr/bin/env bash
# Startup wrapper for the robot SLAM pipeline.
# Called by the systemd service; activates the venv and runs src.main.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

# Activate virtual environment.
# ROBOT_VENV is set by the systemd service (baked in at install time by
# install_service.sh).  When running manually, fall back to venv38 or test38.
if [ -z "$ROBOT_VENV" ]; then
    if [ -d "$HOME/venv38" ]; then
        ROBOT_VENV="$HOME/venv38"
    elif [ -d "$HOME/test38" ]; then
        ROBOT_VENV="$HOME/test38"
    fi
fi
VENV="${ROBOT_VENV}"
if [ -f "$VENV/bin/activate" ]; then
    # shellcheck disable=SC1091
    source "$VENV/bin/activate"
    echo "[$(date)] Using venv: $VENV"
else
    echo "WARNING: venv not found at $VENV, using system Python" >&2
fi

# Use local_config.yaml if present, otherwise fall back to default
if [ -f "$PROJECT_ROOT/config/local_config.yaml" ]; then
    CONFIG="$PROJECT_ROOT/config/local_config.yaml"
else
    CONFIG="$PROJECT_ROOT/config/default_config.yaml"
fi

echo "[$(date)] Starting robot SLAM pipeline with config: $CONFIG"
exec python -m src.main --config "$CONFIG"
