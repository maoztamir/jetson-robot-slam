#!/usr/bin/env bash
# Install the robot-slam systemd service so src.main runs on every boot.
# Run once on each device:
#   bash scripts/install_service.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CURRENT_USER="$(whoami)"
SERVICE_NAME="robot-slam"
SERVICE_FILE="/etc/systemd/system/${SERVICE_NAME}.service"

echo "Installing ${SERVICE_NAME}.service for user: $CURRENT_USER"
echo "Project root: $PROJECT_ROOT"

# Make the startup wrapper executable
chmod +x "$SCRIPT_DIR/start_robot.sh"

# Write the systemd unit file
sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=Robot SLAM Pipeline
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=${CURRENT_USER}
WorkingDirectory=${PROJECT_ROOT}
ExecStart=${SCRIPT_DIR}/start_robot.sh
Restart=on-failure
RestartSec=10
StandardOutput=append:${PROJECT_ROOT}/logs/service.log
StandardError=append:${PROJECT_ROOT}/logs/service.log

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd and enable the service
sudo systemctl daemon-reload
sudo systemctl enable "$SERVICE_NAME"

echo ""
echo "Service installed and enabled."
echo ""
echo "Useful commands:"
echo "  sudo systemctl start  $SERVICE_NAME   # start now"
echo "  sudo systemctl stop   $SERVICE_NAME   # stop"
echo "  sudo systemctl status $SERVICE_NAME   # check status"
echo "  journalctl -u $SERVICE_NAME -f        # follow logs"
echo "  tail -f ${PROJECT_ROOT}/logs/service.log"
