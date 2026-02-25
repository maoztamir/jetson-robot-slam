# Autostart on Boot

The `install_service.sh` script installs a systemd service that starts the robot SLAM pipeline automatically whenever the device powers on. The service waits for a network connection before starting (required for AWS IoT) and restarts automatically if the process crashes.

Works on both the **Jetson Nano** and **Raspberry Pi 5**.

---

## Install

Run once on the device, from the project root:

```bash
bash scripts/install_service.sh
```

The script will:
1. Make `scripts/start_robot.sh` executable
2. Write `/etc/systemd/system/robot-slam.service` (runs as the current user)
3. Enable the service so it starts on every boot

---

## Start / Stop / Status

```bash
# Start immediately (without rebooting)
sudo systemctl start robot-slam

# Stop the pipeline
sudo systemctl stop robot-slam

# Check whether it is running
sudo systemctl status robot-slam

# Disable autostart (won't start on next boot)
sudo systemctl disable robot-slam

# Re-enable autostart
sudo systemctl enable robot-slam
```

---

## Logs

The pipeline writes to two places:

```bash
# systemd journal (live)
journalctl -u robot-slam -f

# Application log file (rotating, includes all session history)
tail -f logs/service.log
tail -f logs/robot.log
```

---

## How It Works

### `scripts/start_robot.sh`

A thin shell wrapper that:
- Activates the `~/venv38` virtual environment
- Selects `config/local_config.yaml` if it exists, otherwise falls back to `config/default_config.yaml`
- Runs `python -m src.main` with the selected config

### `/etc/systemd/system/robot-slam.service`

```ini
[Unit]
Description=Robot SLAM Pipeline
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=<your-username>
WorkingDirectory=/home/<your-username>/jetson-robot-slam
ExecStart=/home/<your-username>/jetson-robot-slam/scripts/start_robot.sh
Restart=on-failure
RestartSec=10
StandardOutput=append:.../logs/service.log
StandardError=append:.../logs/service.log

[Install]
WantedBy=multi-user.target
```

Key settings:
- `After=network-online.target` — waits for a real network connection before starting, so AWS IoT can connect
- `Restart=on-failure` + `RestartSec=10` — automatically restarts 10 seconds after any crash
- Runs as the current user (not root), so the venv, certs, and config paths all resolve correctly

---

## Uninstall

```bash
sudo systemctl stop robot-slam
sudo systemctl disable robot-slam
sudo rm /etc/systemd/system/robot-slam.service
sudo systemctl daemon-reload
```

---

## Troubleshooting

### Service fails to start

Check the log for the error:

```bash
journalctl -u robot-slam -e
tail -50 logs/service.log
```

### Network not ready — AWS IoT connection fails on boot

`network-online.target` may resolve before Wi-Fi fully connects on some systems. Add a short delay to `scripts/start_robot.sh`:

```bash
# Add near the top of start_robot.sh, before the exec line:
sleep 15
```

### Wrong config being used

Check which config was loaded:

```bash
grep "config:" logs/service.log | tail -5
```

Make sure `config/local_config.yaml` exists on the device with the correct `camera.backend` and `aws` settings.

### Venv not found

If `~/venv38` is missing, recreate it:

```bash
# Jetson / RPi5 (pyenv)
~/.pyenv/versions/3.8.19/bin/python3.8 -m venv ~/venv38
source ~/venv38/bin/activate
pip install -r requirements.txt
```
