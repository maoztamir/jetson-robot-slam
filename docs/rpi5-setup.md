# Raspberry Pi 5 — Setup & Installation Guide

Complete instructions for provisioning a Raspberry Pi 5 to run the stereo-inertial SLAM pipeline with AWS IoT cloud integration.

> **Jetson Nano users:** see [jetson-setup.md](jetson-setup.md).
> The RPi 5 and Jetson Nano share the same project code, AWS infrastructure, and IMU wiring, but differ in OS, camera pipeline, and performance profile.

---

## Table of Contents

1. [Hardware](#1-hardware)
2. [Flash Raspberry Pi OS](#2-flash-raspberry-pi-os)
3. [First-boot configuration](#3-first-boot-configuration)
4. [Enable I2C and verify the cameras](#4-enable-i2c-and-verify-the-cameras)
5. [Install system packages](#5-install-system-packages)
6. [Patch the camera pipeline for RPi 5](#6-patch-the-camera-pipeline-for-rpi-5)
7. [Install ORB_SLAM3](#7-install-orb_slam3)
8. [Clone the project](#8-clone-the-project)
9. [Install Python dependencies](#9-install-python-dependencies)
10. [Configure the project](#10-configure-the-project)
11. [AWS IoT setup](#11-aws-iot-setup)
12. [Verify everything works](#12-verify-everything-works)
13. [Running the pipeline](#13-running-the-pipeline)
14. [Autostart on boot (optional)](#14-autostart-on-boot-optional)
15. [Troubleshooting](#15-troubleshooting)

---

## 1. Hardware

| Item | Model / Spec |
|---|---|
| SBC | Raspberry Pi 5 **8 GB** (4 GB works but 8 GB gives more headroom for ORB_SLAM3) |
| Camera | Waveshare **IMX219-83** stereo CSI module |
| IMU | **ICM-20948** 9-DoF (soldered on the IMX219-83 PCB) |
| Power | **5 V / 5 A** USB-C supply with PD (the RPi 5 demands more current than earlier models) |
| Storage | 32 GB+ microSD card (Class 10 / A2 speed) or NVMe via PCIe HAT |
| Network | Onboard Wi-Fi 802.11ac or Ethernet |

### Cable adapter — important

The Raspberry Pi 5 uses **22-pin FPC connectors** for its two camera ports (CAM0 and CAM1), which is different from the 15-pin connector on the RPi 4 and Jetson Nano. The IMX219-83 module ships with 15-pin FFC cables.

You need two **15-to-22-pin FFC adapter cables** (available from the Raspberry Pi Foundation and most RPi retailers). Without these the cameras will not be detected.

### Physical connections

- Connect the **left** camera ribbon to **CAM0** (left connector when GPIO header faces away from you).
- Connect the **right** camera ribbon to **CAM1** (right connector).
- Both cables must be inserted with the blue reinforcement strip facing the USB ports on each camera connector.
- The ICM-20948 IMU on the camera PCB communicates over I2C bus 1 at address `0x68` — the same as on the Jetson Nano.
- Use a USB-C supply rated at **5 V / 5 A** (25 W). Standard phone chargers are insufficient for sustained CPU load.

---

## 2. Flash Raspberry Pi OS

1. Download **Raspberry Pi Imager** from `https://www.raspberrypi.com/software/`

2. In the Imager:
   - **Device**: Raspberry Pi 5
   - **OS**: *Raspberry Pi OS (64-bit)* — Bookworm (Debian 12), the full desktop version
   - **Storage**: your microSD card

3. Click the gear icon (⚙) and pre-configure:
   - Hostname (e.g. `rpi-slam`)
   - Username / password
   - Wi-Fi SSID and password
   - Enable SSH

4. Write the image, insert the card, and power on.

> **Why 64-bit Bookworm?**
> 64-bit is required for ORB_SLAM3 and gives access to current Python 3.11 packages. Bookworm ships with `libcamera` and `picamera2` pre-installed, which are the camera stack used here instead of the Jetson's `nvarguscamerasrc`.

---

## 3. First-boot configuration

SSH in or open a terminal and update the system:

```bash
sudo apt update && sudo apt full-upgrade -y
sudo reboot
```

### Enable the camera, I2C, and SPI interfaces

```bash
sudo raspi-config
```

Navigate to: **Interface Options** and enable:
- **Camera** (Legacy Camera off — we use libcamera)
- **I2C**
- **SPI** (optional, not required by this project)

Exit and reboot when prompted.

### Set CPU governor to performance

The RPi 5 has no GPU compute, so CPU throughput is critical for ORB_SLAM3.
`cpufrequtils` is not packaged for Raspberry Pi OS Bookworm — use the direct sysfs approach instead.

```bash
# Apply immediately (all four cores)
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Verify
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
# Expected output: performance
```

Make it persistent across reboots with a one-shot systemd service:

```bash
sudo tee /etc/systemd/system/cpu-performance.service > /dev/null << 'EOF'
[Unit]
Description=Set CPU governor to performance
After=sysinit.target

[Service]
Type=oneshot
ExecStart=/bin/sh -c 'echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable cpu-performance.service
sudo systemctl start cpu-performance.service
```

### Expand the swap for building ORB_SLAM3

```bash
sudo dphys-swapfile swapoff
sudo sed -i 's/CONF_SWAPSIZE=.*/CONF_SWAPSIZE=8192/' /etc/dphys-swapfile
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
# Verify:
free -h
```

---

## 4. Enable I2C and verify the cameras

### Verify I2C — ICM-20948 at 0x68

```bash
sudo apt install -y i2c-tools
sudo usermod -aG i2c $USER
# Log out and back in, then:
i2cdetect -y 1
```

Expected output (address `0x68` must appear):

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
...
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
```

If `0x68` is absent, reseat both camera ribbon cables — the IMU is powered through the camera connector.

### Verify the cameras with rpicam-apps

On Raspberry Pi OS Bookworm the camera tools are prefixed `rpicam-*`
(the older `libcamera-*` names were retired in Bookworm).

```bash
# List detected cameras — must show two IMX219 sensors
rpicam-hello --list-cameras
```

Expected output:

```
Available cameras
-----------------
0 : imx219 [3280x2464 10-bit RGGB] (/base/.../cam0)
1 : imx219 [3280x2464 10-bit RGGB] (/base/.../cam1)
```

Preview each camera individually (requires a connected monitor or VNC):

```bash
# Left camera (CAM0)
rpicam-hello --camera 0 -t 5000

# Right camera (CAM1)
rpicam-hello --camera 1 -t 5000
```

Capture a test still from each:

```bash
rpicam-jpeg --camera 0 -o /tmp/left_test.jpg
rpicam-jpeg --camera 1 -o /tmp/right_test.jpg
```

If only one camera is detected, reseat the ribbon cable on the failing port and reboot.

---

## 5. Install system packages

```bash
sudo apt install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    python3-picamera2 \
    python3-libcamera \
    rpicam-apps \
    i2c-tools \
    libi2c-dev \
    cmake \
    git \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libcamera \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxrender1 \
    libxext6 \
    build-essential \
    pkg-config \
    libssl-dev \
    awscli
```

> The `gstreamer1.0-libcamera` package bridges libcamera into GStreamer so that OpenCV can open camera streams via `cv2.VideoCapture` with a `libcamerasrc` pipeline — the RPi 5 equivalent of the Jetson's `nvarguscamerasrc`.

### Upgrade pip

```bash
python3 -m pip install --upgrade pip
```

---

## 6. Patch the camera pipeline for RPi 5

This is the only code change required. The Jetson Nano uses `nvarguscamerasrc` (NVIDIA-specific GStreamer element). The RPi 5 uses `libcamerasrc` instead.

Open `src/sensors/camera_imu_handler.py` and replace the `build_gstreamer_pipeline` function:

```python
# ── REPLACE this function (Jetson-specific) ──────────────────────────────
def build_gstreamer_pipeline(sensor_id, width, height, fps, flip_method):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        ...
    )

# ── WITH this RPi 5 version ───────────────────────────────────────────────
def build_gstreamer_pipeline(sensor_id, width, height, fps, flip_method):
    """libcamerasrc pipeline for Raspberry Pi 5 (replaces nvarguscamerasrc)."""
    flip = ""
    if flip_method == 2:          # rotate-180 equivalent
        flip = "! videoflip method=rotate-180 "
    return (
        f"libcamerasrc camera-name=cam{sensor_id} ! "
        f"video/x-raw, width={width}, height={height}, "
        f"framerate={fps}/1, format=RGBx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! "
        f"{flip}"
        f"appsink drop=1"
    )
```

> **Note:** `camera-name=cam0` and `camera-name=cam1` map to the physical CAM0 and CAM1 ports. If this does not work, run `libcamera-hello --list-cameras` to find the exact camera name strings and substitute them.

### Verify the patched pipeline with OpenCV

```bash
python3 - << 'EOF'
import cv2
pipe = (
    "libcamerasrc camera-name=cam0 ! "
    "video/x-raw, width=640, height=480, framerate=30/1, format=RGBx ! "
    "videoconvert ! video/x-raw, format=BGR ! appsink drop=1"
)
cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
ok, frame = cap.read()
print("Left camera OK:", ok, frame.shape if ok else "FAILED")
cap.release()
EOF
```

Both cameras must print `Left camera OK: True (480, 640, 3)`.

---

## 7. Install ORB_SLAM3

ORB_SLAM3 is optional — the pipeline runs in mock mode (synthetic figure-eight trajectory) if it is absent. Skip to section 8 for a quick cloud-pipeline test.

> **Performance note:** The RPi 5 has no GPU. ORB_SLAM3 runs fully on the four Cortex-A76 cores. Set `slam.skip_frames: 3` in the config to process every 3rd frame and maintain real-time throughput at 30 fps input.

### 7.1 Install build dependencies

```bash
sudo apt install -y \
    libboost-all-dev \
    libeigen3-dev \
    libopencv-dev
```

Pangolin must be built from source (not in apt for Bookworm):

```bash
sudo apt install -y libglew-dev libpython3-dev ffmpeg libavcodec-dev \
    libavutil-dev libavformat-dev libswscale-dev libavdevice-dev \
    libjpeg-dev libpng-dev libtiff5-dev libopenexr-dev

cd /tmp
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j2        # -j2 avoids OOM on 4 GB boards
sudo make install
sudo ldconfig
```

### 7.2 Build ORB_SLAM3 (CPU only — no CUDA flag needed)

```bash
sudo mkdir -p /opt/ORB_SLAM3
sudo chown $USER /opt/ORB_SLAM3

git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git /opt/ORB_SLAM3
cd /opt/ORB_SLAM3
chmod +x build.sh
./build.sh      # 40–60 minutes on RPi 5
```

> If the build runs out of memory, add `-j1` to the `make` calls inside `build.sh` and ensure the 8 GB swap is active (`free -h`).

### 7.3 Build Python bindings

```bash
cd /opt/ORB_SLAM3
pip3 install orbslam3   # community Python wrapper, if available
```

If bindings are unavailable the system falls back to mock mode automatically.

### 7.4 Verify vocabulary file

```bash
ls /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt
```

---

## 8. Clone the project

```bash
cd ~
git clone https://github.com/maoztamir/jetson-robot-slam.git jetson-robot-slam
cd jetson-robot-slam
```

---

## 9. Install Python dependencies

```bash
pip3 install -r requirements.txt
```

> Do **not** `pip install opencv-python` — `python3-opencv` installed in step 5 is compiled against the system libcamera stack. The PyPI wheel does not include the GStreamer libcamera plugin.

### Verify imports

```bash
python3 -c "import cv2, numpy, yaml, icm20948; print('OK')"
```

---

## 10. Configure the project

### 10.1 Create a local config

```bash
cp config/default_config.yaml config/local_config.yaml
```

Key values to adjust for RPi 5:

```yaml
camera:
  resolution: [640, 480]
  fps: 30
  flip_method: 0         # 0=none, 2=rotate-180 if image is upside-down

slam:
  vocab_path: /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt
  settings_path: config/stereo_imu_settings.yaml
  skip_frames: 3         # RPi 5 is CPU-only — process every 3rd frame

aws:
  enabled: true
  endpoint: <YOUR_IOT_ENDPOINT>.iot.us-east-2.amazonaws.com
  thing_name: <YOUR_THING_NAME>          # e.g. rpi5-robot
  client_id: basicPubSub
  certificate: certs/<thing>.cert.pem
  private_key: certs/<thing>.private.key
  publish_interval: 5

logging:
  level: INFO
  file: logs/robot.log
```

> `skip_frames: 3` is a good starting point. Reduce it if the RPi 5 keeps up (monitor with `--verbose`); increase it if tracking is lost frequently.

### 10.2 Stereo camera calibration

The shipped `config/stereo_imu_settings.yaml` has placeholder intrinsics. For accurate SLAM, replace them with real calibration values for the IMX219-83 at your chosen resolution:

1. Print an 11×8 checkerboard (30 mm squares).
2. Capture 20–30 stereo pairs using the patched `camera_imu_handler`:
   ```bash
   python3 -m src.sensors.camera_imu_handler
   ```
3. Run OpenCV `stereoCalibrate` or Kalibr on the collected pairs.
4. Update `Camera1.*`, `Camera2.*`, and `Stereo.T_c1_c2` in `stereo_imu_settings.yaml`.

> The IMX219-83 baseline is 83 mm. Set `Stereo.b: 0.083` and `Camera.bf: <fx * 0.083>` as a starting approximation before full calibration.

---

## 11. AWS IoT setup

Skip this section to run locally with `--no-aws`.

### 11.1 Deploy the CloudFormation stack

```bash
aws cloudformation deploy \
  --template-file aws/cloudformation/robot-infrastructure.yaml \
  --stack-name jetson-robot-slam \
  --capabilities CAPABILITY_IAM \
  --region us-east-2 \
  --parameter-overrides \
      ThingName=rpi5-robot \
      OriginLat=32.0853 \
      OriginLon=34.7818
```

This creates the same infrastructure as the Jetson setup: DynamoDB table, IoT Thing, Lambda, Location Service tracker, and Cognito Identity Pool.

### 11.2 Download device certificates

AWS Console → IoT Core → Things → `rpi5-robot` → Certificates:

1. Create a certificate (one-click) and download all three files.
2. Activate the certificate and attach the IoT policy created by CloudFormation.

### 11.3 Place certificates on the RPi 5

```bash
mkdir -p ~/jetson-robot-slam/certs
mv ~/Downloads/*.pem.crt  certs/rpi5-robot.cert.pem
mv ~/Downloads/*.private.key certs/rpi5-robot.private.key
mv ~/Downloads/AmazonRootCA1.pem certs/AmazonRootCA1.pem
chmod 600 certs/*.pem certs/*.key
```

Update `config/local_config.yaml`:

```yaml
aws:
  certificate: certs/rpi5-robot.cert.pem
  private_key:  certs/rpi5-robot.private.key
```

### 11.4 Find your IoT endpoint

```bash
aws iot describe-endpoint --endpoint-type iot:Data-ATS \
  --region us-east-2 --query endpointAddress --output text
```

Paste the result into `aws.endpoint` in the config.

### 11.5 Deploy the Lambda function

```bash
bash scripts/deploy_lambda.sh
```

---

## 12. Verify everything works

### Check 1 — Mock pipeline (no hardware needed)

```bash
cd ~/jetson-robot-slam
python3 -m src.main --no-aws --verbose
```

Expected output:

```
INFO  root: Config loaded from config/local_config.yaml
INFO  root: Camera mock mode active
INFO  root: IMU mock mode active
INFO  root: ORB_SLAM3 mock mode -- synthetic figure-eight trajectory
INFO  root: AWS publishing disabled (--no-aws)
INFO  root: Pipeline running. Press Ctrl+C to stop.
```

### Check 2 — Camera capture

```bash
python3 -m src.sensors.camera_imu_handler
```

Look for:

```
INFO  camera_imu_handler: Left frame 640x480, right frame 640x480
INFO  camera_imu_handler: IMU accel=(0.01, 0.02, 9.81) ...
```

If both cameras fall back to mock mode, the `libcamerasrc` pipeline is not working — check section 6 and rerun the OpenCV pipeline test.

### Check 3 — IMU

```bash
i2cdetect -y 1   # must show 0x68
python3 -c "
import icm20948
imu = icm20948.ICM20948()
ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
print(f'accel=({ax:.3f}, {ay:.3f}, {az:.3f})  gyro=({gx:.3f}, {gy:.3f}, {gz:.3f})')
"
```

`az` should be approximately `9.81` when the board is resting flat.

### Check 4 — AWS connectivity

```bash
python3 -m src.main --verbose 2>&1 | grep -E "Connected|mock|ERROR"
```

Expected:

```
INFO  aws_iot_publisher: Connected to AWS IoT Core at <endpoint> as 'rpi5-robot'
```

### Check 5 — DynamoDB records

Run the full pipeline for 30+ seconds, then:

```bash
aws dynamodb query \
  --table-name RobotTrajectory \
  --key-condition-expression "device_id = :d" \
  --expression-attribute-values '{":d":{"S":"rpi5-robot"}}' \
  --region us-east-2 \
  --query 'Count'
```

### Run the unit tests

```bash
pytest tests/ -v
```

---

## 13. Running the pipeline

```bash
# Full pipeline — SLAM + AWS
python3 -m src.main --config config/local_config.yaml

# SLAM only — no network required
python3 -m src.main --config config/local_config.yaml --no-aws

# Live stereo view (requires display or VNC)
python3 -m src.main --config config/local_config.yaml --visualize

# Verbose performance logging
python3 -m src.main --config config/local_config.yaml --verbose
```

Press `Ctrl+C` to stop. On shutdown the trajectory is saved to `trajectory.txt` (TUM format).

### Local web dashboard

```bash
python3 scripts/local_server.py --telemetry telemetry/telemetry.jsonl
# Open http://<rpi5-ip>:8080
```

### Trajectory viewer (run on a desktop machine)

```bash
python3 scripts/trajectory_viewer.py --region us-east-2 --table RobotTrajectory
```

---

## 14. Autostart on boot (optional)

```bash
sudo tee /etc/systemd/system/robot-slam.service > /dev/null << 'EOF'
[Unit]
Description=Robot SLAM pipeline
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/jetson-robot-slam
ExecStart=/usr/bin/python3 -m src.main --config config/local_config.yaml
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable robot-slam.service
sudo systemctl start robot-slam.service

# Monitor
sudo systemctl status robot-slam.service
journalctl -u robot-slam.service -f
```

> Change `User=pi` to your actual username if different.

---

## 15. Troubleshooting

### Camera not detected by libcamera

```
Available cameras: none
```

1. Confirm the ribbon cables are fully seated — the metal contacts must engage, blue strip facing the USB ports.
2. Check you have the **22-to-15-pin FFC adapter** cables. Standard 15-pin cables from RPi 4 kits are the wrong size for the RPi 5.
3. Run `dmesg | grep imx219` after boot — you should see two entries.
4. Re-enable the camera in `sudo raspi-config` → Interface Options → Camera.
5. Reboot after any change.

### libcamerasrc not found in GStreamer

```
No such element or plugin 'libcamerasrc'
```

Install the missing package:

```bash
sudo apt install -y gstreamer1.0-libcamera
```

Then verify:

```bash
gst-inspect-1.0 libcamerasrc
```

### Only one camera appears in `libcamera-hello --list-cameras`

- One ribbon cable may be backwards. Try flipping the insertion direction.
- Check both CAM0 and CAM1 port latches are fully closed.

### IMU not found (0x68 missing from i2cdetect)

- I2C must be enabled: `sudo raspi-config` → Interface Options → I2C → Enable.
- The IMU is powered through the camera ribbon. A loose camera cable will also kill the IMU.
- Verify the user is in the i2c group: `groups $USER | grep i2c`.

### ORB_SLAM3 build fails / runs out of memory

- Ensure 8 GB swap is active: `free -h` (should show ~8G swap).
- Edit `build.sh` and change `make -j$(nproc)` to `make -j1` — slower but memory-safe.
- Check available disk space: `df -h` — the build needs ~4 GB free.

### SLAM tracking lost immediately / very low FPS

The RPi 5 is CPU-only — this is the most common issue when comparing to the Jetson Nano.

1. Increase `slam.skip_frames` to `4` or `5` in the config.
2. Confirm the performance CPU governor is active:
   ```bash
   cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
   # must output: performance
   ```
3. Reduce `camera.resolution` to `[480, 320]` (update calibration intrinsics accordingly).
4. Close all other applications — the desktop GUI consumes significant CPU.
   Consider switching to the headless Lite image and connecting via SSH for production deployments.

### AWS publish fails

```
ERROR  aws_iot_publisher: Certificate file missing
```

- Confirm all three certificate files are in `certs/` with correct names.
- Check paths in `config/local_config.yaml` match the actual filenames.
- Verify the certificate is **activated** in the AWS Console and the IoT policy is attached.

### DynamoDB returns no data in the trajectory viewer

- The `timestamp` sort key is the SLAM monotonic clock, not a Unix timestamp. The trajectory viewer filters by `ingested_at` (Unix ms, stamped by the IoT rule).
- Ensure the selected date/time range in the viewer matches when the robot was actually running.
- Verify records exist: `aws dynamodb scan --table-name RobotTrajectory --max-items 1 --region us-east-2`

### Comparing RPi 5 vs Jetson Nano performance

| Metric | Jetson Nano 4 GB | Raspberry Pi 5 8 GB |
|---|---|---|
| CPU | Quad A57 @ 1.43 GHz | Quad A76 @ 2.4 GHz |
| GPU compute | CUDA 128 cores | None |
| ORB_SLAM3 FPS (640×480) | ~25–30 fps | ~8–12 fps (skip_frames=3) |
| ORB_SLAM3 build time | 20–40 min | 40–60 min |
| Camera interface | nvarguscamerasrc | libcamerasrc |
| Power draw (full load) | ~5 W | ~8 W |

The RPi 5 is faster per core but lacks GPU-accelerated feature extraction. Expect roughly 3× higher skip_frames requirement compared to the Jetson Nano to maintain real-time operation.
