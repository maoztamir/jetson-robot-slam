# Jetson Nano — Setup & Installation Guide

Complete instructions for provisioning a new Jetson Nano to run the stereo-inertial SLAM pipeline with AWS IoT cloud integration.

---

## Table of Contents

1. [Hardware](#1-hardware)
2. [Flash JetPack OS](#2-flash-jetpack-os)
3. [First-boot configuration](#3-first-boot-configuration)
4. [Enable I2C and verify the camera](#4-enable-i2c-and-verify-the-camera)
5. [Install system packages](#5-install-system-packages)
6. [Install ORB_SLAM3](#6-install-orb_slam3)
7. [Clone the project](#7-clone-the-project)
8. [Install Python dependencies](#8-install-python-dependencies)
9. [Configure the project](#9-configure-the-project)
10. [AWS IoT setup](#10-aws-iot-setup)
11. [Verify everything works](#11-verify-everything-works)
12. [Running the pipeline](#12-running-the-pipeline)
13. [Autostart on boot (optional)](#13-autostart-on-boot-optional)
14. [Troubleshooting](#14-troubleshooting)

---

## 1. Hardware

| Item | Model / Spec |
|---|---|
| SBC | NVIDIA Jetson Nano **4 GB** |
| Camera | Waveshare **IMX219-83** stereo CSI module |
| IMU | **ICM-20948** 9-DoF (soldered on the IMX219-83 PCB) |
| Power | 5 V / 4 A barrel-jack supply (barrel mode recommended over USB) |
| Storage | 32 GB+ microSD card (Class 10 / A2 speed) |
| Network | Ethernet or compatible Wi-Fi adapter |

### Physical connections

- Plug the **left** camera ribbon into CSI0 (`sensor-id=0`).
- Plug the **right** camera ribbon into CSI1 (`sensor-id=1`).
- The ICM-20948 IMU on the camera PCB connects automatically via the ribbon cable (uses I2C bus 1, address `0x68`).
- Use the barrel-jack 5 V supply — USB power is insufficient for sustained SLAM workloads.

---

## 2. Flash JetPack OS

1. Download **JetPack 4.6.x** (L4T 32.7.x) from the NVIDIA developer portal:
   `https://developer.nvidia.com/embedded/jetpack`

2. Flash the image to the microSD card using **Balena Etcher** or `dd`:

   ```bash
   # Using Etcher (cross-platform GUI)
   # Select the .img file and target SD card, then Flash.

   # Or using dd on Linux (replace /dev/sdX with your SD device):
   sudo dd if=jetson-nano-jp46-sd-card-image.img of=/dev/sdX bs=1M status=progress
   sync
   ```

3. Insert the card into the Jetson, connect a monitor, keyboard, and power on.

4. Complete the Ubuntu 18.04 first-boot wizard (locale, user account, timezone, etc.).

> **Note:** JetPack 4.6 includes CUDA 10.2, GStreamer 1.14, `nvarguscamerasrc`, and Python 3.6. All of these are required by this project.

---

## 3. First-boot configuration

After completing the wizard, run the following to update the system:

```bash
sudo apt-get update && sudo apt-get upgrade -y
```

### Set the power mode to maximum performance

```bash
sudo nvpmodel -m 0    # MAXN mode (all 4 cores, full GPU)
sudo jetson_clocks    # lock clocks to maximum
```

To make the clock lock persistent across reboots:

```bash
# Add to /etc/rc.local before the exit 0 line:
sudo sh -c 'echo "/usr/bin/jetson_clocks" >> /etc/rc.local'
```

### Expand the swap (recommended for building ORB_SLAM3)

```bash
sudo fallocate -l 8G /var/swapfile
sudo chmod 600 /var/swapfile
sudo mkswap /var/swapfile
sudo swapon /var/swapfile
# Make persistent:
echo '/var/swapfile swap swap defaults 0 0' | sudo tee -a /etc/fstab
```

---

## 4. Enable I2C and verify the camera

### Enable I2C

```bash
sudo usermod -aG i2c $USER
# Log out and back in, then verify:
i2cdetect -y -r 1
```

You should see address `0x68` in the output — that is the ICM-20948 IMU.

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
...
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
```

If `0x68` is not visible, check the camera ribbon cable connections.

### Verify the cameras

Each CSI sensor must be detected individually:

```bash
# Left camera (sensor-id=0)
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=10 ! fakesink -v

# Right camera (sensor-id=1)
gst-launch-1.0 nvarguscamerasrc sensor-id=1 num-buffers=10 ! fakesink -v
```

Both commands must complete without error. If a sensor fails, reseat the ribbon cable on that CSI port.

To visually preview the cameras:

```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
  nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! autovideosink
```

---

## 5. Install system packages

```bash
sudo apt-get install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    i2c-tools \
    libi2c-dev \
    cmake \
    git \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxrender1 \
    libxext6 \
    build-essential \
    pkg-config \
    libssl-dev \
    awscli
```

> `awscli` is needed for the Lambda deployment script.

### Upgrade pip

```bash
python3 -m pip install --upgrade pip
```

---

## 6. Install ORB_SLAM3

ORB_SLAM3 is optional — the pipeline falls back to a synthetic figure-eight trajectory if it is not installed. Skip this section for a quick test setup; install it for real SLAM operation.

### 6.1 Install ORB_SLAM3 build dependencies

```bash
sudo apt-get install -y \
    libboost-all-dev \
    libeigen3-dev \
    libopencv-dev \
    libpangolin-dev
```

Pangolin must be built from source on JetPack 4.6:

```bash
cd /tmp
git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 6.2 Build ORB_SLAM3

```bash
sudo mkdir -p /opt/ORB_SLAM3
sudo chown $USER /opt/ORB_SLAM3

git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git /opt/ORB_SLAM3
cd /opt/ORB_SLAM3
chmod +x build.sh
./build.sh
```

The build takes 20–40 minutes on the Jetson Nano.

### 6.3 Build Python bindings

```bash
cd /opt/ORB_SLAM3
# If a python-bindings branch or wrapper exists in the repo, follow its README.
# Alternatively install a community Python wrapper:
pip3 install orbslam3   # if available for your Python version
```

> If Python bindings cannot be installed the system automatically runs in **mock mode** — all other features (AWS publishing, trajectory viewer) still work.

### 6.4 Verify vocabulary file

```bash
ls /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt
```

The file must exist at this path (configurable in `config/default_config.yaml`).

---

## 7. Clone the project

```bash
cd ~
git clone https://github.com/maoztamir/jetson-robot-slam.git jetson-robot-slam
cd jetson-robot-slam
```

---

## 8. Install Python dependencies

### Minimal install (Jetson — runs the SLAM pipeline)

```bash
pip3 install -r requirements.txt
```

Contents of `requirements.txt`:

```
numpy
pyyaml
icm20948          # ICM-20948 IMU driver
smbus2            # I2C interface
awsiotsdk         # AWS IoT Device SDK v2 (optional — mock mode if absent)
boto3             # DynamoDB / Lambda (optional)
matplotlib        # trajectory_viewer.py (desktop only)
pytest            # tests
```

> `opencv-python` is already provided system-wide by `python3-opencv` installed in step 5. Do **not** install it again via pip on the Jetson — the JetPack build includes CUDA acceleration that the PyPI wheel lacks.

### Verify imports

```bash
python3 -c "import cv2, numpy, yaml, icm20948; print('OK')"
```

---

## 9. Configure the project

### 9.1 Create a local config

```bash
cp config/default_config.yaml config/local_config.yaml
```

Open `config/local_config.yaml` and adjust the key values:

```yaml
camera:
  resolution: [640, 480]   # capture size sent to ORB_SLAM3
  fps: 30
  flip_method: 0           # 0=none, 2=rotate-180 if image is upside-down

slam:
  vocab_path: /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt
  settings_path: config/stereo_imu_settings.yaml
  skip_frames: 1           # increase to 2–3 if Jetson cannot keep up

aws:
  enabled: true
  endpoint: <YOUR_IOT_ENDPOINT>.iot.us-east-2.amazonaws.com
  thing_name: <YOUR_THING_NAME>          # e.g. tele2-jetson
  client_id: basicPubSub
  certificate: certs/<thing>.cert.pem
  private_key: certs/<thing>.private.key
  publish_interval: 5      # publish every 5th pose to reduce bandwidth

logging:
  level: INFO
  file: logs/robot.log
```

### 9.2 Stereo camera calibration

The file `config/stereo_imu_settings.yaml` ships with **placeholder** intrinsics for the IMX219-83 at 640×480. These work for smoke-testing but will produce drift in real operation. Replace them with values from your own calibration:

1. Print a calibration checkerboard (11×8, 30 mm squares recommended).
2. Capture 20–30 stereo image pairs with the cameras held at different angles:
   ```bash
   python3 -m src.sensors.camera_imu_handler   # press 's' to save frames
   ```
3. Run OpenCV `stereoCalibrate`, Kalibr, or the MATLAB stereo calibration app.
4. Update these keys in `stereo_imu_settings.yaml`:

   ```yaml
   Camera1.fx, Camera1.fy, Camera1.cx, Camera1.cy  # left intrinsics
   Camera1.k1, Camera1.k2, Camera1.p1, Camera1.p2  # left distortion
   Camera2.*                                         # right intrinsics
   Stereo.T_c1_c2                                   # right-to-left transform
   ```

---

## 10. AWS IoT setup

Skip this section if you only want to run locally (`--no-aws` flag).

### 10.1 Deploy the CloudFormation stack

The full AWS infrastructure (IoT Thing, DynamoDB table, Lambda, Location Service, Cognito Identity Pool) is defined in a single CloudFormation template.

```bash
aws cloudformation deploy \
  --template-file aws/cloudformation/robot-infrastructure.yaml \
  --stack-name jetson-robot-slam \
  --capabilities CAPABILITY_IAM \
  --region us-east-2 \
  --parameter-overrides \
      ThingName=tele2-jetson \
      OriginLat=32.0853 \
      OriginLon=34.7818
```

This creates:

| Resource | Purpose |
|---|---|
| IoT Thing + Policy | Authenticates the Jetson to AWS IoT Core |
| DynamoDB `RobotTrajectory` | Stores every published pose (partition key: `device_id`, sort key: `timestamp`) |
| IoT Rule | Routes MQTT messages → DynamoDB |
| Lambda `process_trajectory` | DynamoDB Stream → AWS Location Service |
| Location Service Tracker | Stores GPS positions for device history |
| Cognito Identity Pool | Allows the web dashboard to query DynamoDB from the browser |

### 10.2 Download device certificates

In the AWS Console → IoT Core → Things → `<your-thing>` → Certificates:

1. Create a certificate (one-click).
2. Download:
   - Device certificate (`*.pem.crt`)
   - Private key (`*.private.key`)
   - Amazon Root CA 1 (`AmazonRootCA1.pem`)
3. Activate the certificate.
4. Attach the IoT policy created by CloudFormation.

### 10.3 Place certificates on the Jetson

```bash
mkdir -p ~/jetson-robot-slam/certs
# Copy the three downloaded files into the certs/ directory.
# Rename them to match the config:
mv ~/Downloads/*.pem.crt  certs/tele2-jetson.cert.pem
mv ~/Downloads/*.private.key certs/tele2-jetson.private.key
mv ~/Downloads/AmazonRootCA1.pem certs/AmazonRootCA1.pem
chmod 600 certs/*.pem certs/*.key
```

### 10.4 Find your IoT endpoint

```bash
aws iot describe-endpoint --endpoint-type iot:Data-ATS \
  --region us-east-2 --query endpointAddress --output text
```

Copy the output (e.g. `aw4tyjjeoxel4-ats.iot.us-east-2.amazonaws.com`) into `config/local_config.yaml` under `aws.endpoint`.

### 10.5 Deploy the Lambda function

After the CloudFormation stack is up, push the actual Lambda code:

```bash
bash scripts/deploy_lambda.sh
```

This packages `aws/lambda/process_trajectory.py` and calls
`aws lambda update-function-code` to replace the placeholder code that
CloudFormation uploaded.

---

## 11. Verify everything works

Run through these checks in order.

### Check 1 — Mock pipeline (no hardware needed)

```bash
cd ~/jetson-robot-slam
python3 -m src.main --no-aws --verbose
```

Expected output (first few seconds):

```
INFO  root: Config loaded from config/default_config.yaml
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

Look for lines like:

```
INFO  camera_imu_handler: Left frame 640x480, right frame 640x480
INFO  camera_imu_handler: IMU accel=(0.01, 0.02, 9.81) gyro=(0.00, 0.00, 0.00)
```

If the camera falls back to mock mode (`Camera mock mode active`) the GStreamer pipeline is not working — recheck the CSI cables.

### Check 3 — IMU

```bash
i2cdetect -y -r 1   # must show 0x68
python3 -c "
import icm20948
imu = icm20948.ICM20948()
ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
print(f'accel=({ax:.3f}, {ay:.3f}, {az:.3f})  gyro=({gx:.3f}, {gy:.3f}, {gz:.3f})')
"
```

`az` should be approximately `9.81` (gravity) when the board is flat.

### Check 4 — AWS connectivity

```bash
python3 -m src.main --verbose 2>&1 | grep -E "Connected|mock|ERROR"
```

Expected when certificates are correctly configured:

```
INFO  aws_iot_publisher: Connected to AWS IoT Core at <endpoint> as 'tele2-jetson'
```

If you see `running in mock mode` check the certificate paths and endpoint in the config.

### Check 5 — DynamoDB data

After running the full pipeline for 30+ seconds, verify records appear in DynamoDB:

```bash
aws dynamodb query \
  --table-name RobotTrajectory \
  --key-condition-expression "device_id = :d" \
  --expression-attribute-values '{":d":{"S":"tele2-jetson"}}' \
  --region us-east-2 \
  --query 'Count'
```

The count should increase each time you run the pipeline.

### Run the unit tests

```bash
pip3 install pytest
pytest tests/ -v
```

All tests should pass. Tests use mock AWS clients and do not require real credentials.

---

## 12. Running the pipeline

From the project root:

```bash
# Full pipeline — SLAM + AWS cloud publishing
python3 -m src.main --config config/local_config.yaml

# SLAM only — no network required
python3 -m src.main --config config/local_config.yaml --no-aws

# Show live stereo camera feed (requires a connected monitor or VNC)
python3 -m src.main --config config/local_config.yaml --visualize

# Verbose debug logging
python3 -m src.main --config config/local_config.yaml --verbose

# Write poses to a specific local file
python3 -m src.main --telemetry-file /tmp/my_run.jsonl
```

Press `Ctrl+C` to stop gracefully. On shutdown:

- The trajectory is written to `trajectory.txt` in TUM format (`timestamp tx ty tz qx qy qz qw`).
- Rotating logs are in `logs/robot.log` (10 MB per file, 5 backups).

### Local web dashboard (no AWS required)

```bash
python3 scripts/local_server.py --telemetry telemetry/telemetry.jsonl
# Open http://<jetson-ip>:8080 in a browser
```

### Trajectory viewer (desktop machine)

```bash
# Requires boto3 and matplotlib; AWS credentials must be configured
python3 scripts/trajectory_viewer.py --region us-east-2 --table RobotTrajectory
```

---

## 13. Autostart on boot (optional)

Run the installer script once to register a systemd service that starts the pipeline on every boot:

```bash
bash scripts/install_service.sh
```

Then start it immediately without rebooting:

```bash
sudo systemctl start robot-slam
sudo systemctl status robot-slam
journalctl -u robot-slam -f
```

See [docs/autostart.md](autostart.md) for full details, log locations, and troubleshooting.

---

## 14. Troubleshooting

### Camera not detected

```
ERROR  camera_imu_handler: GStreamer pipeline failed for sensor-id=0
```

1. Power cycle the Jetson with the camera connected.
2. Check the ribbon cable is fully inserted and the latch is closed on both ends.
3. Confirm `nvarguscamerasrc` works: `gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=5 ! fakesink`
4. If it still fails, try swapping the two ribbon cables (left ↔ right).

### IMU not found (`0x68` missing from i2cdetect)

1. The IMU is on the camera module PCB — if the camera ribbon is loose the IMU will also be unreachable.
2. Reseat both CSI ribbon cables.
3. Check that the user is in the `i2c` group: `groups $USER | grep i2c`

### ORB_SLAM3 build fails

- The most common cause is insufficient RAM. Ensure the 8 GB swap file is active (`free -h`) and build with `-j2` instead of `-j$(nproc)`.
- Eigen version conflicts: JetPack 4.6 ships Eigen 3.3.7 — do not install a newer version system-wide.

### AWS publish fails (mock mode fallback)

```
ERROR  aws_iot_publisher: Certificate file missing: certs/tele2-jetson.cert.pem
```

- Verify all three certificate files exist in `certs/`.
- Check that `aws.certificate` and `aws.private_key` paths in the config match the actual filenames.
- Check that the certificate is **activated** and has the IoT policy attached in the AWS Console.

### DynamoDB returns no data in the trajectory viewer

- The `timestamp` sort key in DynamoDB is the **SLAM monotonic clock** (a small float, not a Unix timestamp). The trajectory viewer filters by `ingested_at` (Unix milliseconds added by the IoT rule). Ensure the selected time range matches when the robot actually ran.
- Verify `ingested_at` exists in the records: `aws dynamodb scan --table-name RobotTrajectory --max-items 1 --region us-east-2`

### Low SLAM frame rate / tracking lost

1. Increase `slam.skip_frames` to `2` or `3` in the config (process every 2nd or 3rd frame).
2. Ensure MAXN power mode is active: `sudo nvpmodel -q`.
3. Reduce `camera.resolution` to `[480, 320]` to reduce processing time (recalibrate intrinsics after changing resolution).

### Pipeline crashes with `Segmentation fault`

This is most often caused by a mismatch between the ORB_SLAM3 build and the settings file. Try:

```bash
python3 -m src.main --no-aws --verbose   # run in mock SLAM mode to isolate
```

If mock mode works but real ORB_SLAM3 crashes, rebuild ORB_SLAM3 from scratch with a clean build directory.
