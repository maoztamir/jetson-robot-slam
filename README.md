# Jetson Robot SLAM

Stereo-inertial SLAM pipeline for the Jetson Nano using ORB_SLAM3, an IMX219-83 stereo camera, and AWS IoT Core for cloud telemetry.

## Hardware Requirements

- **Jetson Nano** (4 GB recommended)
- **IMX219-83 stereo camera module** connected via dual CSI (sensor-id 0 = left, sensor-id 1 = right)
- **ICM-20948 9-DoF IMU** on the camera module PCB (accessed via Waveshare driver)

## Prerequisites

### 1. JetPack OS

Install [NVIDIA JetPack](https://developer.nvidia.com/embedded/jetpack) (4.6+ recommended). JetPack provides CUDA, GStreamer, and `nvarguscamerasrc` which are required for hardware-accelerated camera capture.

### 2. Python 3.6+

JetPack ships with Python 3.6. Verify:

```bash
python3 --version
```

### 3. System packages

```bash
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-opencv \
    i2c-tools \
    libi2c-dev
```

### 4. Python dependencies

```bash
pip3 install numpy pyyaml icm20948
```

For AWS IoT publishing (optional):

```bash
pip3 install awsiotsdk
```

### 5. ORB_SLAM3 (optional)

For real SLAM processing, build and install ORB_SLAM3 with Python bindings:

```bash
# Clone and build ORB_SLAM3 to /opt/ORB_SLAM3
# See: https://github.com/UZ-SLAMLab/ORB_SLAM3
```

The vocabulary file is expected at `/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt`. You can change this path in `config/default_config.yaml`.

If ORB_SLAM3 is not installed, the pipeline automatically falls back to a **mock backend** that generates a synthetic figure-eight trajectory. This lets you test the full pipeline without the SLAM library.

## Project Structure

```
jetson-robot-slam/
├── config/
│   ├── default_config.yaml         # Main configuration file
│   └── stereo_imu_settings.yaml    # ORB_SLAM3 camera/IMU calibration
├── src/
│   ├── main.py                     # Entry point
│   ├── sensors/
│   │   └── camera_imu_handler.py   # Stereo camera + IMU capture
│   ├── slam/
│   │   └── orb_slam3_wrapper.py    # ORB_SLAM3 wrapper (with mock fallback)
│   └── cloud/
│       └── aws_iot_publisher.py    # AWS IoT Core MQTT publisher
├── aws/
│   ├── cloudformation/             # AWS infrastructure templates
│   └── lambda/                     # Lambda functions for trajectory processing
├── certs/                          # AWS IoT certificates (not committed)
└── logs/                           # Rotating log files (created at runtime)
```

## Setup

### 1. Clone the repository

```bash
git clone https://github.com/maoztamir/jetson-robot-slam.git ~/jetson-robot-slam
cd ~/jetson-robot-slam
```

### 2. Verify the camera

Confirm both CSI sensors are detected:

```bash
# Left camera
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! fakesink

# Right camera
gst-launch-1.0 nvarguscamerasrc sensor-id=1 ! fakesink
```

If either fails, check the ribbon cable connections.

### 3. Verify the IMU (ICM-20948)

The IMX219-83 module uses an ICM-20948 9-DoF IMU accessed via the Waveshare driver:

```bash
wget https://files.waveshare.com/upload/e/eb/D219-9dof.tar.gz
tar zxvf D219-9dof.tar.gz
cd D219-9dof/07-icm20948-demo
make
./ICM20948-Demo
```

You should see live accelerometer, gyroscope, and magnetometer readings. If not, check the CSI connection to the camera module.

### 4. Configure

Copy and edit the config for your environment:

```bash
cp config/default_config.yaml config/local_config.yaml
```

Key settings in the config:

| Section    | Key              | Default                          | Description                        |
|------------|------------------|----------------------------------|------------------------------------|
| `camera`   | `resolution`     | `[640, 480]`                     | Capture resolution                 |
| `camera`   | `fps`            | `30`                             | Frame rate                         |
| `camera`   | `flip_method`    | `0`                              | `0` = none, `2` = rotate 180      |
| `slam`     | `vocab_path`     | `/opt/ORB_SLAM3/Vocabulary/...`  | ORB vocabulary file                |
| `slam`     | `skip_frames`    | `1`                              | Process every Nth frame            |
| `aws`      | `enabled`        | `true`                           | Enable/disable cloud publishing    |
| `aws`      | `endpoint`       | *(must be set)*                  | Your IoT Core data endpoint        |

### 5. Camera calibration

The file `config/stereo_imu_settings.yaml` contains placeholder camera intrinsics for the IMX219-83 at 640x480. For accurate SLAM you should replace these with values from your own stereo calibration using OpenCV `stereoCalibrate`, Kalibr, or MATLAB.

### 6. AWS IoT setup (optional)

If you want to publish trajectory data to AWS IoT Core:

1. Create an IoT Thing in the AWS console (or use the CloudFormation template in `aws/cloudformation/`).
2. Download the device certificate, private key, and Amazon Root CA.
3. Place them in the `certs/` directory:
   ```
   certs/
   ├── certificate.pem.crt
   ├── private.pem.key
   └── AmazonRootCA1.pem
   ```
4. Set your IoT endpoint in the config:
   ```yaml
   aws:
     endpoint: your-endpoint.iot.us-east-1.amazonaws.com
   ```

If AWS is not configured, the publisher runs in mock mode and logs messages locally.

## Running

From the project root:

```bash
# Full pipeline (SLAM + AWS publishing)
python3 -m src.main

# SLAM only, no AWS
python3 -m src.main --no-aws

# With live stereo camera feed (requires a display)
python3 -m src.main --visualize

# Debug logging
python3 -m src.main --verbose

# Custom config
python3 -m src.main --config config/local_config.yaml

# Combined
python3 -m src.main --no-aws --visualize --verbose
```

Press `Ctrl+C` to stop gracefully. On shutdown the trajectory is saved to `trajectory.txt` in TUM format.

### Quick test without hardware

You can run the full pipeline on any machine. Without the camera, IMU, ORB_SLAM3 bindings, or AWS certs, each component falls back to its mock mode:

```bash
python3 -m src.main --no-aws --verbose
```

You can also test individual components:

```bash
# Camera + IMU capture (10-second test)
python3 -m src.sensors.camera_imu_handler

# SLAM mock trajectory (300 frames)
python3 -m src.slam.orb_slam3_wrapper

# AWS publisher mock test
python3 -m src.cloud.aws_iot_publisher
```

## Output

- **Trajectory file** -- `trajectory.txt` in TUM format (`timestamp tx ty tz qx qy qz qw`), saved on shutdown.
- **Logs** -- `logs/robot.log` with rotation (10 MB per file, 5 backups).
- **AWS IoT** -- poses published to MQTT topic `robot/<thing_name>/trajectory` as JSON.
- **Performance stats** -- logged every 30 seconds with FPS, SLAM success rate, queue sizes, and memory usage.

## License

MIT
