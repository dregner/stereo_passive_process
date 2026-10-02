# passive_stereo_capture

A ROS 2 node that acquires synchronized stereo frames from two FLIR Blackfly S (BFS) color cameras via the Spinnaker SDK and feeds them to three parallel processing workers: **ORB-SLAM3** (localization), **Retinify** (GPU dense stereo / depth), and a **JPEG preview streamer**. All camera-to-worker data paths are direct C++ calls — ROS 2 is used only for output topics and parameter management, eliminating the serialization overhead of image topics on the hot path.

Designed to run on a **Jetson Orin Nano Super (JetPack 6.2, aarch64)** with cameras connected via USB 3.0 or GigE.

---

## Architecture

```
Cameras (BayerRG8 raw, Spinnaker SDK)
         │
         │  grabThread × 2  ──  syncThread (FrameID match)
         │  (raw BayerRG8, CV_8UC1 — no conversion in grabber)
         ▼
    preprocess queue
         │
    preprocessThread
      ├─ SLAM path ──────────── BayerRG2GRAY → rectify
      │                         → StereoFrame::left_gray / right_gray
      │
      └─ Disparity + Preview ── BayerRG2RGB → rectify → CLAHE (CIE Lab L)
                                → StereoFrame::left_rgb / right_rgb
                                  (one debayer, two consumers)
         │              │                │
    SlamWorker    DisparityWorker   PreviewWorker
    ORB-SLAM3     Retinify GPU      JPEG compress
    (gray,        (RGB+CLAHE)       (reuses left_rgb)
     resize,
     CLAHE gray)
         │              │                │
    /Passive/      /Passive/        /Passive/left/
    slam/…         disparity/       preview/image/
                   pointcloud       compressed
```

### Key design decisions

| Decision | Rationale |
|---|---|
| Raw BayerRG8 in grabber | Defer conversion cost; each consumer chooses its format |
| `BayerRG2GRAY` for SLAM | Single-step, no intermediate RGB; ~3× cheaper than full debayer |
| `BayerRG2RGB` + CLAHE shared by Retinify and Preview | One debayer operation serves two consumers |
| CLAHE in CIE Lab L-channel (full-res) | Improves contrast for depth estimation without hue shift |
| Grayscale CLAHE on resized SLAM image | Applied post-scale on a small image — negligible cost |
| Hardware GPIO trigger (libgpiod PWM) | Guarantees both cameras fire on the same pulse → zero-skew synchronization |
| Preprocess thread decouples sync thread | Sync thread returns immediately; debayer/rectify/CLAHE never block frame acquisition |
| Drop-oldest queues for Disparity / Preview | Always process the newest frame; skip if behind |
| Drop-newest queue (depth 2) for SLAM | SLAM must not skip frames to maintain tracking continuity |
| `std::unique_ptr<ORB_SLAM3::System, SlamDeleter>` | Exception-safe; destructor calls `Shutdown()` cleanly |

---

## Dependencies

### Required on both build machine and Jetson

| Library | Version | Notes |
|---|---|---|
| ROS 2 | Jazzy | `ros-jazzy-desktop` or `ros-jazzy-ros-base` |
| Spinnaker SDK | 4.x | Install to `/opt/spinnaker/` |
| ORB-SLAM3 | — | Build at `/home/daniel/ORB_SLAM3` |
| Pangolin | — | Required even if viewer is disabled (ORB-SLAM3 headers) |
| Retinify | — | GPU stereo depth library (`libretinify.so`) |
| OpenCV | 4.x | With CUDA support recommended |
| CUDA Toolkit | 12.x | For Retinify pinned memory and GPU operations |
| Eigen3 | 3.4+ | |
| Sophus | — | From ORB-SLAM3 third-party |

### Required on Jetson only

```bash
sudo apt install -y libgpiod-dev gpiod
```

> **libgpiod** is detected optionally by CMake. On x86 dev machines without it the `HAVE_GPIOD` define is not set and the GPIO trigger compiles as a no-op stub.

### USB 3 tuning (Jetson)

```bash
# Add to /etc/rc.local or a systemd unit
echo 1024 > /sys/module/usbcore/parameters/usbfs_memory_mb
```

Or add `usbcore.usbfs_memory_mb=1024` to the kernel command line in `/boot/extlinux/extlinux.conf`.

### GigE tuning (Jetson)

```bash
sudo ip link set eth0 mtu 9000              # Jumbo frames
sudo sysctl -w net.core.rmem_max=26214400
sudo sysctl -w net.core.wmem_max=26214400
```

---

## Building

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select passive_stereo_capture \
             --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

> On the Jetson, CUDA architectures `87` and `72` are automatically selected. On x86, `75 80 86 87` are targeted.

---

## Camera calibration

The node requires an OpenCV `FileStorage` YAML calibration file. Use the template as a starting point:

```
config/stereo_calibration_template.yaml
```

Required fields:

```yaml
%YAML:1.0
---
image_width:  2448
image_height: 2048
K1: !!opencv-matrix   # 3×3 intrinsic matrix — left camera
  rows: 3
  cols: 3
  dt: d
  data: [ fx, 0, cx, 0, fy, cy, 0, 0, 1 ]
D1: !!opencv-matrix   # 1×5 distortion [k1, k2, p1, p2, k3]
  rows: 1
  cols: 5
  dt: d
  data: [ 0., 0., 0., 0., 0. ]
K2: !!opencv-matrix   # right camera intrinsic
  ...
D2: !!opencv-matrix   # right camera distortion
  ...
R:  !!opencv-matrix   # 3×3 rotation — right relative to left
  ...
T:  !!opencv-matrix   # 3×1 translation in metres (baseline)
  rows: 3
  cols: 1
  dt: d
  data: [ -0.12, 0., 0. ]
```

Calibrate with [kalibr](https://github.com/ethz-asl/kalibr) or OpenCV's `stereoCalibrate()`.

---

## Configuration

All parameters live in [`config/passive_stereo.yaml`](config/passive_stereo.yaml). Key parameters:

### Camera

| Parameter | Default | Description |
|---|---|---|
| `cam_left_serial` | `"22548033"` | Spinnaker serial of the left BFS camera |
| `cam_right_serial` | `"22548025"` | Spinnaker serial of the right BFS camera |
| `frame_rate` | `30.0` | Frames per second |
| `exposure_time` | `33333.84` | Exposure in microseconds |
| `gain` | `0.0` | Gain in dB (when `gain_auto: false`) |
| `gain_auto` | `false` | Enable auto-gain |
| `balance_white_auto` | `true` | Enable auto white balance |
| `binning` | `1` | Sensor binning factor (1 = full resolution) |
| `max_consec_errors` | `10` | Consecutive grab errors before `rclcpp::shutdown()` |

### Trigger

| Parameter | Default | Description |
|---|---|---|
| `trigger_mode` | `true` | `true` = hardware GPIO trigger; `false` = continuous |
| `trigger_delay_us` | `29` | Trigger-to-exposure delay (µs) |
| `gpio_chip` | `"gpiochip0"` | libgpiod chip name |
| `gpio_line` | `85` | GPIO line number (PN.01 on Jetson Orin Nano) |

### Image enhancement

| Parameter | Default | Description |
|---|---|---|
| `clahe_clip_limit` | `2.0` | CLAHE clip limit (Lab L-channel, full-res, for Retinify/Preview) |
| `clahe_tile_size` | `8` | CLAHE tile grid size |

### SLAM (ORB-SLAM3)

| Parameter | Default | Description |
|---|---|---|
| `slam_enabled` | `true` | Enable/disable the SLAM worker |
| `slam_voc_file` | — | Path to ORB vocabulary `.txt` |
| `slam_settings_file` | — | Path to ORB-SLAM3 stereo settings `.yaml` |
| `slam_scale_factor` | `0.33` | Scale applied to gray image before tracking (e.g. 0.33 → ~808×676 from 2448×2048) |
| `slam_use_pangolin` | `false` | Open Pangolin visualizer (requires X11/display) |
| `slam_enu_publish` | `true` | Rotate pose to ENU (East-North-Up) frame |
| `slam_tf_publish` | `false` | Also broadcast TF transform |
| `slam_cloud_hz` | `2.0` | Max rate to publish sparse map cloud (Hz) |

### Disparity (Retinify)

| Parameter | Default | Description |
|---|---|---|
| `disparity_enabled` | `true` | Enable/disable GPU depth worker |
| `depth_mode` | `"accurate"` | `"fast"` \| `"balanced"` \| `"accurate"` |
| `max_dist` | `15.0` | Maximum point depth in metres (0 = no limit) |
| `sampling_factor` | `1.0` | Point cloud decimation: 0.5 = every other pixel |
| `crop_factor` | `1.0` | Central crop fraction of the image |
| `min_confidence` | `0.35` | Minimum local disparity confidence to keep a point |
| `publish_confidence` | `true` | Add a `confidence` field to the point cloud |

### Preview (unrectified raw perspective, BGR8)

| Parameter | Default | Description |
|---|---|---|
| `preview_enabled` | `true` | Enable/disable JPEG preview streaming |
| `preview_width` | `612` | Output preview width |
| `preview_height` | `512` | Output preview height |
| `preview_quality` | `50` | JPEG quality (0–100) |
| `preview_fps` | `10.0` | Max publishing framerate (Hz) for preview |

---

## Running

### Full pipeline (hardware trigger)

```bash
ros2 launch passive_stereo_capture passive_stereo_capture.launch.py \
    calibration_file:=/path/to/your_calibration.yaml
```

### Continuous mode (no GPIO, for bench testing)

```bash
ros2 launch passive_stereo_capture passive_stereo_capture.launch.py \
    trigger:=false \
    calibration_file:=/path/to/your_calibration.yaml
```

### SLAM only, no disparity or preview

```bash
ros2 launch passive_stereo_capture passive_stereo_capture.launch.py \
    disparity:=false preview:=false \
    calibration_file:=/path/to/your_calibration.yaml
```

### With Pangolin viewer (on a machine with a display)

```bash
ros2 launch passive_stereo_capture passive_stereo_capture.launch.py \
    pangolin:=true \
    calibration_file:=/path/to/your_calibration.yaml
```

### Reset SLAM from the command line

```bash
ros2 service call /Passive/slam/reset std_srvs/srv/Trigger
```

---

## Published topics

All topics are prefixed with `/Passive/` (configurable via `namespace` parameter).

| Topic | Type | Hz | Description |
|---|---|---|---|
| `/Passive/slam/pose_cov` | `PoseWithCovarianceStamped` | 30 | Camera pose in ENU map frame |
| `/Passive/slam/pointcloud` | `PointCloud2` | ≤2 | Sparse ORB-SLAM3 map points |
| `/Passive/slam/path` | `Path` | 30 | Pose history |
| `/Passive/disparity/pointcloud` | `PointCloud2` | 30 | Dense Retinify depth cloud (XYZRGB[+conf]) |
| `/Passive/left/preview/image/compressed` | `CompressedImage` | 30 | Left JPEG preview |
| `/Passive/right/preview/image/compressed` | `CompressedImage` | 30 | Right JPEG preview |

---

## Services

| Service | Type | Description |
|---|---|---|
| `/Passive/slam/reset` | `std_srvs/Trigger` | Reset ORB-SLAM3 map and initial pose offset |

---

## Source layout

```
passive_stereo_capture/
├── config/
│   ├── passive_stereo.yaml              # All node parameters
│   ├── stereo_calibration_template.yaml # Calibration format reference
│   └── stereo_calibration_bfs.yaml      # Fill in your measured values
├── include/
│   ├── bounded_queue.hpp        # Thread-safe drop-oldest/newest queue
│   ├── clahe_processor.hpp      # applyClaheRGB() — Lab L-channel CLAHE
│   ├── disparity_worker.hpp     # Retinify GPU worker
│   ├── gpio_trigger.hpp         # libgpiod PWM trigger
│   ├── passive_stereo_node.hpp  # Top-level ROS 2 node
│   ├── preview_worker.hpp       # JPEG preview streamer
│   ├── slam_worker.hpp          # ORB-SLAM3 tracking worker
│   ├── spinnaker_grabber.hpp    # Spinnaker camera pair + sync
│   ├── stereo_frame.hpp         # Shared stereo frame struct
│   └── stereo_rectifier.hpp     # Stereo rectification maps
├── launch/
│   └── passive_stereo_capture.launch.py
└── src/
    ├── disparity_worker.cpp
    ├── gpio_trigger.cpp
    ├── main.cpp
    ├── passive_stereo_node.cpp
    ├── preview_worker.cpp
    ├── slam_worker.cpp
    ├── spinnaker_grabber.cpp
    └── stereo_rectifier.cpp
```

---

## Troubleshooting

### Cameras not found

```
SpinnakerGrabber: camera serial 'XXXXXXXX' not found
```

Check connected cameras with:

```bash
/opt/spinnaker/bin/SpinView   # GUI
# or
python3 -c "import PySpin; sys = PySpin.System.GetInstance(); print(sys.GetCameras().GetSize()); sys.ReleaseInstance()"
```

Verify serials in `passive_stereo.yaml` match your hardware:

```bash
ros2 param set /passive_stereo_node cam_left_serial "XXXXXXXX"
```

### USB bandwidth errors / incomplete frames

```bash
echo 1024 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

Lower `frame_rate` or enable `binning: 2` to reduce bandwidth.

### SLAM tracking lost immediately

- Verify `slam_voc_file` and `slam_settings_file` paths are correct.
- Check that `slam_settings_file` has matching image dimensions for your `slam_scale_factor` (e.g. 808×676 for 0.33 on 2448×2048).
- Start with `slam_scale_factor: 0.5` if tracking is unstable.
- Enable `slam_use_pangolin: true` (with a display) to visually inspect initialization.

### No GPIO trigger output

On Jetson, confirm the line is correct:

```bash
gpioinfo gpiochip0 | grep -E "PN\.01|line 85"
```

Check `gpio_line` in the YAML. The node logs the chip and line at startup.

### libgpiod not found (on x86 dev machine)

Expected — `HAVE_GPIOD` is not defined and GPIO trigger is compiled as a no-op. Set `trigger_mode: false` when running on non-Jetson hardware.

---

## License

See [LICENSE](LICENSE).
