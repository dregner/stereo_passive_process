# passive_stereo_capture

A ROS 2 node that acquires synchronized stereo frames from two FLIR Blackfly S (BFS) color cameras via the Spinnaker SDK and feeds them to three parallel processing workers: **ORB-SLAM3** (localization), **Retinify** (GPU dense stereo / depth), and a **JPEG preview streamer**. All camera-to-worker data paths are direct C++ calls — ROS 2 is used only for output topics and parameter management, eliminating the serialization overhead of image topics on the hot path.

Designed to run on a **Jetson Orin Nano Super (JetPack 6.2, aarch64)** with cameras connected via USB 3.0 or GigE.

---

## Architecture

```
Spinnaker raw Bayer stereo pair
  ├─ SLAM queue (latest pending pair) → gray → scale → optional gray CLAHE → ORB-SLAM3
  └─ Color queue (latest pending pair) → RGB
       ├─ Depth queue → optional resize → optional Lab L CLAHE → Retinify
       │              → rate-limited cloud / disparity visualization
       └─ Preview queue → resize → optional Lab L CLAHE → JPEG
```

SLAM is dispatched before RGB conversion. Frames share owned Bayer storage;
color workers receive separate metadata so they never modify a frame being read
by SLAM. All pending queues have capacity one and drop their oldest pending item.
This bounds queued work, but does not guarantee processing every acquired frame.

ROS headers preserve the left image's host receipt timestamp. This is not an
exposure timestamp; camera-clock mapping and hardware synchronization remain
necessary for accurate timing and geometry. Worker diagnostics report the latest
processing duration, receipt age, and queue drop counters, not percentile statistics.

`enable_clahe` controls Lab L-channel CLAHE for depth/preview and grayscale CLAHE
for SLAM. Depth CLAHE runs after the optional processing resize.

**Calibration warning:** the supplied calibration values are examples. The selected
ORB-SLAM3 `Rectified` settings expect already rectified images, while this wrapper
currently supplies unrectified grayscale. Benchmark rates and latency before
calibration, but do not interpret the resulting pose scale or cloud as validated
geometry. Generate matching SLAM and depth settings and enable the appropriate
rectification when calibrating the final stereo assembly.

### Live load tests

Build separately without changing the existing workspace installation:

```bash
source /opt/ros/jazzy/setup.bash
cmake -S . -B /tmp/passive_stereo_build -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/passive_stereo_build -j2
/usr/bin/python3 tools/run_load_test.py --binary /tmp/passive_stereo_build/passive_stereo_node --output /tmp/stereo_native
/usr/bin/python3 tools/run_load_test.py --binary /tmp/passive_stereo_build/passive_stereo_node --width 1600 --height 1200 --output /tmp/stereo_2mp
/usr/bin/python3 tools/run_load_test.py --binary /tmp/passive_stereo_build/passive_stereo_node --width 2448 --height 2048 --clahe --output /tmp/stereo_5mp
```

Add `--large-data` to load `config/fastdds_large_data.xml` in both test processes.
The profile uses a 128 MiB shared-memory segment and retains UDP for discovery
and remote participants. Apply it to both publisher and local subscriber in
production with `FASTRTPS_DEFAULT_PROFILES_FILE`; it is not enabled automatically
by the launch file. Remote cloud delivery still requires a separate bandwidth test.

Each test enables best-effort subscribers for pose, cloud, disparity JPEG, and both
previews, saves pipeline logs, and reports delivered rates plus receipt-stamp age
p50/p95. Exit via SIGINT releases camera acquisition. Software enlargement only
stresses depth processing/output; it does not simulate larger-sensor acquisition,
USB traffic, native full-resolution demosaicing, or additional image detail.

`depth_width` / `depth_height` must both be zero (native) or both positive.
`disp_cloud_hz` defaults to 15 Hz; actual delivery depends on worker throughput.
Pose is published for every valid tracking result (states OK / OK_KLT), with no
pose rate limiter. Lost/uninitialized tracking does not publish a fabricated pose.

The cloud confidence field is a local disparity smoothness heuristic, not model
uncertainty. Cloud packing reuses a flat buffer and emits the confidence field
when confidence filtering is available. Clouds are skipped if rectified color
retrieval fails, because raw color would not align with rectified XYZ.

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

The node currently requires an OpenCV `FileStorage` YAML calibration file even for timing tests. Use the template as a starting point:

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
| `slam_scale_factor` | `1.0` | Scale applied to gray image before tracking (e.g. 0.33 → ~808×676 from 2448×2048) |
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
| `publish_confidence` | `true` | Filter by local disparity smoothness and publish its score |

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
| `/Passive/disparity/pointcloud` | `PointCloud2` | ≤15 | Dense Retinify depth cloud (XYZRGB[+conf]) |
| `/Passive/left/preview/image/compressed` | `CompressedImage` | ≤10 | Left JPEG preview |
| `/Passive/right/preview/image/compressed` | `CompressedImage` | ≤10 | Right JPEG preview |

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
