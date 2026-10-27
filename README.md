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
| `slam_enu_publish` | `true` | Use local FLU (forward/left/up) axes for pose, path and cloud; false retains optical axes |
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

### Reset SLAM from the command line

```bash
ros2 service call /Passive/slam/reset std_srvs/srv/Trigger
```

---

## Published topics

All topics are prefixed with `/Passive/` (configurable via `namespace` parameter).

| Topic | Type | Hz | Description |
|---|---|---|---|
| `/Passive/slam/pose_cov` | `PoseWithCovarianceStamped` | 30 | Base/camera pose in the configured map frame |
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

### Conventional CPU disparity

Set `disparity_backend: "stereosgbm"` (OpenCV SGBM 3-way) or
`disparity_backend: "stereobm"`, or `disparity_backend: "stereobinary"`
(OpenCV contrib StereoBinarySGBM) in your parameter YAML. The default remains
`"retinify"`. All CPU backends publish through the existing disparity JPEG and
optionally enabled XYZRGB point-cloud topics, using the same publication rates, crop, sampling,
distance limit, and optional local smoothness confidence filter.

CPU matching rectifies the RGB pair using the stereo calibration, resizes to
`depth_width` / `depth_height` when configured, and matches grayscale images.
The calibration Q matrix is adjusted for the resized pixel coordinates and
disparities before reconstruction in metres. Invalid or nonpositive disparities
are excluded from the cloud and shown black in the JPEG.

| Parameter | Default | Meaning |
|---|---:|---|
| `stereo_min_disparity` | 0 | Minimum searched disparity in output-image pixels (nonnegative) |
| `stereo_num_disparities` | 128 | Search span; positive multiple of 16 |
| `stereo_block_size` | 9 | Odd block size; BM requires 5–255, SGBM 1–255 |
| `stereo_uniqueness_ratio` | 10 | Match uniqueness margin (%) |
| `stereo_speckle_window_size` | 100 | Remove small disparity regions; 0 disables |
| `stereo_speckle_range` | 2 | Speckle disparity tolerance in pixels |
| `stereo_disp12_max_diff` | 1 | Left/right consistency tolerance; nonpositive disables |
| `stereo_pre_filter_cap` | 31 | Prefilter clipping (1–63) |
| `stereo_texture_threshold` | 10 | BM texture rejection threshold |

SGBM uses grayscale smoothness penalties P1=8×block_size² and
P2=32×block_size². See the [OpenCV StereoSGBM documentation](https://docs.opencv.org/4.x/d2/d85/classcv_1_1StereoSGBM.html)
and [StereoBM documentation](https://docs.opencv.org/4.x/d9/dba/classcv_1_1StereoBM.html).
For an initial CPU configuration, try `depth_width: 800`, `depth_height: 600`,
and tune the disparity search at that resolution. The search plus block size
must fit within the image width. Calibration must describe a horizontal stereo
rig, with translation expressed in metres. CPU backend selection avoids Retinify
execution and pinned-memory allocation; the package still requires its existing
Retinify/CUDA build dependencies.

With `BUILD_TESTING=ON`, `conventional_stereo_test` checks all three matchers against
synthetic stereo pairs with a known shift, invalid matches, parameter validation,
and metric reconstruction after resizing.

`stereobinary` uses binary descriptors with SGBM aggregation, with P1=100 and
P2=1000. Constant left-image patches (local grayscale variance below 1) are
masked to reject arbitrary matches. It shares the `stereo_*` matching parameters; `stereo_texture_threshold`
is BM-only. The OpenCV contrib `stereo` module is required at build time.
See [OpenCV StereoBinarySGBM](https://docs.opencv.org/4.10.0/d1/d9f/classcv_1_1stereo_1_1StereoBinarySGBM.html).


## Standalone stereo camera GUI

Run without ROS, SLAM, or disparity processing:

```bash
./tools/stereo_camera_gui.sh config/passive_stereo_pc.yaml /tmp/stereo_capture
```

The launcher compiles a small C++ viewer using the installed Spinnaker SDK,
OpenCV HighGUI, yaml-cpp, and a C++17 compiler. The compiled executable is cached
under `/tmp/passive_stereo_gui_$UID` (override with `STEREO_GUI_BUILD_DIR`).
Stop other camera acquisition programs before launching it.

For remote operation, connect from a computer with a running X server, then run
the launcher **on the camera host**:

```bash
ssh -X user@camera-host
cd ~/ros2_ws/src/passive_stereo_capture
./tools/stereo_camera_gui.sh config/passive_stereo_pc.yaml ~/stereo_capture
```

The camera host needs `X11Forwarding yes` in its SSH server configuration and
`xauth` installed. Let SSH set `DISPLAY` automatically. The camera SDK and GUI
dependencies must be installed on the camera host. Both windows appear on your
local desktop; controls change the remote cameras, and PNG files are saved to
`~/stereo_capture` **on the camera host**. Changes to controls apply to the running
session; they do not rewrite the configuration YAML.

Over SSH, the launcher defaults to a windowed preview at 1280 pixels total width
and at most 10 FPS. Resizing happens before display to reduce X11 traffic; camera
acquisition and saved image resolution are unchanged. Override the display limits
for a slower connection, for example:

```bash
STEREO_GUI_PREVIEW_WIDTH=960 STEREO_GUI_PREVIEW_FPS=5 \
  ./tools/stereo_camera_gui.sh config/passive_stereo_pc.yaml ~/stereo_capture
```

`STEREO_GUI_PREVIEW_WIDTH=0` displays native pixels;
`STEREO_GUI_FULLSCREEN=1` starts fullscreen. FPS accepts integers from 1 to 120.
The launcher selects Qt's X11 backend and disables its shared-memory transport
when running over SSH. If `DISPLAY` is missing, it exits with setup guidance.

To explicitly override `DISPLAY` for the viewer, place `--display` first:

```bash
./tools/stereo_camera_gui.sh --display localhost:10.0 config/passive_stereo.yaml ~/stereo_capture
```

Use the display assigned to an active SSH X11 forwarding tunnel. This option
overrides the shell variable but does not create a tunnel or X11 authorization;
`localhost:10.0` is only an example, not a value that works for every session.

If no window appears, run `./tools/stereo_camera_gui.sh --check-display` in the
same SSH session. It prints the build date, `DISPLAY`, and OpenCV GUI backend,
then shows a test window for 15 seconds without opening cameras. Also run
`xdpyinfo >/dev/null` to check whether the SSH session can reach the X server.
The launcher checks a source checksum before reusing its compiled binary, so
copying an updated source with an older timestamp still triggers a rebuild.

The YAML can use the existing `passive_stereo_node.ros__parameters` structure or
flat camera parameters. Both camera serials must match exactly. Acquisition uses
continuous free-running mode, full native Width/Height, and minimum binning and
decimation, regardless of the YAML preview size, binning, or trigger setting.
The two latest retrieved images form a pair; free-running cameras do not guarantee
simultaneous exposures. Use a hardware-synchronized acquisition pipeline when
exposure synchronization is required.

For local sessions, a fullscreen OpenCV window defaults to presenting the
native-resolution left/right images concatenated horizontally at up to 30 FPS.
OpenCV fits the view to the screen while preserving
aspect ratio; the source and saved images retain every pixel. Press **F** to toggle
fullscreen, **S** to save, or **Q/Esc** to exit. The separate controls window has a
**Save image pair** button and trackbars that apply to both cameras:

- Exposure time in microseconds and gain in 0.1 dB steps.
- Exposure auto and gain auto: 0 = Off/manual, 1 = Once, 2 = Continuous.
- White balance auto with the same three modes, plus manual red/blue ratios in
  0.01 steps. Manual values apply when the corresponding auto mode is Off.

Exposure uses the camera's Timed mode; the exposure mode control selects automatic
versus manual exposure, rather than TriggerWidth acquisition. Unsupported or locked
camera features are reported in the controls window and terminal. Float settings
are clamped to camera limits; long exposures can lower the achieved frame rate.
To run Once again, move its mode control away from 1 and back.

The output argument selects the parent folder. Captures are saved as
`left/L000.png`, `right/R000.png`, then `L001.png`/`R001.png`, and so on. Existing
numbers are skipped, including when only one side exists. Saved images are BGR8
color PNGs with no overlays or resizing. The default output is `./stereo_pairs`.

Initial settings use `exposure_time`, `gain`, `gain_auto`, and `balance_white_auto`
from the YAML. Optional additions are `exposure_auto`, `gain_mode`, and
`white_balance_mode` (`Off`, `Once`, or `Continuous`), plus `balance_ratio_red`
and `balance_ratio_blue` (default 1.5).


### Disparity output and rectification

`disp_publish_cloud: false` (default) publishes only the colored disparity JPEG
on `/Passive/disparity/image/compressed`. No raw disparity topic is published.
Enable `disp_publish_cloud` explicitly to restore the dense point cloud.
Cloud-only XYZ, point packing, and rectified color buffers are omitted when disabled.

OpenCV BM/SGBM/Binary matching always receives CPU-rectified images. Retinify
receives raw images plus K/D/R/T and performs its own rectification by default.
Set `disparity_rectify_on_cpu: true` to rectify before Retinify; that path supplies
rectified P intrinsics, zero distortion, identity rotation, and rectified baseline.
Resized principal points use `(c + 0.5) * scale - 0.5`, matching pixel-center
coordinates; scale=1 leaves calibration unchanged. This applies to image resizing,
not an arbitrary calibration offset or a replacement for sensor binning calibration.

`trigger_source` selects the camera input line (default `Line3`). For the supplied
camera manual, Line2 is connector pin 3; pin 2 is Line0. Confirm physical wiring
before enabling trigger mode. Jetson chip offsets must be verified with `gpioinfo`.
The libgpiod trigger is software timed, with absolute deadlines to prevent drift;
it is not hardware PWM and scheduler jitter remains possible.


Generate matched native-resolution SLAM settings rather than reusing a differently
scaled camera file:

```bash
python3 tools/make_slam_settings.py --calibration config/stereo_calibration_bfs.yaml \
  --template config/20261006_orbslam3_pinhole.yaml --output /tmp/slam.yaml
```

Use `slam_scale_factor: 1.0` with this output. ORB-SLAM3 PinHole rectifies internally.
Its `Stereo.T_c1_c2` is the inverse of OpenCV calibration R/T. The generator sets
this direction and retains translation in metres. OpenCV contrib `stereo` is now
optional at build time; without it `stereobinary` reports an explicit error, while
BM/SGBM and Retinify remain available. Build against the same OpenCV version as
ORB-SLAM3 to avoid loading conflicting versions in one process.


### Protect tracking from background depth work

Depth processing defaults to `depth_process_hz: 10.0`, matching the default JPEG
publication rate. Set it explicitly for other desired depth rates; `0.0` processes
every available pair. Camera acquisition is independent and retains its configured
frequency. `disparity_nice: 5` lowers the depth worker's CPU scheduling priority on
Linux; `0` leaves it unchanged. This does not reserve GPU or memory bandwidth.
Keep `disp_publish_cloud: false` when only the colored JPEG is required.

`slam_trace_path` optionally writes per-frame tracking state, queue delay,
TrackStereo duration, total duration, and input frame gaps as CSV. This helps
distinguish resource contention from tracking failure; frame drops and tracking
loss are separate measurements. `disparity_trace_path` measures depth stages.

The launch file loads its installed parameter file by default. Override it with
`params_file:=/absolute/path/to/parameters.yaml`. On the Jetson, select the same
OpenCV installation for this package and ORB-SLAM3. The CUDA-enabled OpenCV 4.10
installation is selected with `-DOpenCV_DIR=/usr/local/lib/cmake/opencv4`; an older
`pkg-config` entry alone does not identify the CMake-selected or runtime version.
CUDA-enabled OpenCV does not automatically accelerate the existing CPU OpenCV
calls. Retinify uses its GPU backend independently.

### Optional OpenCV CUDA processing (OpenCV 4.10)

Build against CUDA-enabled OpenCV with `cudaimgproc`, `cudastereo` and
`cudaarithm`. Select `disparity_backend: cuda_sgm` and
`stereo_num_disparities: 256` for the current close-range calibration. CUDA SGM
accepts only 64, 128 or 256 search disparities. It uses Census costs and differs
from CPU SGBM; `stereo_cuda_p1` and `stereo_cuda_p2` tune its penalties. CPU block
size, speckle and left/right filter settings do not configure this CUDA matcher.
Rectification remains required and is performed before conventional matching.

`color_conversion_backend: cuda` moves Bayer-to-RGB conversion to CUDA; the
CUDA matcher also converts RGB to gray on-device. CPU SLAM preprocessing,
rectification, CLAHE, the JET display color map, JPEG encoding and ROS message
packing remain on CPU. CUDA Bayer channel order is checked against CPU RGB.
Buffers and CUDA streams are reused, but uploads/downloads still incur cost.

With `disp_publish_cloud: true`, CUDA SGM retains pixel disparity on-device for
CUDA XYZ reprojection. CPU matchers can opt into the same reprojection through
`opencv_reproject_on_gpu: true`. SGM fixed-point disparities are divided by 16
before reprojection. Invalid matches stay invalid, and Q accounts for image
resizing. XYZ download and point cloud packing still run on CPU. With cloud
publication disabled, or no cloud subscriber, XYZ generation is skipped.
For the requested colored JPEG output, leave `disp_publish_cloud: false`.

`tools/check_opencv_cuda.cpp` checks a known 12-pixel stereo shift, CPU/GPU XYZ
agreement and Bayer RGB channel order. It is a synthetic kernel benchmark,
not an accuracy or tracking validation on a moving rig. GPU matching can free
CPU time for tracking but also shares GPU and memory bandwidth with other work.

### Integrated hardware PWM trigger on Jetson pin 33

The stereo node can own the hardware PWM channel directly; no separate Python
process is needed. Set these ROS parameters (also available in
`config/passive_stereo.yaml`):

```yaml
trigger_mode: true
trigger_generator: pwm
trigger_source: Line2
trigger_activation: RisingEdge
frame_rate: 10.0
exposure_time: 10000.0       # Camera exposure, microseconds; independent of PWM duty
pwm_chip: /sys/class/pwm/pwmchip2
pwm_channel: 0
pwm_duty_cycle_percent: 10.0 # 10 ms HIGH at 10 Hz; does not account for trigger latency
```

`frame_rate` sets both the camera frame-rate request and PWM frequency. PWM starts
only after both cameras are armed. The node sets normal polarity and disables
and unexports its channel on shutdown or a PWM startup failure. It refuses to
take over an already exported channel. The peripheral produces the waveform;
there is no software pulse-timing thread. The supplied YAML selects hardware PWM
with 50% duty; the example above demonstrates a shorter light-control pulse.
Exposure remains `Timed`; duty cycle does not determine camera exposure.

Verify the chip's device symlink before use: on the tested setup `pwmchip2/device`
identifies `32c0000.pwm` (PWM5, pin 33), and `pwmchip3/device` identifies
`32e0000.pwm` (PWM7, pin 32). Change `pwm_chip` to select pin 32. Clock selection
and pinmux must already be configured on the Jetson; this code does not modify
either. The capture user needs write access to `export`, `unexport` and the
exported channel's `period`, `duty_cycle`, `polarity` and `enable` controls.
Configure permissions with suitable udev rules; one-time chmod on an exported
channel is lost when it is unexported. The node waits up to two seconds for those
permissions and fails explicitly if configuration or enable is rejected.

Other modes remain available: `trigger_generator: gpio` uses software-timed
libgpiod pulses; `external` arms cameras without generating any pulses. Do not
run the standalone PWM script alongside the integrated generator.

### Standalone PWM trigger on Jetson pin 33

`tools/pwm_trigger.py` controls `/sys/class/pwm/pwmchip2/pwm0` directly with no
ROS dependencies. The tested Jetson rejects true hardware PWM periods for 10/30
Hz and accepts approximately 195 Hz. This is why the supplied LED controller
uses software timing at camera frequencies. Explicit `--mode gated` schedules
constant LOW/HIGH output via duty-cycle 0/100% on the PWM peripheral; it does not
produce a 195 Hz carrier at the cameras. Its timing still depends on scheduling.
`--mode hardware` requests actual peripheral PWM and fails if unsupported.

Launch capture with the calibrated parameters and these overrides:

```bash
ros2 run passive_stereo_capture passive_stereo_node --ros-args \
  --params-file /home/jetson/ros2_ws/src/stereo_passive_process/config/passive_stereo.yaml \
  -p trigger_mode:=true -p trigger_source:=Line2 -p trigger_generator:=external
```

Once the cameras are armed, run in a second terminal:

```bash
sudo python3 /home/jetson/ros2_ws/src/stereo_passive_process/tools/pwm_trigger.py --mode gated --hz 30
```

Stop PWM with Ctrl-C. The controller sets duty to zero, disables and unexports
its channel on normal shutdown/errors. It refuses to take over an already
exported channel. Root privileges are needed for the default sysfs permissions;
the capture process runs as the normal user. Physical pin33 must be routed to
PWM; acquisition testing verifies the connected path. No ROS services or topic
commands drive the PWM. Persistent trigger parameters are unchanged.

On the validated Jetson, PWM5 has now been changed from `pllp_out0` (408 MHz)
to `clk_m` (19.2 MHz). The driver accepts 10, 20 and 30 Hz hardware periods.
`passive-pwm5-clock.service` reapplies this clock at boot without enabling output;
its script refuses to reconfigure an exported channel. With this setup, use
`pwm_trigger.py --mode hardware --hz 30` for peripheral timing. Boot execution
has not yet been verified after a restart. Pin33 acquisition remains unverified:
the latest hardware PWM test received no camera frames, and the existing pin33
overlay lacks an explicit PWM function selection. Clock acceptance alone does
not confirm the physical pin waveform.

`trigger_activation` selects the camera trigger edge (`RisingEdge` by default,
`FallingEdge` for the alternate edge). This is applied to each camera while its
trigger mode is disabled, before arming it. Edge selection does not compensate
for a missing or constant physical PWM waveform.

SLAM axis conversion uses the legacy `slam_enu_publish` parameter. With it enabled,
the local map uses forward/left/up axes; this does not establish geographic ENU.
Without a camera extrinsic TF, the published camera orientation also uses FLU
(`R_out = C R_optical C^T`, `t_out = C t_optical`). When TF is available,
`child_frame_id` must identify the ORB camera optical frame (right/down/forward),
and the published pose describes `parent_frame_id` using that TF. Pose, path, TF
and sparse map points share the same map transform. With an extrinsic, the map
is anchored to the full first valid base pose (position and orientation), so
subsequent base rotations remain visible. If TF arrives late, the map is
re-anchored and the previous path is cleared. Without TF, only the initial
camera position is zeroed. With the option disabled,
the map and fallback camera retain ORB optical axes.


### Calibrated RGB color correction

`config/passive_stereo.yaml` enables `color_correction_enabled` and contains the
nine row-major `Gain00`–`Gain22` coefficients from `color_calib/L000_flir_ccm.yaml`
(left camera) and `color_calib/R000_flir_ccm.yaml` (right camera). Verify that
`cam_left_serial` and `cam_right_serial` correspond to the cameras used for those
captures; the calibration files do not record serial numbers.

Following [Teledyne's CCM equation](https://www.teledynevisionsolutions.com/en-ca/support/support-center/application-note/iis/using-color-correction/),
output RGB = CCM × input RGB. Because acquisition uses raw BayerRG8, this matrix
runs on the host after CPU/CUDA demosaicing and before preview/depth processing.
Camera color transformation is disabled while host correction is enabled.
SLAM continues to receive raw Bayer frames. `gain` remains acquisition gain in dB.
Set `color_correction_enabled: false` to bypass host correction.

Auto white balance is disabled in the supplied configuration to keep colors
stable. Reproduce the illumination, exposure, gain, and white balance ratios used
for calibration; these files do not include those acquisition settings. The
large supplied coefficients can amplify noise and clip bright pixels. The
OpenCV implementation preserves signed intermediate values, then rounds and clips to
8-bit RGB. The saved calibration metrics are not live-camera validation.

Rebuild the package and launch with this updated parameter file to activate the
correction. The parameters are read at startup. With `BUILD_TESTING=ON`,
`color_correction_test` checks matrix orientation, clipping, identity, and invalid
coefficients.


`color_correction_backend: spinnaker` (the supplied default) uses
`ImageUtilityCCM::CreateColorCorrected()` with an encrypted custom CCM generated
once per camera at preprocessing startup. Extra SDK gamma is disabled
(`ColorSpace=OFF`). SDK results are copied into owned OpenCV buffers for safe
asynchronous worker consumption. Use `opencv` to compare with the previous path.
Both paths apply the same gains at the same point after demosaicing.

Local CPU benchmark on 2026-10-12 (synthetic RGB, OpenCV threads=1, 5 warmups and
30 measured stereo pairs, correction only including SDK output copy):

| RGB size per camera | OpenCV median ms/pair | Spinnaker median ms/pair |
|---|---:|---:|
| 800×600 | 5.15 | 2.56 |
| 1600×1200 | 29.63 | 10.73 |
| 2448×2048 | 80.76 | 27.81 |

These are host measurements, not Jetson or end-to-end FPS results. SDK internal
threading uses its defaults. Source generation, input preparation, demosaicing,
CCM encryption, and downstream processing are outside the timing interval.
`color_correction_benchmark` accepts two text files containing nine whitespace
separated gains in row-major order. The SDK test also verifies strided inputs,
source preservation and result ownership.

### MAVLink horizontal vision worker

A dedicated worker encodes MAVLink 2 and writes tracked vehicle XY directly to
native UDP/TCP sockets or a serial device. The sender has no ROS API, MAVROS,
`libmavconn`, event loop, or intermediate application transmit queue. Only the
capture node supplies the clock and error logging callbacks; the sender can
also run independently using its default system clock. The capture application
still uses ROS for parameters, TF, and its other outputs.

Build with the standalone, header-only [MAVLink C library](https://mavlink.io/en/mavgen_c/):

```bash
git clone --depth 1 https://github.com/mavlink/c_library_v2.git /tmp/c_library_v2
colcon build --packages-select passive_stereo_capture \
  --cmake-args -DMAVLINK_INCLUDE_DIR=/tmp/c_library_v2
```

Use a persistent directory for the headers if rebuilding later. CMake also
searches standard include paths; `MAVLINK_INCLUDE_DIR` must contain
`ardupilotmega/mavlink.h`. No ROS MAVLink package or linked MAVLink library is
required. The bridge is closed by default; all parameters below are read at startup.

```bash
ros2 launch passive_stereo_capture passive_stereo_capture.launch.py \
  params_file:=/absolute/path/to/parameters.yaml slam:=true \
  mavlink_enabled:=true \
  mavlink_bridge_url:='udp://:14555@127.0.0.1:14550' mavlink_message:=estimate
```

Choose the controller's or router's actual input address/port. UDP syntax is
`udp://bind_host:port@remote_host:port`; an empty bind host binds all local
interfaces, and bind port `0` selects an ephemeral source port. Only IPv4
addresses and hostnames are supported. URL query options are not supported.
For TCP use
`tcp://127.0.0.1:5760`; for serial use `serial:///dev/ttyACM0:115200`.
Only one process should own a serial port. `mavlink_enabled:=false` prevents
this process from opening the bridge or starting its publishing worker.

For `ros2 run`, add these to `passive_stereo_node.ros__parameters` in your YAML:

```yaml
    mavlink_enabled: true
    mavlink_bridge_url: "udp://:14555@127.0.0.1:14550"
    mavlink_message: "estimate"  # or "delta"
    mavlink_system_id: 1
    mavlink_component_id: 197
    mavlink_map_yaw_deg: 0.0
    mavlink_max_gap_sec: 0.5
    mavlink_position_variance: 0.05  # m², estimate covariance diagonal
    mavlink_angle_variance: 0.1      # rad², estimate covariance diagonal
    mavlink_confidence: 100.0        # delta confidence, (0, 100]
```

The launch arguments override the first three YAML settings. Source IDs identify
this sender; they are not controller target IDs. The worker always uses MAVLink 2.
It sends at the tracking update rate and keeps only the latest pending sample.
A skipped sample's movement is included in the next delta, relative to the last
successfully written update. It does not replay poses when tracking stops.
UDP sends one packet per datagram. TCP uses `TCP_NODELAY`. Serial uses raw 8N1
with flow control disabled (baud: 9600, 19200, 38400, 57600, 115200, 230400,
460800 or 921600). Writes are nonblocking: a full output buffer drops the
current packet instead of creating an application backlog. A partial stream
write followed by failure closes the connection so later poses cannot be mixed
with an unfinished packet. Kernel/device queues and wire time still apply;
this change does not establish a measured end-to-end latency improvement.

`slam_enabled` and `slam_enu_publish` must be true. A valid extrinsic TF from
`parent_frame_id` (controller vehicle base, FLU axes) to `child_frame_id`
(the camera optical frame used by ORB-SLAM3) is required before transmission.
This avoids treating a camera pose as the vehicle pose. The base pose already
includes the camera lever arm: use zero controller-side `VISO_POS_X/Y/Z` and
`VISO_ORIENT=0` to avoid applying the extrinsic twice.

The SLAM map has local forward/left/up axes, not geographic ENU. With
`mavlink_map_yaw_deg=0`, map +X becomes local NED +X, and map +Y becomes NED -Y.
The alignment parameter rotates this result about NED +Z in degrees (clockwise
viewed from above); set it to align the map with the controller's navigation
frame. The base orientation is converted from FLU to FRD consistently.

- `estimate`: `VISION_POSITION_ESTIMATE` (#102), containing local NED XY,
  Z=0, base roll/pitch/yaw, covariance, capture timestamp in microseconds and
  a reset counter. The counter changes after a SLAM reset, a tracking loss,
  a backward timestamp, or an excessive sample gap.
- `delta`: `VISION_POSITION_DELTA` (#11011), containing horizontal displacement
  rotated into the **current body FRD** frame, a rotation vector from previous
  body frame to current body frame, elapsed capture time and configured
  confidence. Body Z can be nonzero when tilted, even though world Z motion is
  excluded. The first sample and the first sample after a reset/gap seed the
  baseline without sending a delta.

Invalid or stale samples are discarded. `mavlink_max_gap_sec` bounds sample age
and delta intervals. A closed connection logs an error; restarting the capture
node is required to reopen it. A successful write does not acknowledge receipt
by the controller. SLAM loop-closure corrections can still change the map;
validate their effect on the target estimator.

For absolute XY fusion with `estimate`, select `EK3_SRC1_POSXY=6` (ExternalNav)
in the controller's active source set (use SRC2/SRC3 equivalents if selected).
Keep POSZ, VELZ and YAW on their existing sensors, and use VELXY=0 unless a
separate velocity source is provided. Z=0 in the packet is a placeholder, not a
height measurement. Select a MAVLink visual odometry backend appropriate for
the installed firmware, enable MAVLink 2 on that controller port, and establish
an EKF origin if GPS is absent. ArduPilot documents `VISO_TYPE=3` for external
navigation and an update rate of at least 4 Hz; tracking must sustain this rate.
See [ArduPilot external navigation setup](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html).

`delta` is not an absolute XY observation: ArduPilot routes it through
`writeBodyFrameOdom`, so fusion and source selection must be verified against
the controller's firmware. Selecting `EK3_SRC1_POSXY=6` alone does not make a
delta packet an absolute position update. See the
[ArduPilot delta handler](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_VisualOdom/AP_VisualOdom_Backend.cpp)
and [MAVLink delta definition](https://mavlink.io/en/messages/ardupilotmega.html#VISION_POSITION_DELTA).
The node sends observations only; it does not change controller parameters.

`mavlink_worker_test` exercises real MAVLink 2 packets over localhost UDP:
message IDs/source IDs, XY sign conversion, current-body delta translation and
rotation direction, timestamps, reset boundaries, stale samples and shutdown.
It also checks native TCP and serial I/O with a pseudoterminal, URL validation,
and nonblocking serial backpressure. The test executable links only system C/C++
libraries, not ROS or `libmavconn`. It does not validate controller fusion,
physical serial hardware, camera alignment, or end-to-end latency.
