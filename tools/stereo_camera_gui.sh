#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ${1:-} == --display ]]; then
    if [[ $# -lt 2 || -z ${2:-} || ${2:-} == --* ]]; then
        echo "Usage: $0 --display DISPLAY [CONFIG.yaml] [OUTPUT_FOLDER]" >&2
        exit 1
    fi
    export DISPLAY="$2"
    shift 2
    echo "Using explicit DISPLAY=$DISPLAY"
fi
if [[ ${1:-} == --help ]]; then
    echo "Usage: $0 [--display DISPLAY] [CONFIG.yaml] [OUTPUT_FOLDER]"
    echo "       $0 --check-display  (test a window without opening cameras)"
    echo "Default YAML: config/passive_stereo_pc.yaml; output: ./stereo_pairs"
    echo "Run on the camera host after connecting with ssh -X user@camera-host."
    echo "Images are saved on that host; the windows appear on your local X server."
    echo "--display overrides DISPLAY; the selected X server / SSH tunnel must already exist."
    echo "SSH preview defaults: 1280 pixels wide, 10 FPS, windowed."
    echo "Overrides: STEREO_GUI_PREVIEW_WIDTH, STEREO_GUI_PREVIEW_FPS,"
    echo "           STEREO_GUI_FULLSCREEN (0 or 1). Width 0 preserves native preview."
    echo "Build: CMake, C++17, OpenCV HighGUI, yaml-cpp, and Spinnaker are required."
    echo "Set OpenCV_DIR to the directory containing OpenCVConfig.cmake if needed."
    exit 0
fi
if [[ -z ${DISPLAY:-} ]]; then
    echo "No X11 DISPLAY is available. Connect using ssh -X user@camera-host from a running X server." >&2
    echo "The camera host needs X11Forwarding enabled in sshd and xauth installed." >&2
    exit 1
fi
if [[ -n ${SSH_CONNECTION:-} || -n ${SSH_TTY:-} ]]; then
    # Qt must use the forwarded X display, even on hosts with a local Wayland session.
    export QT_QPA_PLATFORM=xcb
    export QT_X11_NO_MITSHM=1
    export STEREO_GUI_PREVIEW_WIDTH="${STEREO_GUI_PREVIEW_WIDTH:-1280}"
    export STEREO_GUI_PREVIEW_FPS="${STEREO_GUI_PREVIEW_FPS:-10}"
    export STEREO_GUI_FULLSCREEN="${STEREO_GUI_FULLSCREEN:-0}"
fi
build_dir="${STEREO_GUI_BUILD_DIR:-/tmp/passive_stereo_gui_${UID}}"
mkdir -p "$build_dir/cmake-source"
binary="$build_dir/stereo_camera_gui"
# Source installations of OpenCV often provide CMake metadata but no .pc file.
cat > "$build_dir/cmake-source/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.18)
project(stereo_camera_gui LANGUAGES CXX)
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs highgui)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_path(SPINNAKER_INCLUDE_DIR Spinnaker.h HINTS /opt/spinnaker/include REQUIRED)
find_library(SPINNAKER_LIBRARY Spinnaker HINTS /opt/spinnaker/lib REQUIRED)
add_executable(stereo_camera_gui "${STEREO_GUI_SOURCE_FILE}")
target_compile_features(stereo_camera_gui PRIVATE cxx_std_17)
target_include_directories(stereo_camera_gui PRIVATE ${OpenCV_INCLUDE_DIRS} "${SPINNAKER_INCLUDE_DIR}")
if(TARGET yaml-cpp::yaml-cpp)
    set(YAML_TARGET yaml-cpp::yaml-cpp)
else()
    set(YAML_TARGET yaml-cpp)
endif()
target_link_libraries(stereo_camera_gui PRIVATE ${OpenCV_LIBS} ${YAML_TARGET} "${SPINNAKER_LIBRARY}" Threads::Threads)
message(STATUS "Stereo GUI uses OpenCV ${OpenCV_VERSION} from ${OpenCV_DIR}")
CMAKE
cmake_args=(-DCMAKE_BUILD_TYPE=Release "-DSTEREO_GUI_SOURCE_FILE=$script_dir/stereo_camera_gui.cpp")
if [[ -n ${OpenCV_DIR:-} ]]; then
    cmake_args+=("-DOpenCV_DIR=$OpenCV_DIR")
fi
cmake -S "$build_dir/cmake-source" -B "$build_dir" "${cmake_args[@]}"
cmake --build "$build_dir" --target stereo_camera_gui
if [[ ${1:-} == --check-display ]]; then
    exec "$binary" --check-display
fi
exec "$binary" "${1:-$script_dir/../config/passive_stereo_pc.yaml}" "${2:-stereo_pairs}"
