#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ${1:-} == --help ]]; then
    echo "Usage: $0 [CONFIG.yaml] [OUTPUT_FOLDER]"
    echo "Default YAML: config/passive_stereo_pc.yaml; output: ./stereo_pairs"
    exit 0
fi
build_dir="${STEREO_GUI_BUILD_DIR:-/tmp/passive_stereo_gui_${UID}}"
mkdir -p "$build_dir"
binary="$build_dir/stereo_camera_gui"
if [[ ! -x "$binary" || "$script_dir/stereo_camera_gui.cpp" -nt "$binary" ]]; then
    c++ -std=c++17 -O2 "$script_dir/stereo_camera_gui.cpp" -o "$binary" \
        -I/opt/spinnaker/include -L/opt/spinnaker/lib -Wl,-rpath,/opt/spinnaker/lib \
        $(pkg-config --cflags --libs opencv4 yaml-cpp) -lSpinnaker -pthread
fi
exec "$binary" "${1:-$script_dir/../config/passive_stereo_pc.yaml}" "${2:-stereo_pairs}"
