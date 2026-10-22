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
mkdir -p "$build_dir"
binary="$build_dir/stereo_camera_gui"
source_checksum="$(cksum < "$script_dir/stereo_camera_gui.cpp")"
cached_checksum="$(cat "$binary.source-checksum" 2>/dev/null || true)"
if [[ ! -x "$binary" || "$source_checksum" != "$cached_checksum" ]]; then
    echo "Compiling stereo GUI from $script_dir/stereo_camera_gui.cpp"
    c++ -std=c++17 -O2 "$script_dir/stereo_camera_gui.cpp" -o "$binary" \
        -I/opt/spinnaker/include -L/opt/spinnaker/lib -Wl,-rpath,/opt/spinnaker/lib \
        $(pkg-config --cflags --libs opencv4 yaml-cpp) -lSpinnaker -pthread
    printf '%s\n' "$source_checksum" > "$binary.source-checksum"
fi
if [[ ${1:-} == --check-display ]]; then
    exec "$binary" --check-display
fi
exec "$binary" "${1:-$script_dir/../config/passive_stereo_pc.yaml}" "${2:-stereo_pairs}"
