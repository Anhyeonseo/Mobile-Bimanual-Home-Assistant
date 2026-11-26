#!/usr/bin/env bash
# Entry point inside a prepared kit, on the notebook or on the Jetson.
set -eo pipefail
pick_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$pick_root"
if [[ ! -f SHA256SUMS ]]; then
    echo "Run this launcher inside the prepared pick kit." >&2
    exit 2
fi
pick_action="${1:-preflight}"
if [[ $# -gt 0 ]]; then shift; fi
pick_python="${PICK_PYTHON:-}"
pick_active="${VIRTUAL_ENV:-${CONDA_PREFIX:-}}"
pick_source="${GRASP_SOURCE_REPO:-$HOME/Documents/GitHub/SO101-Can-Towel-Manipulation}"
if [[ "$pick_action" == language ]]; then
    [[ $# == 0 ]] || exit 2
    pick_lang="${PICK_LANGUAGE_DIR:-$pick_source/workspaces/home-assistant/artifacts/s5-language}"
    if [[ ! -x "$pick_lang/llama-build/bin/llama-server" || ! -f "$pick_lang/Qwen3-4B-Q4_K_M.gguf" ]]; then
        echo "Set PICK_LANGUAGE_DIR to the existing Qwen3-4B / llama-build folder on the notebook." >&2
        exit 2
    fi
    exec "$pick_lang/llama-build/bin/llama-server" -m "$pick_lang/Qwen3-4B-Q4_K_M.gguf" \
      --host 127.0.0.1 --port 18767 -c 2048 -t 4 --parallel 1
fi
pick_jetson_home="${PICK_JETSON_HOME:-$HOME}"
pick_control="$pick_jetson_home/robot/shared/venvs/s4-jazzy"
pick_vision_venv="$pick_jetson_home/realsense-grounding/.venv"
pick_vision="$pick_vision_venv/bin/python"
if [[ "$pick_action" != demo ]]; then
    source "$pick_root/tools/lib/pick_ros_env.sh"
    pick_source_ros
    cd "$pick_root"
fi
if [[ -z "$pick_python" ]]; then
    if [[ "$pick_active" == "$pick_control" && -x "$pick_vision" ]]; then
        pick_python="$pick_vision"
    elif [[ -n "$pick_active" && -x "$pick_active/bin/python" ]]; then
        pick_python="$pick_active/bin/python"
    elif [[ -x "$pick_root/.venv/bin/python" ]]; then
        pick_python="$pick_root/.venv/bin/python"
    elif [[ -x "$pick_vision" ]]; then
        pick_python="$pick_vision"
    elif [[ -x "$pick_source/.venv-realsense/bin/python" ]]; then
        pick_python="$pick_source/.venv-realsense/bin/python"
    else
        pick_python=python3
    fi
fi
if ! command -v "$pick_python" >/dev/null; then
    echo "Python not found: $pick_python. Set PICK_PYTHON to an existing interpreter." >&2
    exit 2
fi
if [[ "$pick_python" == "$pick_vision" ]]; then
    # Keep the vision venv's native libraries first. Reuse control-side modules
    # only from a site-packages directory matching this interpreter's ABI.
    pick_vision_site="$("$pick_python" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
    pick_s4_site="$pick_control/${pick_vision_site#"$pick_vision_venv/"}"
    if [[ -d "$pick_s4_site" && "$pick_vision_site" == "$pick_vision_venv/"* ]]; then
        export PYTHONPATH="$pick_vision_site${PYTHONPATH:+:$PYTHONPATH}:$pick_s4_site"
    fi
elif [[ "$pick_python" == "$pick_source/.venv-realsense/bin/python" ]]; then
    export PYTHONPATH="$pick_source/workspaces/home-assistant/.venv-room-reconstruction/lib/python3.12/site-packages${PYTHONPATH:+:$PYTHONPATH}"
fi
printf 'Python: %s\n' "$(command -v "$pick_python")" >&2
export HF_HUB_OFFLINE=1 ROS_DOMAIN_ID=197 ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR="$pick_root/output/ros"
mkdir -p "$ROS_LOG_DIR"
case "$pick_action" in
  env) [[ $# == 0 ]] || exit 2; exec "$pick_python" tools/run/pick_service.py --mode live --check --candidate-source graspnet --mobile-mesh-dir assets/mobile_meshes ;;
  preflight) [[ $# == 0 ]] || exit 2; exec "$pick_python" tools/diagnostics/preflight_pick_station.py ;;
  inspect-target)
    if [[ $# != 2 || ! -d "$1" ]]; then
        echo 'Usage: pick_kit.sh inspect-target /path/to/near/RGBD "물체 이름 또는 집기 요청"' >&2
        exit 2
    fi
    mkdir -p "$pick_root/output/target-checks"
    pick_result=$(mktemp -d "$pick_root/output/target-checks/check-XXXXXXXX")
    exec "$pick_python" tools/diagnostics/check_pick_models.py --artifacts-dir "$pick_root/artifacts" \
      --snapshot "$1" --request "$2" --output "$pick_result/result" --camera-mode near --device cpu --candidate-source graspnet ;;
  demo) [[ $# == 0 ]] || exit 2; exec "$pick_python" tools/run/pick_service.py --mode demo ;;
  web)
    [[ $# == 0 ]] || exit 2
    pick_resume_flags=()
    if [[ -n "${PICK_RESUME_FROM:-}" ]]; then pick_resume_flags=(--resume-from "$PICK_RESUME_FROM"); fi
    exec "$pick_python" tools/run/pick_service.py --mode station --port "${PICK_WEB_PORT:-18768}" --device cpu --candidate-source graspnet-so101 \
      --stage-timeout-seconds 0 "${pick_resume_flags[@]}" \
      --camera-mode wide --artifacts-dir "$pick_root/artifacts" --mobile-mesh-dir "$pick_root/assets/mobile_meshes" \
      --top-baseline "${PICK_TOP_BASELINE:-/home/hyper/robot/diagnostics/tape-top-reference-20261117-preinstall-baseline.json}" \
      --baseline "${PICK_BASELINE:-/home/hyper/robot/shared/diagnostics/s4-trial-20261114/baseline.json}" \
      --serial-port "${PICK_SERIAL:-/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_005100393235511438363730-if02}" ;;
  plan-only|pick)
    if [[ $# != 1 || "$1" != --supported-empty-bottom ]]; then
        echo "Confirm both empty arms are supported and the lift is physically at the bottom, then pass --supported-empty-bottom." >&2
        exit 2
    fi
    pick_flags=()
    if [[ "$pick_action" == plan-only ]]; then pick_flags=(--plan-only); fi
    exec "$pick_python" tools/run/pick_service.py --mode live --device cpu --camera-mode wide \
      --candidate-source so101 --artifacts-dir "$pick_root/artifacts" --mobile-mesh-dir "$pick_root/assets/mobile_meshes" \
      --baseline "${PICK_BASELINE:-/home/hyper/robot/shared/diagnostics/s4-trial-20261114/baseline.json}" \
      --serial-port "${PICK_SERIAL:-/dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_005100393235511438363730-if02}" \
      --lift-target-um 0 --execute --supported-empty-bottom "${pick_flags[@]}" ;;
  *) echo 'Usage: pick_kit.sh {env|preflight|demo|language|inspect-target|web|plan-only|pick}' >&2; exit 2 ;;
esac
