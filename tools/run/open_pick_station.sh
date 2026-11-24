#!/usr/bin/env bash
# Start the Jetson station and keep its loopback web connection open.
set -eo pipefail
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
host=${JETSON_HOST:-hyper@192.168.35.2}
python3 "$repo/tools/setup/package_pick_station.py"
kit=$(cd "$repo/output/pick-kit/current" && pwd -P)
kit_id=$(sha256sum "$kit/SHA256SUMS" | cut -d' ' -f1)
port_probe="$repo/tools/setup/pick_station_ports.py"
selection=$(python3 "$port_probe" local "$kit_id")
read -r local_state ports_text <<< "$selection"
read -r -a ports <<< "$ports_text"
station_matches() {
    python3 -c 'import json,sys; s=json.load(sys.stdin); sys.exit(s.get("mode")!="station" or s.get("kit_id")!=sys.argv[1])' "$kit_id"
}
if [[ "$local_state" == existing ]]; then
    url="http://127.0.0.1:${ports[0]}"
    printf 'Pick Station %s\n' "$url"
    if command -v xdg-open >/dev/null; then xdg-open "$url" >/dev/null 2>&1 || true; fi
    exit 0
fi
# Forward identical port numbers so the web server's Host/Origin checks apply.
selection=$(ssh -T -o ConnectTimeout=8 "$host" "python3 - remote '$kit_id' ${ports[*]}" < "$port_probe")
read -r remote_state port <<< "$selection"
if [[ ! "$port" =~ ^[0-9]+$ || ( "$remote_state" != available && "$remote_state" != existing ) ]]; then
    echo 'Jetson 웹 포트 응답을 확인할 수 없습니다.' >&2
    exit 1
fi
url="http://127.0.0.1:$port"
record="$kit/output/last-jetson-release"
release=''
if [[ -f "$record" ]]; then read -r release < "$record"; fi
reuse=false
if [[ "$release" =~ ^/home/[a-zA-Z0-9_-]+/robot/releases/pick-station-[a-zA-Z0-9]+$ ]]; then
    local_sum=$(sha256sum < "$kit/SHA256SUMS")
    remote_sum=$(ssh -o ConnectTimeout=8 "$host" "sha256sum < '$release/SHA256SUMS'") || remote_sum=''
    if [[ "$local_sum" == "$remote_sum" ]]; then reuse=true; fi
fi
if [[ "$reuse" == false && "$remote_state" != existing ]]; then
    bash "$repo/tools/run/stage_pick_station.sh"
    read -r release < "$record"
fi
# Known Korean names work immediately. Use the installed translator for other descriptions.
lang_dir=${PICK_LANGUAGE_DIR:-${GRASP_SOURCE_REPO:-$HOME/Documents/GitHub/SO101-Can-Towel-Manipulation}/workspaces/home-assistant/artifacts/s5-language}
forward=()
if [[ "$remote_state" == existing ]]; then
    : # The existing station keeps its own language connection.
elif curl -fsS --max-time 2 http://127.0.0.1:18767/health >/dev/null 2>&1; then
    forward=(-R 127.0.0.1:18767:127.0.0.1:18767)
elif [[ -x "$lang_dir/llama-build/bin/llama-server" && -f "$lang_dir/Qwen3-4B-Q4_K_M.gguf" ]]; then
    bash "$kit/tools/run/pick_kit.sh" language >"$kit/output/web-language.log" 2>&1 &
    lang_pid=$!
    forward=(-R 127.0.0.1:18767:127.0.0.1:18767)
fi
open_browser() {
    while true; do
        if curl -fsS --max-time 1 "$url/api/status" 2>/dev/null | station_matches 2>/dev/null; then
            xdg-open "$url" >/dev/null 2>&1 || true
            return
        fi
        sleep 1
    done
}
cleanup() {
    if [[ -n ${browser_pid:-} ]]; then kill "$browser_pid" 2>/dev/null || true; fi
    if [[ -n ${lang_pid:-} ]]; then kill "$lang_pid" 2>/dev/null || true; fi
}
trap cleanup EXIT
printf 'Pick Station %s · 이 터미널은 열어 두세요.\n' "$url"
if command -v xdg-open >/dev/null; then open_browser & browser_pid=$!; fi
if [[ "$remote_state" == existing ]]; then
    ssh -N -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 -o ServerAliveCountMax=2 \
        -L "127.0.0.1:$port:127.0.0.1:$port" "$host"
    exit $?
fi
ssh -t -o ExitOnForwardFailure=yes -o ServerAliveInterval=10 -o ServerAliveCountMax=2 \
    -L "127.0.0.1:$port:127.0.0.1:$port" "${forward[@]}" "$host" \
    "cd '$release' && PICK_WEB_PORT='$port' exec bash tools/run/pick_kit.sh web"
