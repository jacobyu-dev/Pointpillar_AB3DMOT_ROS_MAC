#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd "$script_dir/../.." && pwd)"
default_tracklet="$project_root/data/2011_09_26/2011_09_26_drive_0032_sync/tracklet_labels.xml"
tracklet_path="${1:-$default_tracklet}"

if [[ ! -f "$tracklet_path" ]]; then
  echo "tracklet XML을 찾을 수 없습니다: $tracklet_path" >&2
  exit 1
fi

python "$project_root/mot_kf_tracking/src/mot_ab3dmot_track_node.py" \
  "_tracklet_path:=$tracklet_path" &
tracker_pid=$!

cleanup() {
  kill "$tracker_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

rviz -d "$project_root/mot_kf_tracking/config/tracklet_tracking.rviz"
