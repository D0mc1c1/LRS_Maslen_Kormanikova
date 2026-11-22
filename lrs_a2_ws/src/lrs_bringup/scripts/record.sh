#!/usr/bin/env bash
# Rosbag zaznam oboch dronov pre odovzdanie (§4) a ladenie (§3).
#
#   ros2 run lrs_bringup record.sh                 # bags/follow_<cas>, bez obrazu
#   ros2 run lrs_bringup record.sh moj_test        # bags/moj_test
#   IMAGES=1 ros2 run lrs_bringup record.sh        # aj image_raw (velky subor, zatazi PC)
#
# Ctrl+C ukonci zaznam. Format mcap (ako v A1).
set -euo pipefail

name="${1:-follow_$(date +%Y%m%d_%H%M%S)}"
outdir="${BAGS_DIR:-bags}"
mkdir -p "$outdir"

topics=()
for n in 1 2; do
  ns="/drone_${n}"
  topics+=(
    "$ns/mavros/state"
    "$ns/mavros/local_position/pose"
    "$ns/mavros/setpoint_raw/local"
    "$ns/down_camera/camera_info"
    "$ns/target/pose"
    "$ns/follow_mission/state"
    "$ns/follow_mission/estimate"
    "$ns/follow_mission/search_path"
    "$ns/aruco_detector/debug"
  )
  if [[ "${IMAGES:-0}" == "1" ]]; then
    topics+=("$ns/down_camera/image_raw")
  fi
done
# logy uzlov ("ignorujem cudziu znacku", "NAD ROVEROM", prechody stavov) ako dokaz
topics+=(/rosout)

echo "Zaznam do $outdir/$name (${#topics[@]} tem). Ctrl+C = koniec."
exec ros2 bag record -s mcap -o "$outdir/$name" "${topics[@]}"
