#!/usr/bin/env bash
# A1.3: zaznam outdoor misie do rosbagu.
#
#   ros2 run lrs_control record_outdoor.sh [adresar]     (predvolene ~/lrs_bags/outdoor_<datum>)
#
# Spusti PRED outdoor_mission, zastav Ctrl+C po pristati. Potom:
#   ros2 run lrs_control plot_outdoor.py <adresar>
#
# Nahrava len temy potrebne na vyhodnotenie (zadanie radi aj `ros2 bag record -a`,
# to funguje tiez, len je bag vacsi).
set -e
OUT="${1:-$HOME/lrs_bags/outdoor_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "$OUT")"
echo "nahravam do $OUT  (Ctrl+C = koniec)"
exec ros2 bag record -o "$OUT" \
  /mavros/state \
  /mavros/local_position/pose \
  /mavros/local_position/velocity_local \
  /mavros/setpoint_raw/local \
  /mavros/setpoint_raw/target_local \
  /mavros/global_position/global \
  /mavros/global_position/rel_alt \
  /mavros/global_position/compass_hdg \
  /mavros/home_position/home \
  /lrs/outdoor/state \
  /lrs/outdoor/poi \
  /lrs/outdoor/plan \
  /lrs/outdoor/markers \
  /lrs/outdoor/flown \
  /rosout
