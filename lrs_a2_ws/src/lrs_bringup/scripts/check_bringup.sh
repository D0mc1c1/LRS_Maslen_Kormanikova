#!/usr/bin/env bash
# Kontrola §1: telemetria oboch MAVROS a obe kamery tecu naraz.
#
# Pre kazdy dron overi, ze kazda tema ma skutocneho publishera (Publisher count >= 1 v
# `ros2 topic info --verbose`), nie len meno v `ros2 topic list`, a ze MAVROS je pripojeny
# k SITL (connected: true). Z camera_info vypise fx a rozlisenie (pre 640 px a HFOV 60° ma
# vyjst fx ~ 554 px) a zmeria frekvenciu obrazu.
#
#   ros2 run lrs_bringup check_bringup.sh          # oba drony
#   ros2 run lrs_bringup check_bringup.sh 1        # len drone_1
#
# Navratovy kod 0 = vsetko PASS, 1 = aspon jeden FAIL.
set -uo pipefail

drones=("$@")
if [[ ${#drones[@]} -eq 0 ]]; then
  drones=(1 2)
fi

fails=0
pass() { printf '  \e[32mPASS\e[0m %s\n' "$1"; }
fail() { printf '  \e[31mFAIL\e[0m %s\n' "$1"; fails=$((fails + 1)); }

# pocet publisherov temy (0, ak tema neexistuje)
publishers() {
  timeout 10 ros2 topic info "$1" --verbose 2>/dev/null |
    awk -F': *' '/Publisher count/ {print $2; exit}'
}

# jedna sprava, jedno pole (prazdne pri timeoute)
field() {
  timeout "${3:-10}" ros2 topic echo "$1" --once --field "$2" 2>/dev/null | head -n 20
}

for n in "${drones[@]}"; do
  ns="/drone_${n}"
  echo "== drone_${n} =="

  for t in mavros/state mavros/local_position/pose down_camera/image_raw down_camera/camera_info; do
    c="$(publishers "$ns/$t")"
    if [[ -n "$c" && "$c" -ge 1 ]]; then
      pass "$ns/$t (publisherov: $c)"
    else
      fail "$ns/$t nema publishera"
    fi
  done

  # ROS vypisuje "True"; po "ros2 daemon stop" trva hladanie temy dlhsie, preto 2 pokusy
  conn=""
  for _ in 1 2; do
    conn="$(field "$ns/mavros/state" connected 15 | grep -i -m1 -E '^(true|false)$')"
    [[ -n "$conn" ]] && break
  done
  if [[ "${conn,,}" == "true" ]]; then
    pass "MAVROS pripojeny k SITL (connected: $conn)"
  else
    fail "MAVROS nie je pripojeny (connected: ${conn:-ziadna sprava do 30 s})"
  fi

  k="$(field "$ns/down_camera/camera_info" k)"
  w="$(field "$ns/down_camera/camera_info" width | head -n 1)"
  if [[ -n "$k" ]]; then
    fx="$(echo "$k" | tr -d '[]-' | tr ',' '\n' | awk 'NF {print $1; exit}')"
    pass "camera_info: ${w:-?} px na sirku, fx = ${fx}"
  else
    fail "camera_info neprislo (over nazov: gz topic -l | grep down_camera)"
  fi

  # poloha z MAVROS: pod ~3 Hz ma follow_mission aj detektor zastaranu polohu
  phz="$(timeout 12 ros2 topic hz "$ns/mavros/local_position/pose" 2>/dev/null |
    awk '/average rate/ {r=$3} END {print r}')"
  if [[ -z "$phz" ]]; then
    fail "mavros/local_position/pose nechodi"
  elif awk -v r="$phz" 'BEGIN {exit !(r >= 3.0)}'; then
    pass "poloha z MAVROS ${phz} Hz"
  else
    fail "poloha z MAVROS len ${phz} Hz (treba >= 3): sitl_2x.sh so SITL_STREAMRATE=20, pomala simulacia?"
  fi

  hz="$(timeout 12 ros2 topic hz "$ns/down_camera/image_raw" 2>/dev/null |
    awk '/average rate/ {r=$3} END {print r}')"
  if [[ -n "$hz" ]]; then
    pass "image_raw ${hz} Hz"
  else
    fail "image_raw nechodi"
  fi
done

echo
if [[ $fails -eq 0 ]]; then
  echo "Vsetko PASS (${#drones[@]} dron/y)."
  exit 0
fi
echo "FAIL: $fails"
echo "Tip: ros2 daemon stop, ak ros2 topic nevidi temy, ktore existuju."
exit 1
