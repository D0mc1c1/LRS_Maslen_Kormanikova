#!/usr/bin/env bash
# Test §4: oba drony sleduju rovery, ktore sa pohybuju nahodne (randomize_rovers.py).
#
# Pre kazdy seed:
#   1. pocka, kym su oba drony na zemi a disarmnute,
#   2. spusti rosbag (record.sh), randomizer roverov so seedom a follow.launch.py,
#   3. po --duration sekundach (alebo Ctrl+C) zastavi misiu a rovery, oba drony nechaju
#      pristat (LAND) a pocka na disarm,
#   4. z logu vyhodnoti §4 (analyze_follow_log.py): TRACK, strata -> znova najdeny, ABORT.
#
# Predpoklad: bezi Gazebo, sitl_2x.sh a bringup.launch.py (check_bringup.sh = PASS).
#
#   ros2 run lrs_bringup run_follow_test.sh 42                     # jeden seed, 300 s
#   ros2 run lrs_bringup run_follow_test.sh --slow --debug 42 7 123
#   ros2 run lrs_bringup run_follow_test.sh --duration 180 --images 42
#
# Moznosti:
#   --duration S   dlzka jedneho testu [s] (predvolene 300)
#   --slow         follow.launch.py slow:=true (pomala simulacia)
#   --debug        ladiaci obraz drone_N/aruco_detector/debug (aj do rosbagu)
#   --images       do rosbagu aj image_raw (velky subor)
#   --no-bag       bez rosbagu
# Premenne: LRS_URK_DIR (predvolene ~/LRS-URK), BAGS_DIR (predvolene ~/lrs_tests)
#
# Vystup: $BAGS_DIR/seed<N>_<cas>/ follow.log, randomize.log, summary.txt, bag/
set -uo pipefail

LRS_URK_DIR="${LRS_URK_DIR:-$HOME/LRS-URK}"
BAGS_DIR="${BAGS_DIR:-$HOME/lrs_tests}"
duration=300
slow=false
debug=false
images=0
bag=1
seeds=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration) duration="$2"; shift 2 ;;
    --slow) slow=true; shift ;;
    --debug) debug=true; shift ;;
    --images) images=1; shift ;;
    --no-bag) bag=0; shift ;;
    -h|--help) sed -n '2,25p' "$0"; exit 0 ;;
    *) seeds+=("$1"); shift ;;
  esac
done
if [[ ${#seeds[@]} -eq 0 ]]; then
  echo "Zadaj aspon jeden seed, napr.: run_follow_test.sh 42" >&2
  exit 1
fi

# LRS_URK_DIR moze byt koren repozitara (~/LRS-URK) aj jeho scripts/ (tak ho nastavuje predmet)
RANDOMIZER=""
for cand in "$LRS_URK_DIR/scripts/randomize_rovers.py" "$LRS_URK_DIR/randomize_rovers.py" \
  "$HOME/LRS-URK/scripts/randomize_rovers.py"; do
  if [[ -f "$cand" ]]; then
    RANDOMIZER="$cand"
    break
  fi
done
if [[ -z "$RANDOMIZER" ]]; then
  echo "randomize_rovers.py nenajdeny v $LRS_URK_DIR ani ~/LRS-URK (git pull v LRS-URK?)" >&2
  exit 1
fi
echo "randomizer: $RANDOMIZER"
PKG_LIB="$(ros2 pkg prefix lrs_bringup)/lib/lrs_bringup"

field() {  # jedna hodnota z temy alebo prazdne
  timeout 10 ros2 topic echo "$1" --once --field "$2" 2>/dev/null | head -n 1
}

wait_disarmed() {
  local limit="$1"
  for n in 1 2; do
    local t=0 armed
    while true; do
      armed="$(field "/drone_${n}/mavros/state" armed)"
      if [[ "${armed,,}" == "false" ]]; then
        echo "  drone_${n}: na zemi, disarmnuty"
        break
      fi
      if [[ $t -ge $limit ]]; then
        echo "  drone_${n}: stale armed (${armed:-ziadna sprava}) po ${limit} s" >&2
        return 1
      fi
      sleep 5
      t=$((t + 5))
    done
  done
}

# LAND cez MAVROS: kazdy pokus s casovym limitom (ros2 service call inak moze visiet navzdy),
# az 3 pokusy, uspech = /drone_N/mavros/state ukazuje mode LAND
land_all() {
  for n in 1 2; do
    local ok=0 mode
    for attempt in 1 2 3; do
      timeout 20 ros2 service call "/drone_${n}/mavros/set_mode" mavros_msgs/srv/SetMode \
        "{custom_mode: 'LAND'}" >/dev/null 2>&1
      mode="$(field "/drone_${n}/mavros/state" mode)"
      if [[ "$mode" == *LAND* ]]; then
        ok=1
        break
      fi
      echo "  drone_${n}: LAND pokus $attempt nevysiel (mode ${mode:-?}), skusam znova"
      sleep 2
    done
    if [[ $ok -eq 1 ]]; then
      echo "  drone_${n}: LAND"
    else
      echo "  drone_${n}: LAND sa nepodarilo, pristan rucne v tmux (mode land)" >&2
    fi
  done
}

# Proces na pozadi ma v skripte SIGINT ignorovany, Ctrl+C/kill -INT by ho nezastavil
# (randomizer by nechal rovery jazdit). Obal vrati SIGINT na predvolene spravanie.
run_bg() {  # len s & na konci (exec nahradi podshell)
  exec python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' "$@"
}

rand_pid=""
rec_pid=""
stop_bg() {
  [[ -n "$rand_pid" ]] && kill -INT "$rand_pid" 2>/dev/null && wait "$rand_pid" 2>/dev/null
  rand_pid=""
}
stop_rec() {
  [[ -n "$rec_pid" ]] && kill -INT "$rec_pid" 2>/dev/null && wait "$rec_pid" 2>/dev/null
  rec_pid=""
}
interrupted=0
trap 'interrupted=1' INT

overall=0
results=()
for seed in "${seeds[@]}"; do
  out="$BAGS_DIR/seed${seed}_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$out"
  echo
  echo "================ seed $seed  ->  $out ================"

  echo "1) drony na zemi?"
  if ! wait_disarmed 120; then
    echo "Drony nie su pripravene, test seedu $seed preskakujem." >&2
    results+=("seed $seed: PRESKOCENY (drony neboli na zemi)")
    overall=1
    continue
  fi

  echo "2) rosbag, rovery (seed $seed), misia na ${duration} s"
  if [[ $bag -eq 1 ]]; then
    BAGS_DIR="$out" IMAGES=$images run_bg "$PKG_LIB/record.sh" bag > "$out/record.log" 2>&1 &
    rec_pid=$!
  fi
  run_bg "$RANDOMIZER" --seed "$seed" > "$out/randomize.log" 2>&1 &
  rand_pid=$!
  sleep 3
  if ! kill -0 "$rand_pid" 2>/dev/null; then
    echo "randomize_rovers.py skoncil, pozri $out/randomize.log:" >&2
    tail -5 "$out/randomize.log" >&2
    stop_rec
    exit 1
  fi

  # misia v popredi, log sa uklada; Ctrl+C ukonci len tento test
  RCUTILS_LOGGING_BUFFERED_STREAM=1 timeout -s INT -k 20 "$duration" \
    ros2 launch lrs_bringup follow.launch.py slow:=$slow debug:=$debug 2>&1 | tee "$out/follow.log"

  echo
  echo "3) koniec: zastavujem rovery, drony pristavaju"
  stop_bg
  land_all
  wait_disarmed 180 || echo "  pristan rucne v tmux (mode land) a pockaj na disarm" >&2
  stop_rec

  echo "4) vyhodnotenie"
  if "$PKG_LIB/analyze_follow_log.py" "$out/follow.log" --out "$out/summary.txt"; then
    results+=("seed $seed: PASS  ($out)")
  else
    results+=("seed $seed: FAIL  ($out)")
    overall=1
  fi
  if [[ $interrupted -eq 1 ]]; then
    echo "Prerusene (Ctrl+C), dalsie seedy nespustam."
    break
  fi
done

echo
echo "================ VYSLEDKY ================"
printf '%s\n' "${results[@]}"
exit $overall
