#!/usr/bin/env bash
# Dva ArduPilot SITL pre svet fei_lrs_precision_landing (drone_1 a drone_2).
#
# Kazdy SITL bezi vo vlastnom okne tmux: MAVProxy potrebuje interaktivny terminal (TTY),
# na pozadi bez TTY hned skonci a vyzera to ako "SITL neposiela heartbeat".
# ArduCopter sa zbuilduje raz pred spustenim, potom oba SITL bezia s --no-rebuild,
# aby sa dva buildy nebili o spolocny adresar ardupilot/build.
#
#   drone_1: --instance 0 --sysid 1, FDM 9002, MAVLink UDP 14550
#   drone_2: --instance 1 --sysid 2, FDM 9012, MAVLink UDP 14560
#
# Pouzitie:
#   ros2 run lrs_bringup sitl_2x.sh            # build (ak treba) + oba SITL v tmux
#   ros2 run lrs_bringup sitl_2x.sh --no-build # bez buildu
#   tmux attach -t sitl2x                      # okna: drone_1, drone_2 (Ctrl+b n = dalsie)
#   tmux kill-session -t sitl2x                # zastavit oba
#
# Premenne prostredia:
#   ARDUPILOT_DIR  (predvolene ~/ardupilot)
#   SITL_STREAMRATE (predvolene 20 Hz): ako casto MAVProxy ziada od ArduPilota polohu a stav.
#     MAVProxy ma predvolene 4 Hz simulacneho casu; pri pomalej simulacii (RTF ~0.25) to je
#     ~1 Hz realne a follow_mission aj detektor maju staru polohu (falosny ABORT vzletu).
#   ARDUPILOT_VENV (predvolene ~/venv-ardupilot, ak existuje, aktivuje sa)
set -euo pipefail

ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"
ARDUPILOT_VENV="${ARDUPILOT_VENV:-$HOME/venv-ardupilot}"
SESSION="sitl2x"
STREAMRATE="${SITL_STREAMRATE:-20}"

# domovske polohy = skutocne miesta dronov vo svete (-10,-10) a (10,-10), zo zadania
HOME_1="48.15135451,17.07361560,150,0"
HOME_2="48.15135451,17.07388440,150,0"

build=1
if [[ "${1:-}" == "--no-build" ]]; then
  build=0
fi

SIM_VEHICLE="$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py"
PARM="$ARDUPILOT_DIR/Tools/autotest/default_params/gazebo-iris.parm"
if [[ ! -f "$SIM_VEHICLE" ]]; then
  echo "sim_vehicle.py nenajdeny v ARDUPILOT_DIR=$ARDUPILOT_DIR" >&2
  exit 1
fi
if ! command -v tmux >/dev/null; then
  echo "chyba tmux: sudo apt install tmux" >&2
  exit 1
fi
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "tmux session '$SESSION' uz bezi. Zastav ju: tmux kill-session -t $SESSION" >&2
  exit 1
fi

activate=""
if [[ -f "$ARDUPILOT_VENV/bin/activate" ]]; then
  activate="source '$ARDUPILOT_VENV/bin/activate' && "
fi

if [[ $build -eq 1 ]]; then
  echo "== build ArduCopter (raz, pre oba SITL) =="
  (
    cd "$ARDUPILOT_DIR"
    if [[ -f "$ARDUPILOT_VENV/bin/activate" ]]; then
      # shellcheck disable=SC1091
      source "$ARDUPILOT_VENV/bin/activate"
    fi
    ./waf configure --board sitl >/dev/null
    ./waf copter
  )
fi

sitl_cmd() {
  local inst="$1" sysid="$2" home="$3"
  echo "${activate}cd '$ARDUPILOT_DIR/ArduCopter' && python3 '$SIM_VEHICLE' \
-v ArduCopter -f gazebo-iris --model JSON --add-param-file='$PARM' \
--instance $inst --sysid $sysid --use-dir /tmp/ardupilot-drone$sysid \
--no-rebuild --console -l $home --mavproxy-args='--streamrate=$STREAMRATE'"
}

mkdir -p /tmp/ardupilot-drone1 /tmp/ardupilot-drone2
tmux new-session -d -s "$SESSION" -n drone_1 "$(sitl_cmd 0 1 "$HOME_1"); bash"
tmux new-window -t "$SESSION" -n drone_2 "$(sitl_cmd 1 2 "$HOME_2"); bash"

echo "Oba SITL bezia v tmux session '$SESSION' (MAVLink streamrate $STREAMRATE Hz)."
echo "  tmux attach -t $SESSION      (Ctrl+b n = dalsie okno, Ctrl+b d = odpojit)"
echo "  tmux kill-session -t $SESSION"
