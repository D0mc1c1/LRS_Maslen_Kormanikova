# Find-and-follow

## Balíky

| Balík | Obsah |
|---|---|
| `lrs_planning` | plánovanie (A1) |
| `lrs_control` | riadenie (A1), `follow_mission`, `fake_target`, `follow_sim` |
| `lrs_vision` | `aruco_detector`, `marker_pose_test` |
| `lrs_bringup` | launch súbory, bridge kamier, skripty |

## Rozhranie (namespace `/drone_N`)

| Téma | Typ |
|---|---|
| `target/pose` | `geometry_msgs/PoseStamped` |
| `follow_mission/state` | `std_msgs/String` |
| `follow_mission/estimate` | `geometry_msgs/PointStamped` |
| `follow_mission/search_path` | `nav_msgs/Path` |
| `follow_mission/abort` | `std_srvs/Trigger` |

## Build

```bash
cd lrs_a2_ws
colcon build --symlink-install
source install/setup.bash
```

## Bring-up dvoch dronov (§1)

Každý krok vo vlastnom termináli, v poradí:

```bash
# 1. Gazebo (z koreňa repozitára LRS-URK)
scripts/run_gazebo.sh fei_lrs_precision_landing.world

# 2. Dva SITL v tmux (ArduCopter sa zbuilduje raz, potom oba s --no-rebuild)
ros2 run lrs_bringup sitl_2x.sh          # --no-build, ak je už zbuildovaný
tmux attach -t sitl2x                     # Ctrl+b n = ďalší dron, Ctrl+b d = odpojiť

# 3. 2x MAVROS (témy /drone_N/mavros/...) + bridge kamier
ros2 launch lrs_bringup bringup.launch.py

# 4. Kontrola §1: každá téma má publishera, MAVROS connected, fx z camera_info, Hz obrazu
ros2 run lrs_bringup check_bringup.sh
```

| | drone_1 | drone_2 |
|---|---|---|
| SITL | `--instance 0 --sysid 1` | `--instance 1 --sysid 2` |
| MAVLink → MAVROS | UDP 14550 | UDP 14560 |
| MAVROS uzol (namespace) | `drone_1` | `drone_2` |
| MAVROS témy | `/drone_1/mavros/...` | `/drone_2/mavros/...` |
| kamera | `/drone_1/down_camera/{image_raw,camera_info}` | `/drone_2/down_camera/{image_raw,camera_info}` |

Záznam: `ros2 run lrs_bringup record.sh [meno]` (obraz len s `IMAGES=1`).

## Misia find-and-follow (§2–§4)

Po bring-upe (`check_bringup.sh` = PASS), keď drony stoja na zemi:

```bash
ros2 launch lrs_bringup follow.launch.py                         # oba drony, detektor
ros2 launch lrs_bringup follow.launch.py drones:=1               # len drone_1
ros2 launch lrs_bringup follow.launch.py drones:=1 fake:=true    # fake_target namiesto detektora
ros2 launch lrs_bringup follow.launch.py slow:=true              # pomalá simulácia (dlhšie timeouty)

ros2 topic echo /drone_1/target/pose                             # detekcie vlastného roveru
ros2 service call /drone_1/follow_mission/abort std_srvs/srv/Trigger
```

Detektor `aruco_detector` (balík `lrs_vision`, parametre v `config/vision.yaml`):
publikuje `target/pose` len pre značku s `id == marker_id == N`, cudziu značku loguje
(„ignorujem cudziu znacku id …"), pri značke pod dronom loguje „NAD ROVEROM" (overenie §3)
a každých 10 s štatistiku snímok a času spracovania.
