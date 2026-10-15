# LRS – Zadanie 1

## Build
```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/lrs_ws
colcon build --symlink-install
source install/setup.bash
```

## A1.1 – plánovač (bez simulátora)

```bash
MAP=~/LRS-URK/maps/FEI_LRS_PCD/map.pcd
ros2 run lrs_planning plan_cli $MAP 13 7 1 8.65 2.02 1 [radius=0.5]      # jeden segment
ros2 run lrs_planning plan_cli $MAP --mission src/lrs_control/missions/exam_example.csv
ros2 run lrs_planning plan_cli $MAP --query 5.945 6.818 2.45             # obsadenost bodu
```

## A1.2 – indoor misia

V `src/lrs_control/config/indoor.yaml` nastav `map_path`. Potom 3 terminály podľa LRS-URK README (Gazebo, SITL, MAVROS) a:

```bash
ros2 run lrs_control indoor_mission --ros-args \
  --params-file src/lrs_control/config/indoor.yaml \
  -p mission_file:=src/lrs_control/missions/exam_example.csv
```

- `mission_file` – ľubovoľné CSV `x,y,z,precision,task` (map frame)
- `planner:=astar` (default) – A* z A1.1; `planner:=straight` – priamky bez kontroly kolízií (len test)
- abort počas letu: `ros2 service call /indoor_mission/abort std_srvs/srv/Trigger`

### Offline test (bez ROS a simulátora)

```bash
SIM_MAP=$MAP ros2 run lrs_control mission_sim src/lrs_control/missions/exam_example.csv   # A* + stavovy automat
ros2 run lrs_control mission_sim src/lrs_control/missions/test_open.csv                   # priamky
```

## A1.3 – outdoor misia

### Simulácia (outdoor svet)

```bash
cd ~/LRS-URK && scripts/run_gazebo.sh fei_lrs_outdoor.world                          # terminál 1
cd ~/LRS-URK && HOME_LOCATION=48.15144444,17.07375000,150,0 scripts/run_sitl.sh      # terminál 2
cd ~/LRS-URK && scripts/run_mavros.sh                                                # terminál 3
```

### Časť 1: GPS, home, kompas

```bash
ros2 run lrs_control geo_test      # prevod LLA <-> ENU a kompasu, bez ROS
ros2 run lrs_control gps_info      # 1 Hz: lat/lon/alt, ENU od home, local position, kurz
```

