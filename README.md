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
ros2 run lrs_planning plan_cli <cesta>/LRS-URK/maps/FEI_LRS_PCD/map.pcd
```

## A1.2 – indoor misia

Najprv 3 terminály podľa LRS-URK README (Gazebo, SITL, MAVROS), potom:

```bash
ros2 run lrs_control indoor_mission --ros-args \
  --params-file src/lrs_control/config/indoor.yaml \
  -p mission_file:=src/lrs_control/missions/test_open.csv
```

- `mission_file` – ľubovoľné CSV `x,y,z,precision,task` (map frame)
- `planner:=straight` – dočasne priamky bez kontroly kolízií, len pre `test_open.csv`
- abort počas letu: `ros2 service call /indoor_mission/abort std_srvs/srv/Trigger`

### Offline test stavového automatu (bez ROS a simulátora)

```bash
ros2 run lrs_control mission_sim src/lrs_control/missions/exam_example.csv
SIM_DETOUR=1 ros2 run lrs_control mission_sim src/lrs_control/missions/exam_example.csv  # s medzibodmi
```
