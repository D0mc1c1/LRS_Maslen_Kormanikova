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

## A1.2 – indoor misia (3 terminály podľa LRS-URK README + tento)
```bash
ros2 run lrs_control indoor_mission --ros-args \
  --params-file src/lrs_control/config/indoor.yaml \
  -p mission_file:=src/lrs_control/missions/exam_example.csv
```
