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
