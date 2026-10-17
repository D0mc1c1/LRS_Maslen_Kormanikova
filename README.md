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


### Časť 2: vzlet, stabilné držanie, pristátie

Stavy: `WAIT_CONNECTION → WAIT_GPS → WAIT_ORIGIN → SET_MODE → ARM → TAKEOFF → HOLD → LAND → DONE`
(+ `ABORT`). Bez použiteľného GPS dron **nearmuje**. Ak pilot prepne mód z RC, program
prestane posielať setpointy a nič neprikazuje (nebojuje s pilotom).

```bash
ros2 run lrs_control outdoor_sim normal    # offline test, aj: nohome nogps rc abort
ros2 run lrs_control outdoor_mission --ros-args --params-file src/lrs_control/config/outdoor.yaml
ros2 service call /outdoor_mission/abort std_srvs/srv/Trigger    # núdzové pristátie
```

### Časť 3: štvorec s natáčaním na POI

Po vzlete sa dron natočí na POI, preletí na W1 a obletí štvorec W1→W2→W3→W4→W1 (S = 5 m,
stred = POI). Smer na POI sa počíta **každý tick** zo skutočnej polohy, len vodorovne
(yaw, nie gimbal). Úseky majú lichobežníkový profil rýchlosti; do setpointu ide aj rýchlosť
a rýchlosť otáčania ako feedforward. Na konci sa vráti nad miesto vzletu a pristane.

```bash
ros2 run lrs_control outdoor_sim normal    # aj: noff setpoint latlon nohome nogps geofence rc abort
ros2 run lrs_control outdoor_mission --ros-args --params-file src/lrs_control/config/outdoor.yaml
# POI cez lat/lon:  ... -p poi_lat:=48.1514867 -p poi_lon:=17.0737554
```

### Časť 4: osmička (časová trajektória)

Po štvorci presun do stredu nad POI (yaw sa cestou plynulo stočí na tangentu), potom
`x = A sin θ, y = A sin θ cos θ`, `θ = ω·(now − t_start)` – v každom ticku riadiaceho
timera (20 Hz) nový setpoint (poloha + rýchlosť + yaw = tangenta + yaw_rate). Yaw je
unwrapnutý. Rozbeh/dobeh fázy `fig8_ramp` odstráni skok rýchlosti na štarte.

```bash
ros2 run lrs_control outdoor_sim normal    # aj: noramp fast toofast noff abort ...
ros2 run lrs_control outdoor_mission --ros-args --params-file src/lrs_control/config/outdoor.yaml
# pomalšie:  ... -p fig8_omega:=0.15
```

### Časť 5: abort, zásah pilota, strata spojenia (indoor aj outdoor)

| Udalosť | Reakcia programu |
|---|---|
| `~/abort` alebo chyba misie (timeout, odchýlka) | prepne do `abort_mode` (outdoor LAND/RTL/BRAKE/LOITER, indoor vždy LAND); po potvrdení módu node skončí |
| pilot prepne mód z RC, disarm vo vzduchu | **prestane posielať setpointy, nič neprikazuje** (nebojuje s pilotom), node skončí |
| MAVROS spadne / FCU odpojené (`/mavros/state` > 3 s, poloha > 1 s) | to isté – prestane riadiť, ArduPilot drží pozíciu, ďalej pilot a failsafe |

```bash
ros2 service call /outdoor_mission/abort std_srvs/srv/Trigger
ros2 service call /indoor_mission/abort std_srvs/srv/Trigger
ros2 run lrs_control outdoor_sim rc          # aj: abort abortrtl linkloss
SIM_RC=30 ros2 run lrs_control mission_sim src/lrs_control/missions/test_open.csv   # aj SIM_ABORT, SIM_LINK
```

### Časť 6: rosbag a graf preletenej vs. prikázanej trasy

`outdoor_mission` publikuje fázu misie (`/lrs/outdoor/state`), POI, plánovanú trasu,
značky a preletenú trasu (`/lrs/outdoor/*`, frame `map` = MAVROS local ENU). Prikázaná
trajektória je `/mavros/setpoint_raw/local`, skutočná `/mavros/local_position/pose`.

```bash
# terminál 4: záznam (spustiť PRED misiou, Ctrl+C po pristátí)
ros2 run lrs_control record_outdoor.sh ~/lrs_bags/outdoor_run1
# terminál 5: misia
ros2 run lrs_control outdoor_mission --ros-args --params-file src/lrs_control/config/outdoor.yaml
# RViz (voliteľne)
rviz2 -d src/lrs_control/rviz/outdoor.rviz
# graf + súhrn (chyba na POI, odchýlka osmičky, yaw vs. tangenta, rozmery, geofence)
ros2 run lrs_control plot_outdoor.py ~/lrs_bags/outdoor_run1
# bez Gazeba: záznam z offline simulácie
SIM_CSV=/tmp/sim.csv ros2 run lrs_control outdoor_sim normal && ros2 run lrs_control plot_outdoor.py --csv /tmp/sim.csv
```

Výstup: `outdoor_xy.png` (pohľad zhora + detail osmičky), `outdoor_poi.png`,
`outdoor_fig8.png`, `outdoor_alt.png` v adresári bagu.
