#pragma once
// Vizualizacia pre RViz2 (frame "map"): mapa ako voxely, regaly, plan cesty.
// Pouziva indoor_mission aj plan_viz. Spolocne temy, aby stacil jeden .rviz:
//   /lrs/viz/map    (MarkerArray, transient local)  obsadene voxely + regaly
//   /lrs/viz/plan   (MarkerArray, transient local)  waypointy + cesty z A*
//   /lrs/viz/flown  (nav_msgs/Path)                 skutocne preletena trasa
#include <limits>
#include <vector>

#include <visualization_msgs/msg/marker_array.hpp>

#include "lrs_control/mission.hpp"
#include "lrs_planning/astar_planner.hpp"
#include "lrs_planning/voxel_map.hpp"

namespace lrs_control
{

// Obsadene voxely mapy (bez podlahy) + regaly ako plne a inflovane boxy.
// Voxely nad roof_z idu do samostatneho priesvitneho namespace "strecha"
// (v RViz: Displays -> Mapa a regaly -> Namespaces sa da vypnut).
// flight_ceiling_z = strop letu planovaca, kresli sa ako priesvitna rovina (NaN = nekreslit).
visualization_msgs::msg::MarkerArray mapMarkers(
  const lrs_planning::VoxelMap & map,
  const std::vector<lrs_planning::AStarPlanner::Box> & boxes,
  double inflation_radius, double roof_z,
  double flight_ceiling_z = std::numeric_limits<double>::quiet_NaN());

// Waypointy misie (gula + popis) a naplanovane segmenty (ciara + body).
// segments moze byt prazdne -> len waypointy.
visualization_msgs::msg::MarkerArray planMarkers(
  const Mission & mission, const std::vector<lrs_planning::Path> & segments);

// Parameter rack_boxes [x0,y0,z0,x1,y1,z1, ...] -> boxy. Vyhodi pri zlej dlzke.
std::vector<lrs_planning::AStarPlanner::Box> parseBoxes(const std::vector<double> & flat);

}  // namespace lrs_control
