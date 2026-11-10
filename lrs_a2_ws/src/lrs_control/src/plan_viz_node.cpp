// plan_viz: naplanuje celu misiu cez A* (A1.1) a zobrazi ju v RViz2.
// Netreba Gazebo, SITL ani MAVROS.
//
// Prvy segment zacina v bode `start_xy` (spawn) vo vyske prveho waypointu,
// rovnako ako v indoor_mission.
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lrs_control/mission.hpp"
#include "lrs_control/viz.hpp"
#include "lrs_planning/astar_planner.hpp"

class PlanVizNode : public rclcpp::Node
{
public:
  PlanVizNode()
  : Node("plan_viz")
  {
    const auto mission_file = declare_parameter<std::string>("mission_file", "");
    const auto map_path = declare_parameter<std::string>("map_path", "");
    const double voxel = declare_parameter<double>("voxel_size", 0.10);
    const double radius = declare_parameter<double>("inflation_radius", 0.70);
    const double max_z = declare_parameter<double>("max_flight_z", 3.0);
    const auto rack_boxes = declare_parameter<std::vector<double>>("rack_boxes", std::vector<double>{});
    const auto start_xy = declare_parameter<std::vector<double>>("start_xy", {13.0, 7.0});
    const double viz_max_z = declare_parameter<double>("viz_max_z", 4.0);

    if (mission_file.empty() || map_path.empty()) {
      throw std::runtime_error("treba parametre mission_file a map_path");
    }
    if (start_xy.size() != 2) {
      throw std::runtime_error("start_xy musi mat 2 cisla");
    }
    const auto mission = lrs_control::loadMission(mission_file);
    const auto boxes = lrs_control::parseBoxes(rack_boxes);
    RCLCPP_INFO(get_logger(), "nacitavam mapu %s ...", map_path.c_str());
    lrs_planning::AStarPlanner planner(map_path, voxel, radius, boxes, max_z);

    // transient local: RViz dostane spravy aj ked sa spusti neskor
    const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/lrs/viz/map", latched);
    plan_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/lrs/viz/plan", latched);

    auto map_markers = lrs_control::mapMarkers(planner.map(), boxes, radius, viz_max_z, max_z);
    RCLCPP_INFO(get_logger(), "mapa: %zu voxelov stien, %zu voxelov strechy, %zu regalov",
      map_markers.markers.at(0).points.size(), map_markers.markers.at(1).points.size(), boxes.size());
    map_pub_->publish(map_markers);

    // planovanie vsetkych segmentov
    std::vector<lrs_planning::Path> segments;
    lrs_planning::Vec3 prev{start_xy[0], start_xy[1], mission.front().p.z};
    int ok = 0;
    for (std::size_t i = 0; i < mission.size(); ++i) {
      const auto r = planner.plan(prev, mission[i].p);
      if (r) {
        RCLCPP_INFO(get_logger(), "segment %zu -> WP%zu %s: OK %.0f ms, %zu -> %zu bodov",
          i, i + 1, lrs_control::describe(mission[i]).c_str(), r->time_ms, r->raw_points,
          r->path.size());
        segments.push_back(r->path);
        ++ok;
      } else {
        RCLCPP_ERROR(get_logger(), "segment %zu -> WP%zu %s: NEMA CESTU",
          i, i + 1, lrs_control::describe(mission[i]).c_str());
        segments.push_back({});  // prazdny segment, zvysne sa aj tak zobrazia
      }
      prev = mission[i].p;
    }
    plan_pub_->publish(lrs_control::planMarkers(mission, segments));
    RCLCPP_INFO(get_logger(), "%d / %zu segmentov OK. Zobrazene v RViz (/lrs/viz/*), Ctrl+C = koniec",
      ok, mission.size());
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plan_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<PlanVizNode> node;
  try {
    node = std::make_shared<PlanVizNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("plan_viz"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);  // drzi publishery nazive, kym bezi RViz
  rclcpp::shutdown();
  return 0;
}
