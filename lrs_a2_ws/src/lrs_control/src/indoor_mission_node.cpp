// A1.2 indoor misia
//
// Spustenie uzla s konfiguraciou a suborom misie
//   ros2 run lrs_control indoor_mission --ros-args
//     --params-file src/lrs_control/config/indoor.yaml
//     -p mission_file:=src/lrs_control/missions/test_open.csv
//
// Rozdelenie vlakien pri MultiThreadedExecutor
//   control_group  : riadenie misie a sluzba na prerusenie
//   sensor_group   : prijimanie udajov z MAVROS
//   service_group  : spracovanie odpovedi sluzieb MAVROS

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lrs_control/geometry.hpp"
#include "lrs_control/mavros_vehicle.hpp"
#include "lrs_control/mission.hpp"
#include "lrs_control/mission_executor.hpp"
#include "lrs_control/viz.hpp"
#include "lrs_planning/astar_planner.hpp"
#include "lrs_planning/planner.hpp"

using lrs_control::MissionExecutor;

// Hlavny ROS 2 uzol pre vykonavanie vnutornej misie
class IndoorMissionNode : public rclcpp::Node
{
public:
  IndoorMissionNode()
  : Node("indoor_mission")
  {
    // Nacitanie parametrov misie a planovania
    const auto mission_file = declare_parameter<std::string>("mission_file", "");
    // Prevod suradnic medzi map a lokalnym systemom MAVROS
    const double tf_yaw_deg = declare_parameter<double>("local_from_map_yaw_deg", 90.0);
    const auto tf_xyz = declare_parameter<std::vector<double>>("local_from_map_xyz", {0.0, 0.0, 0.0});
    const auto yaw_frame = declare_parameter<std::string>("yaw_task_frame", "map");
    const auto planner_type = declare_parameter<std::string>("planner", "astar");
    const auto map_path = declare_parameter<std::string>("map_path", "");
    const double voxel_size = declare_parameter<double>("voxel_size", 0.10);
    const double viz_max_z = declare_parameter<double>("viz_max_z", 4.0);  // Vyska zobrazenej mapy v RViz
    const double inflation_radius = declare_parameter<double>("inflation_radius", 0.70);
    const double max_flight_z = declare_parameter<double>("max_flight_z", 3.0);
    // Regaly definovane pomocou hranicnych suradnic boxov
    const auto rack_boxes = declare_parameter<std::vector<double>>("rack_boxes", std::vector<double>{});
    const double rate = declare_parameter<double>("setpoint_rate", 20.0);

    // Parametre riadenia letu
    lrs_control::ExecutorParams p;
    p.tol_hard = declare_parameter("tol_hard", p.tol_hard);
    p.tol_soft = declare_parameter("tol_soft", p.tol_soft);
    p.tol_pass = declare_parameter("tol_pass", p.tol_pass);
    p.v_settle = declare_parameter("v_settle", p.v_settle);
    p.settle_time = declare_parameter("settle_time", p.settle_time);
    p.yaw_tol_deg = declare_parameter("yaw_tol_deg", p.yaw_tol_deg);
    p.yaw_hold_time = declare_parameter("yaw_hold_time", p.yaw_hold_time);
    p.min_leg_xy = declare_parameter("min_leg_xy", p.min_leg_xy);
    p.turn_stop_deg = declare_parameter("turn_stop_deg", p.turn_stop_deg);
    p.takeoff_tol = declare_parameter("takeoff_tol", p.takeoff_tol);
    p.landtakeoff_wait = declare_parameter("landtakeoff_wait", p.landtakeoff_wait);
    p.arm_timeout = declare_parameter("arm_timeout", p.arm_timeout);
    p.segment_timeout = declare_parameter("segment_timeout", p.segment_timeout);
    p.state_timeout = declare_parameter("state_timeout", p.state_timeout);
    p.pose_timeout = declare_parameter("pose_timeout", p.pose_timeout);

    // Nacitanie waypointov zo suboru misie
    if (mission_file.empty()) {
      throw std::runtime_error("chyba parameter mission_file");
    }
    lrs_control::Mission mission = lrs_control::loadMission(mission_file);
    RCLCPP_INFO(get_logger(), "misia %s: %zu waypointov", mission_file.c_str(), mission.size());
    // Vypis vsetkych waypointov do konzoly
    for (std::size_t i = 0; i < mission.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  WP%zu %s", i + 1, lrs_control::describe(mission[i]).c_str());
    }

    // Vytvorenie planovaca trasy
    std::vector<lrs_planning::AStarPlanner::Box> boxes;
    if (planner_type == "straight") {
      // Priamy let bez kontroly prekazok
      planner_ = std::make_unique<lrs_planning::StraightLinePlanner>();
      RCLCPP_WARN(get_logger(), "planner=straight: priamky BEZ kontroly kolizii, "
        "len pre misie v otvorenom priestore!");
    } else if (planner_type == "astar") {
      // AStar potrebuje mapu prostredia
      if (map_path.empty()) {
        throw std::runtime_error("planner=astar potrebuje parameter map_path");
      }
      boxes = lrs_control::parseBoxes(rack_boxes);
      auto astar = std::make_unique<lrs_planning::AStarPlanner>(
        map_path, voxel_size, inflation_radius, boxes, max_flight_z);
      astar_ = astar.get();
      planner_ = std::move(astar);
      RCLCPP_INFO(get_logger(),
        "planner=astar: mapa %s, voxel %.2f m, inflacia %.2f m, strop %.2f m, %zu boxov",
        map_path.c_str(), voxel_size, inflation_radius, max_flight_z, boxes.size());
    } else {
      // Neznamy typ planovaca
      throw std::runtime_error("neznamy planner '" + planner_type + "' (straight / astar)");
    }

    // Vytvorenie prevodu suradnic a rozhrania pre dron
    if (tf_xyz.size() != 3) {
      throw std::runtime_error("local_from_map_xyz musi mat 3 cisla");
    }
    lrs_control::FrameTransform tf;
    tf.yaw = lrs_control::deg2rad(tf_yaw_deg);
    tf.t = {tf_xyz[0], tf_xyz[1], tf_xyz[2]};
    vehicle_ = std::make_unique<lrs_control::MavrosVehicle>(*this, tf);

    // Nastavenie referencneho systemu pre ulohy natocenia
    if (yaw_frame == "enu") {
      p.yaw_task_offset_deg = -tf_yaw_deg;
    } else if (yaw_frame != "map") {
      throw std::runtime_error("yaw_task_frame musi byt 'map' alebo 'enu'");
    }

    // Vytvorenie vykonavaca misie a jeho logovania
    executor_ = std::make_unique<MissionExecutor>(
      *vehicle_, *planner_, mission, p,
      [this](MissionExecutor::LogLevel l, const std::string & m) {
        switch (l) {
          case MissionExecutor::LogLevel::Info: RCLCPP_INFO(get_logger(), "%s", m.c_str()); break;
          case MissionExecutor::LogLevel::Warn: RCLCPP_WARN(get_logger(), "%s", m.c_str()); break;
          case MissionExecutor::LogLevel::Error: RCLCPP_ERROR(get_logger(), "%s", m.c_str()); break;
        }
      });

    // Nastavenie vizualizacie mapy a misie v RViz2
    mission_ = mission;
    const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/lrs/viz/map", latched);
    plan_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/lrs/viz/plan", latched);
    flown_pub_ = create_publisher<nav_msgs::msg::Path>("/lrs/viz/flown", 10);
    flown_.header.frame_id = "map";
    // Zobrazenie mapy s prekazkami a hranicou vysky
    if (astar_) {
      map_pub_->publish(lrs_control::mapMarkers(astar_->map(), boxes, inflation_radius, viz_max_z,
        max_flight_z));
    }
    // Pred vypoctom trasy sa zobrazia waypointy
    plan_pub_->publish(lrs_control::planMarkers(mission_, {}));  // zatial len waypointy

    // Pravidelne spustanie riadiacej slucky
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {controlTick();}, control_group_);

    // Sluzba na prerusenie misie
    abort_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/abort",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        executor_->abort("abort cez sluzbu ~/abort");
        res->success = true;
        res->message = "ABORT -> LAND";
      },
#if RCLCPP_VERSION_MAJOR >= 28
      rclcpp::ServicesQoS(),
#else
      rmw_qos_profile_services_default,
#endif
      control_group_);

    // Vypis pouziteho nastavenia suradnic a frekvencie riadenia
    RCLCPP_INFO(get_logger(),
      "local = R(%.1f deg) * map + [%.2f, %.2f, %.2f], yawN v ramci '%s', %.0f Hz",
      tf_yaw_deg, tf_xyz[0], tf_xyz[1], tf_xyz[2], yaw_frame.c_str(), rate);
  }

private:
  // Jeden krok riadenia misie a kontrola jej ukoncenia
  void controlTick()
  {
    executor_->tick(now().seconds());
    publishViz();
    // Ukoncenie uzla po skonceni misie
    if (executor_->finished() && !finished_) {
      finished_ = true;
      RCLCPP_INFO(get_logger(), "misia skoncila v stave %s, koncim",
        MissionExecutor::name(executor_->state()));
      rclcpp::shutdown();
    }
  }

  // Aktualizacia planu a prejdenej trasy v RViz
  void publishViz()
  {
    // Po vypocte trasy sa zobrazi naplanovana cesta
    if (!plan_published_ && !executor_->segments().empty()) {
      plan_pub_->publish(lrs_control::planMarkers(mission_, executor_->segments()));
      plan_published_ = true;
    }
    // Trasa letu sa aktualizuje kazdym stvrtym tickom
    if (++viz_tick_ % 4 != 0) return;
    const auto st = vehicle_->state();
    if (!st.has_pose || flown_.poses.size() > 20000) return;
    // Vytvorenie bodu aktualnej polohy pre RViz
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = now();
    ps.pose.position.x = st.pos.x;
    ps.pose.position.y = st.pos.y;
    ps.pose.position.z = st.pos.z;
    ps.pose.orientation.z = std::sin(st.yaw / 2.0);
    ps.pose.orientation.w = std::cos(st.yaw / 2.0);
    flown_.header.stamp = ps.header.stamp;
    flown_.poses.push_back(ps);
    flown_pub_->publish(flown_);
  }

  // Ukazovatel na planovac AStar ktory vlastni planner_
  lrs_planning::AStarPlanner * astar_{nullptr};  // vlastni ho planner_
  lrs_control::Mission mission_;
  // Publisher pre mapu plan a prejdenu trasu
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr plan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr flown_pub_;
  nav_msgs::msg::Path flown_;
  bool plan_published_{false};
  unsigned viz_tick_{0};

  // Objekty planovania a komunikacie s dronom
  std::unique_ptr<lrs_planning::Planner> planner_;
  std::unique_ptr<lrs_control::MavrosVehicle> vehicle_;
  std::unique_ptr<MissionExecutor> executor_;
  // Skupina callbackov pre riadenie misie
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  bool finished_{false};
};

// Vstupny bod programu
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<IndoorMissionNode> node;
  // Zachytenie chyby pri vytvarani uzla
  try {
    node = std::make_shared<IndoorMissionNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("indoor_mission"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  // Spustenie uzla vo viacvlaknovom executore
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
