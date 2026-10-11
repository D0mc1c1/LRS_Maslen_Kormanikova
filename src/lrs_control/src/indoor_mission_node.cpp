// A1.2 indoor misia.
//
//   ros2 run lrs_control indoor_mission --ros-args
//     --params-file src/lrs_control/config/indoor.yaml
//     -p mission_file:=src/lrs_control/missions/test_open.csv
//   (v terminali na jeden riadok alebo s '\' na konci riadkov)
//
// Struktura vlakien (MultiThreadedExecutor):
//   control_group  : riadiaci timer (FSM + setpoint) a sluzba ~/abort
//                    -> MissionExecutor sa pouziva len z tejto skupiny, netreba mutex
//   sensor_group   : odbery MAVROS tem (v MavrosVehicle, zapis pod mutexom)
//   service_group  : odpovede MAVROS sluzieb (v MavrosVehicle)
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include <std_srvs/srv/trigger.hpp>

#include "lrs_control/geometry.hpp"
#include "lrs_control/mavros_vehicle.hpp"
#include "lrs_control/mission.hpp"
#include "lrs_control/mission_executor.hpp"
#include "lrs_planning/planner.hpp"

using lrs_control::MissionExecutor;

class IndoorMissionNode : public rclcpp::Node
{
public:
  IndoorMissionNode()
  : Node("indoor_mission")
  {
    const auto mission_file = declare_parameter<std::string>("mission_file", "");
    // map -> MAVROS local: local = R(yaw) * map + t  (pozri FrameTransform v geometry.hpp)
    const double tf_yaw_deg = declare_parameter<double>("local_from_map_yaw_deg", 90.0);
    const auto tf_xyz = declare_parameter<std::vector<double>>("local_from_map_xyz", {0.0, 0.0, 0.0});
    const auto yaw_frame = declare_parameter<std::string>("yaw_task_frame", "map");
    const auto planner_type = declare_parameter<std::string>("planner", "straight");
    const double rate = declare_parameter<double>("setpoint_rate", 20.0);

    lrs_control::ExecutorParams p;
    p.tol_hard = declare_parameter("tol_hard", p.tol_hard);
    p.tol_soft = declare_parameter("tol_soft", p.tol_soft);
    p.tol_pass = declare_parameter("tol_pass", p.tol_pass);
    p.v_settle = declare_parameter("v_settle", p.v_settle);
    p.settle_time = declare_parameter("settle_time", p.settle_time);
    p.yaw_tol_deg = declare_parameter("yaw_tol_deg", p.yaw_tol_deg);
    p.yaw_hold_time = declare_parameter("yaw_hold_time", p.yaw_hold_time);
    p.min_leg_xy = declare_parameter("min_leg_xy", p.min_leg_xy);
    p.takeoff_tol = declare_parameter("takeoff_tol", p.takeoff_tol);
    p.landtakeoff_wait = declare_parameter("landtakeoff_wait", p.landtakeoff_wait);
    p.arm_timeout = declare_parameter("arm_timeout", p.arm_timeout);
    p.segment_timeout = declare_parameter("segment_timeout", p.segment_timeout);

    // --- misia -------------------------------------------------------------
    if (mission_file.empty()) {
      throw std::runtime_error("chyba parameter mission_file");
    }
    lrs_control::Mission mission = lrs_control::loadMission(mission_file);
    RCLCPP_INFO(get_logger(), "misia %s: %zu waypointov", mission_file.c_str(), mission.size());
    for (std::size_t i = 0; i < mission.size(); ++i) {
      RCLCPP_INFO(get_logger(), "  WP%zu %s", i + 1, lrs_control::describe(mission[i]).c_str());
    }

    // --- planovac (A1.1) ---------------------------------------------------
    if (planner_type == "straight") {
      planner_ = std::make_unique<lrs_planning::StraightLinePlanner>();
      RCLCPP_WARN(get_logger(), "planner=straight: priamky BEZ kontroly kolizii, "
        "len pre misie v otvorenom priestore!");
    } else {
      // TODO (A1.1): sem pride A*, napr.
      //   planner_ = std::make_unique<lrs_planning::AStarPlanner>(map_path, voxel, inflation, ...);
      throw std::runtime_error("neznamy planner '" + planner_type + "' (zatial len 'straight')");
    }

    // --- dron + stavovy automat -------------------------------------------
    if (tf_xyz.size() != 3) {
      throw std::runtime_error("local_from_map_xyz musi mat 3 cisla");
    }
    lrs_control::FrameTransform tf;
    tf.yaw = lrs_control::deg2rad(tf_yaw_deg);
    tf.t = {tf_xyz[0], tf_xyz[1], tf_xyz[2]};
    vehicle_ = std::make_unique<lrs_control::MavrosVehicle>(*this, tf);

    // yawN z CSV: "map" = 0 deg je +X mapy, "enu" = 0 deg je vychod (MAVROS local +X)
    if (yaw_frame == "enu") {
      p.yaw_task_offset_deg = -tf_yaw_deg;
    } else if (yaw_frame != "map") {
      throw std::runtime_error("yaw_task_frame musi byt 'map' alebo 'enu'");
    }

    executor_ = std::make_unique<MissionExecutor>(
      *vehicle_, *planner_, mission, p,
      [this](MissionExecutor::LogLevel l, const std::string & m) {
        switch (l) {
          case MissionExecutor::LogLevel::Info: RCLCPP_INFO(get_logger(), "%s", m.c_str()); break;
          case MissionExecutor::LogLevel::Warn: RCLCPP_WARN(get_logger(), "%s", m.c_str()); break;
          case MissionExecutor::LogLevel::Error: RCLCPP_ERROR(get_logger(), "%s", m.c_str()); break;
        }
      });

    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {controlTick();}, control_group_);

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

    RCLCPP_INFO(get_logger(),
      "local = R(%.1f deg) * map + [%.2f, %.2f, %.2f], yawN v ramci '%s', %.0f Hz",
      tf_yaw_deg, tf_xyz[0], tf_xyz[1], tf_xyz[2], yaw_frame.c_str(), rate);
  }

private:
  void controlTick()
  {
    executor_->tick(now().seconds());
    if (executor_->state() == MissionExecutor::State::DONE && !finished_) {
      finished_ = true;
      RCLCPP_INFO(get_logger(), "misia dokoncena, koncim");
      rclcpp::shutdown();
    }
  }

  std::unique_ptr<lrs_planning::Planner> planner_;
  std::unique_ptr<lrs_control::MavrosVehicle> vehicle_;
  std::unique_ptr<MissionExecutor> executor_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  bool finished_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<IndoorMissionNode> node;
  try {
    node = std::make_shared<IndoorMissionNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("indoor_mission"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
