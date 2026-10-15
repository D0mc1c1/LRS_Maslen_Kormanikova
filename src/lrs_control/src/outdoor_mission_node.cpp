// A1.3 outdoor misia.
//
//   ros2 run lrs_control outdoor_mission --ros-args
//     --params-file src/lrs_control/config/outdoor.yaml
//   (v terminali na jeden riadok alebo s '\' na konci riadkov)
//
// Cast 2: GPS -> home -> GUIDED -> arm -> vzlet na `altitude` -> stabilne drzanie -> LAND.
// Ramec: MAVROS local ENU bez transformacie (FrameTransform{} = identita).
//
// Vlakna rovnako ako indoor_mission:
//   control_group : riadiaci timer + sluzba ~/abort (OutdoorExecutor len odtialto)
//   sensor_group  : odbery MAVROS tem (MavrosVehicle)
//   service_group : odpovede MAVROS sluzieb (MavrosVehicle)
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include <std_srvs/srv/trigger.hpp>

#include "lrs_control/geo.hpp"
#include "lrs_control/geometry.hpp"
#include "lrs_control/mavros_vehicle.hpp"
#include "lrs_control/outdoor_executor.hpp"

using lrs_control::OutdoorExecutor;

class OutdoorMissionNode : public rclcpp::Node
{
public:
  OutdoorMissionNode()
  : Node("outdoor_mission")
  {
    const double rate = declare_parameter<double>("setpoint_rate", 20.0);
    status_period_ = declare_parameter<double>("status_period", 2.0);

    lrs_control::OutdoorParams p;
    p.altitude = declare_parameter("altitude", p.altitude);
    p.min_satellites = declare_parameter("min_satellites", p.min_satellites);
    p.max_h_acc = declare_parameter("max_h_acc", p.max_h_acc);
    p.gps_settle_time = declare_parameter("gps_settle_time", p.gps_settle_time);
    p.home_wait = declare_parameter("home_wait", p.home_wait);
    p.takeoff_tol = declare_parameter("takeoff_tol", p.takeoff_tol);
    p.hold_tol = declare_parameter("hold_tol", p.hold_tol);
    p.hold_v = declare_parameter("hold_v", p.hold_v);
    p.hold_time = declare_parameter("hold_time", p.hold_time);
    p.request_retry = declare_parameter("request_retry", p.request_retry);
    p.mode_timeout = declare_parameter("mode_timeout", p.mode_timeout);
    p.arm_timeout = declare_parameter("arm_timeout", p.arm_timeout);
    p.takeoff_timeout = declare_parameter("takeoff_timeout", p.takeoff_timeout);
    p.hold_timeout = declare_parameter("hold_timeout", p.hold_timeout);
    p.land_timeout = declare_parameter("land_timeout", p.land_timeout);

    if (p.altitude < 1.0 || p.altitude > 30.0) {
      throw std::runtime_error("altitude mimo rozsahu 1..30 m");
    }

    // outdoor: pracujeme priamo v MAVROS local ENU
    vehicle_ = std::make_unique<lrs_control::MavrosVehicle>(*this, lrs_control::FrameTransform{});
    executor_ = std::make_unique<OutdoorExecutor>(
      *vehicle_, p,
      [this](OutdoorExecutor::LogLevel l, const std::string & m) {
        switch (l) {
          case OutdoorExecutor::LogLevel::Info: RCLCPP_INFO(get_logger(), "%s", m.c_str()); break;
          case OutdoorExecutor::LogLevel::Warn: RCLCPP_WARN(get_logger(), "%s", m.c_str()); break;
          case OutdoorExecutor::LogLevel::Error: RCLCPP_ERROR(get_logger(), "%s", m.c_str()); break;
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

    RCLCPP_INFO(get_logger(), "outdoor misia: vzlet na %.1f m, drzanie %.0f s, %.0f Hz",
      p.altitude, p.hold_time, rate);
  }

private:
  void controlTick()
  {
    const double t = now().seconds();
    executor_->tick(t);
    if (t - last_status_ >= status_period_) {
      last_status_ = t;
      printStatus();
    }
    if (executor_->finished() && !finished_) {
      finished_ = true;
      RCLCPP_INFO(get_logger(), "misia skoncila v stave %s, koncim",
        OutdoorExecutor::name(executor_->state()));
      rclcpp::shutdown();
    }
  }

  // stav, poloha od miesta vzletu, rel_alt, lat/lon, kurz
  void printStatus()
  {
    const auto st = vehicle_->state();
    const auto & l = executor_->launch();
    char b[256];
    std::snprintf(b, sizeof(b),
      "[%s] %s armed=%d | od vzletu E %.2f N %.2f U %.2f | rel_alt %s | "
      "lat %.7f lon %.7f | kompas %.0f deg",
      OutdoorExecutor::name(executor_->state()), st.mode.c_str(), st.armed ? 1 : 0,
      st.pos.x - l.x, st.pos.y - l.y, st.pos.z - l.z,
      st.has_rel_alt ? (std::to_string(st.rel_alt).substr(0, 5) + " m").c_str() : "?",
      st.gps.lat_deg, st.gps.lon_deg, st.compass_hdg_deg);
    RCLCPP_INFO(get_logger(), "%s", b);
  }

  std::unique_ptr<lrs_control::MavrosVehicle> vehicle_;
  std::unique_ptr<OutdoorExecutor> executor_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  double status_period_{2.0};
  double last_status_{0.0};
  bool finished_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<OutdoorMissionNode> node;
  try {
    node = std::make_shared<OutdoorMissionNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("outdoor_mission"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
