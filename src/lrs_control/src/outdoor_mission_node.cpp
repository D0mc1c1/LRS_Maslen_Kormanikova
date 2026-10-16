// A1.3 outdoor misia.
//
//   ros2 run lrs_control outdoor_mission --ros-args
//     --params-file src/lrs_control/config/outdoor.yaml
//   (v terminali na jeden riadok alebo s '\' na konci riadkov)
//
// GPS -> home -> GUIDED -> arm -> vzlet na `altitude` -> drzanie -> natocenie na POI
// -> stvorec okolo POI (nos stale na POI) -> osmicka nad POI (casova trajektoria,
// yaw = tangenta) -> navrat nad miesto vzletu -> LAND.
//
// Osmicka: riadiaci timer (setpoint_rate, 20 Hz) v kazdom ticku vola executor, ktory
// vypocita t = omega * (now - t_start) a z rovnic novy setpoint (poloha, rychlost, yaw).
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

    // POI: ak je zadane poi_lat aj poi_lon (nie 0), pouzije sa lat/lon, inak posun
    p.poi_lat = declare_parameter("poi_lat", 0.0);
    p.poi_lon = declare_parameter("poi_lon", 0.0);
    p.poi_use_latlon = p.poi_lat != 0.0 && p.poi_lon != 0.0;
    p.poi_east = declare_parameter("poi_east", p.poi_east);
    p.poi_north = declare_parameter("poi_north", p.poi_north);
    p.square_side = declare_parameter("square_side", p.square_side);
    p.speed = declare_parameter("speed", p.speed);
    p.accel = declare_parameter("accel", p.accel);
    p.corner_tol = declare_parameter("corner_tol", p.corner_tol);
    p.leg_timeout_extra = declare_parameter("leg_timeout_extra", p.leg_timeout_extra);
    const auto yaw_src = declare_parameter<std::string>("poi_yaw_source", "position");
    p.poi_min_dist = declare_parameter("poi_min_dist", p.poi_min_dist);
    p.align_tol_deg = declare_parameter("align_tol_deg", p.align_tol_deg);
    p.align_time = declare_parameter("align_time", p.align_time);
    p.velocity_ff = declare_parameter("velocity_ff", p.velocity_ff);
    p.yaw_rate_ff = declare_parameter("yaw_rate_ff", p.yaw_rate_ff);
    p.geofence_radius = declare_parameter("geofence_radius", p.geofence_radius);
    p.fig8_amplitude = declare_parameter("fig8_amplitude", p.fig8_amplitude);
    p.fig8_omega = declare_parameter("fig8_omega", p.fig8_omega);
    p.fig8_laps = declare_parameter("fig8_laps", p.fig8_laps);
    p.fig8_ramp = declare_parameter("fig8_ramp", p.fig8_ramp);
    p.fig8_rotation_deg = declare_parameter("fig8_rotation_deg", p.fig8_rotation_deg);
    p.fig8_max_speed = declare_parameter("fig8_max_speed", p.fig8_max_speed);
    p.max_track_err = declare_parameter("max_track_err", p.max_track_err);
    p.abort_mode = declare_parameter("abort_mode", p.abort_mode);
    p.state_timeout = declare_parameter("state_timeout", p.state_timeout);
    p.pose_timeout = declare_parameter("pose_timeout", p.pose_timeout);
    p.request_retry = declare_parameter("request_retry", p.request_retry);
    p.mode_timeout = declare_parameter("mode_timeout", p.mode_timeout);
    p.arm_timeout = declare_parameter("arm_timeout", p.arm_timeout);
    p.takeoff_timeout = declare_parameter("takeoff_timeout", p.takeoff_timeout);
    p.hold_timeout = declare_parameter("hold_timeout", p.hold_timeout);
    p.land_timeout = declare_parameter("land_timeout", p.land_timeout);

    if (p.altitude < 1.0 || p.altitude > 30.0) {
      throw std::runtime_error("altitude mimo rozsahu 1..30 m");
    }
    if (yaw_src != "position" && yaw_src != "setpoint") {
      throw std::runtime_error("poi_yaw_source musi byt 'position' alebo 'setpoint'");
    }
    p.poi_yaw_from_setpoint = yaw_src == "setpoint";
    if (p.speed <= 0.0 || p.speed > 3.0 || p.accel <= 0.0) {
      throw std::runtime_error("speed musi byt v (0, 3] m/s a accel > 0");
    }
    if (p.fig8_omega <= 0.0 || p.fig8_laps < 1.0 || p.fig8_ramp < 0.0 || p.fig8_amplitude <= 0.0) {
      throw std::runtime_error("fig8: omega > 0, laps >= 1, ramp >= 0, amplitude > 0");
    }
    if (p.abort_mode != "LAND" && p.abort_mode != "RTL" && p.abort_mode != "BRAKE" &&
      p.abort_mode != "LOITER")
    {
      throw std::runtime_error("abort_mode musi byt LAND, RTL, BRAKE alebo LOITER");
    }
    if (p.state_timeout < 1.5) {
      throw std::runtime_error("state_timeout < 1.5 s: /mavros/state chodi len ~1 Hz");
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
        res->message = "ABORT -> " + abort_mode_;
      },
#if RCLCPP_VERSION_MAJOR >= 28
      rclcpp::ServicesQoS(),
#else
      rmw_qos_profile_services_default,
#endif
      control_group_);

    abort_mode_ = p.abort_mode;
    RCLCPP_INFO(get_logger(), "abort: ~/abort -> %s, spojenie stratene po %.1f s (state) / "
      "%.1f s (poloha)", p.abort_mode.c_str(), p.state_timeout, p.pose_timeout);
    RCLCPP_INFO(get_logger(), "outdoor misia: vyska %.1f m, stvorec %.1f m, %.1f m/s, POI %s, "
      "yaw na POI zo %s, feedforward v=%d yaw_rate=%d, %.0f Hz",
      p.altitude, p.square_side, p.speed, p.poi_use_latlon ? "lat/lon" : "posun",
      yaw_src.c_str(), p.velocity_ff, p.yaw_rate_ff, rate);
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

  // stav, poloha od miesta vzletu, rel_alt, lat/lon, kurz v oboch konvenciach, chyba na POI
  void printStatus()
  {
    const auto st = vehicle_->state();
    const auto & l = executor_->launch();
    char b[320];
    std::snprintf(b, sizeof(b),
      "[%s] %s armed=%d | od vzletu E %.2f N %.2f U %.2f | rel_alt %s | "
      "lat %.7f lon %.7f | kompas %.0f deg, ENU yaw %.0f deg%s",
      OutdoorExecutor::name(executor_->state()), st.mode.c_str(), st.armed ? 1 : 0,
      st.pos.x - l.x, st.pos.y - l.y, st.pos.z - l.z,
      st.has_rel_alt ? (std::to_string(st.rel_alt).substr(0, 5) + " m").c_str() : "?",
      st.gps.lat_deg, st.gps.lon_deg, st.compass_hdg_deg, lrs_control::rad2deg(st.yaw),
      executor_->poiErrDeg() >= 0.0 ?
      (" | chyba na POI " + std::to_string(executor_->poiErrDeg()).substr(0, 4) + " deg").c_str() :
      executor_->trackErr() >= 0.0 ?
      (" | odchylka od osmicky " + std::to_string(executor_->trackErr()).substr(0, 4) + " m").c_str() :
      "");
    RCLCPP_INFO(get_logger(), "%s", b);
  }

  std::unique_ptr<lrs_control::MavrosVehicle> vehicle_;
  std::unique_ptr<OutdoorExecutor> executor_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  std::string abort_mode_;
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
