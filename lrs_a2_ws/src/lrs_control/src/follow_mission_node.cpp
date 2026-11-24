#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "lrs_control/follow_executor.hpp"
#include "lrs_control/mavros_vehicle.hpp"

using lrs_control::FollowExecutor;

class FollowMissionNode : public rclcpp::Node
{
public:
  FollowMissionNode()
  : Node("follow_mission")
  {
    const double rate = declare_parameter<double>("setpoint_rate", 20.0);

    lrs_control::FollowParams p;
    p.sysid = declare_parameter("sysid", p.sysid);
    p.search_alt = declare_parameter("search_alt", p.search_alt);
    p.follow_alt = declare_parameter("follow_alt", p.follow_alt);
    p.lost_timeout = declare_parameter("lost_timeout", p.lost_timeout);
    p.confirm_count = declare_parameter("confirm_count", p.confirm_count);
    p.confirm_window = declare_parameter("confirm_window", p.confirm_window);
    p.abort_mode = declare_parameter("abort_mode", p.abort_mode);
    p.takeoff_tol = declare_parameter("takeoff_tol", p.takeoff_tol);
    p.state_timeout = declare_parameter("state_timeout", p.state_timeout);
    p.pose_timeout = declare_parameter("pose_timeout", p.pose_timeout);
    p.request_retry = declare_parameter("request_retry", p.request_retry);
    p.mode_timeout = declare_parameter("mode_timeout", p.mode_timeout);
    p.arm_timeout = declare_parameter("arm_timeout", p.arm_timeout);
    p.takeoff_timeout = declare_parameter("takeoff_timeout", p.takeoff_timeout);
    p.search_speed = declare_parameter("search_speed", p.search_speed);
    p.search_accel = declare_parameter("search_accel", p.search_accel);
    p.search_overlap = declare_parameter("search_overlap", p.search_overlap);
    p.search_max_radius = declare_parameter("search_max_radius", p.search_max_radius);
    p.cam_hfov_deg = declare_parameter("cam_hfov_deg", p.cam_hfov_deg);
    p.cam_aspect = declare_parameter("cam_aspect", p.cam_aspect);
    p.track_alpha = declare_parameter("track_alpha", p.track_alpha);
    p.track_beta = declare_parameter("track_beta", p.track_beta);
    p.track_gate = declare_parameter("track_gate", p.track_gate);
    p.track_lead = declare_parameter("track_lead", p.track_lead);
    p.track_max_speed = declare_parameter("track_max_speed", p.track_max_speed);
    p.track_max_vz = declare_parameter("track_max_vz", p.track_max_vz);
    p.track_max_target_speed = declare_parameter("track_max_target_speed", p.track_max_target_speed);

    if (p.search_alt < 2.0 || p.search_alt > 30.0) {
      throw std::runtime_error("search_alt mimo rozsahu 2..30 m");
    }
    if (p.search_speed <= 0.0 || p.search_speed > 5.0) {
      throw std::runtime_error("search_speed mimo rozsahu 0..5 m/s");
    }
    if (p.track_alpha <= 0.0 || p.track_alpha > 1.0 || p.track_beta < 0.0 || p.track_beta > 1.0) {
      throw std::runtime_error("track_alpha musi byt 0..1, track_beta 0..1");
    }
    if (p.follow_alt < 2.0 || p.follow_alt > 30.0) {
      throw std::runtime_error("follow_alt mimo rozsahu 2..30 m");
    }
    if (p.abort_mode != "LOITER" && p.abort_mode != "BRAKE" && p.abort_mode != "RTL") {
      throw std::runtime_error("abort_mode musi byt LOITER, BRAKE alebo RTL (LAND nie)");
    }
    if (p.state_timeout < 1.5) {
      throw std::runtime_error("state_timeout < 1.5 s: mavros/state chodi len ~1 Hz");
    }

    // lokalne ENU drona z MAVROS bez transformacie; temy relativne k namespace uzla
    vehicle_ = std::make_unique<lrs_control::MavrosVehicle>(*this, lrs_control::FrameTransform{});
    executor_ = std::make_unique<FollowExecutor>(
      *vehicle_, p, [this](const std::string & m) {RCLCPP_INFO(get_logger(), "%s", m.c_str());});

    state_pub_ = create_publisher<std_msgs::msg::String>("~/state",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    path_pub_ = create_publisher<nav_msgs::msg::Path>("~/search_path",
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    est_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("~/estimate", 10);

    // detekcie vlastneho roveru (lokalne ENU drona, frame "map")
    target_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions target_opts;
    target_opts.callback_group = target_group_;
    target_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "target/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {onTarget(*m);}, target_opts);

    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {controlTick();}, control_group_);

    // ~/abort v tej istej skupine ako timer -> nikdy nebezi naraz s tick()
    abort_mode_ = p.abort_mode;
    abort_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/abort",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        executor_->abort("abort cez sluzbu ~/abort");
        res->success = true;
        res->message = std::string("ABORT -> ") + abort_mode_ + ", stav " +
        FollowExecutor::name(executor_->state());
      },
#if RCLCPP_VERSION_MAJOR >= 28
      rclcpp::ServicesQoS(),
#else
      rmw_qos_profile_services_default,
#endif
      control_group_);

    RCLCPP_INFO(get_logger(), "follow_mission: namespace '%s', sysid %d, vyska hladania %.1f m",
      get_namespace(), p.sysid, p.search_alt);
  }

private:
  void onTarget(const geometry_msgs::msg::PoseStamped & m)
  {
    if (m.header.frame_id != "map") {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "target/pose ma frame '%s', cakam 'map' -> ignorujem", m.header.frame_id.c_str());
      return;
    }
    lrs_control::TargetObs o;
    o.t = rclcpp::Time(m.header.stamp).seconds();
    o.p = {m.pose.position.x, m.pose.position.y, m.pose.position.z};
    std::lock_guard<std::mutex> lk(obs_mutex_);
    obs_buf_.push_back(o);
  }

  void controlTick()
  {
    // detekcie od posledneho ticku
    std::vector<lrs_control::TargetObs> obs;
    {
      std::lock_guard<std::mutex> lk(obs_mutex_);
      obs.swap(obs_buf_);
    }
    executor_->tick(now().seconds(), obs);

    const auto s = executor_->state();
    if (s != last_state_) {
      last_state_ = s;
      std_msgs::msg::String m;
      m.data = FollowExecutor::name(s);
      state_pub_->publish(m);
    }
    if (executor_->hasEstimate()) {
      geometry_msgs::msg::PointStamped e;
      e.header.stamp = now();
      e.header.frame_id = "map";
      const auto v = executor_->estimate();
      e.point.x = v.x;
      e.point.y = v.y;
      e.point.z = v.z;
      est_pub_->publish(e);
    }
    if (executor_->searchPathVersion() != path_version_) {
      path_version_ = executor_->searchPathVersion();
      publishSearchPath();
    }
  }

  // vzor pre RViz, lokalne ENU drona
  void publishSearchPath()
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = "map";
    for (const auto & v : executor_->searchPath()) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = path.header;
      ps.pose.position.x = v.x;
      ps.pose.position.y = v.y;
      ps.pose.position.z = v.z;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    path_pub_->publish(path);
  }

  std::unique_ptr<lrs_control::MavrosVehicle> vehicle_;
  std::unique_ptr<FollowExecutor> executor_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr est_pub_;
  int path_version_{0};
  rclcpp::CallbackGroup::SharedPtr target_group_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  std::mutex obs_mutex_;
  std::vector<lrs_control::TargetObs> obs_buf_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;
  std::string abort_mode_;
  FollowExecutor::State last_state_{FollowExecutor::State::ABORT};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<FollowMissionNode>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("follow_mission"), "%s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
