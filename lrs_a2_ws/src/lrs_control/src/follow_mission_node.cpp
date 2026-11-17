// TODO: odber target/pose, ~/abort, ~/estimate, ~/search_path
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

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

    if (p.search_alt < 2.0 || p.search_alt > 30.0) {
      throw std::runtime_error("search_alt mimo rozsahu 2..30 m");
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

    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {controlTick();}, control_group_);

    RCLCPP_INFO(get_logger(), "follow_mission: namespace '%s', sysid %d, vyska hladania %.1f m",
      get_namespace(), p.sysid, p.search_alt);
  }

private:
  void controlTick()
  {
    std::vector<lrs_control::TargetObs> obs;   // TODO: detekcie z target/pose
    executor_->tick(now().seconds(), obs);

    const auto s = executor_->state();
    if (s != last_state_) {
      last_state_ = s;
      std_msgs::msg::String m;
      m.data = FollowExecutor::name(s);
      state_pub_->publish(m);
    }
  }

  std::unique_ptr<lrs_control::MavrosVehicle> vehicle_;
  std::unique_ptr<FollowExecutor> executor_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::TimerBase::SharedPtr control_timer_;
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
