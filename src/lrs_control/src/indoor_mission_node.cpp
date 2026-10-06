// Kostra indoor misie: overi, ze workspace, MAVROS a callback groups funguju.
// Stavovy automat (faza 2.5) sa doplni do control_tick().
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <mavros_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lrs_planning/voxel_map.hpp"

using namespace std::chrono_literals;

class IndoorMission : public rclcpp::Node
{
public:
  IndoorMission()
  : Node("indoor_mission")
  {
    mission_file_ = declare_parameter<std::string>("mission_file", "");
    map_path_ = declare_parameter<std::string>("map_path", "");

    sensor_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = sensor_group_;

    // SensorDataQoS (best effort) prijme spravy od reliable aj best-effort publishera.
    state_sub_ = create_subscription<mavros_msgs::msg::State>(
      "/mavros/state", rclcpp::SensorDataQoS(),
      [this](mavros_msgs::msg::State::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        state_ = *msg;
      }, sub_opts);

    control_timer_ = create_wall_timer(
      50ms, [this]() {control_tick();}, control_group_);

    RCLCPP_INFO(get_logger(), "mission_file='%s' map_path='%s'",
      mission_file_.c_str(), map_path_.c_str());
  }

private:
  void control_tick()
  {
    mavros_msgs::msg::State s;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      s = state_;
    }
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "connected=%d armed=%d mode=%s", s.connected, s.armed, s.mode.c_str());
    // TODO 2.5: switch (state) { ... } + publish setpointu
  }

  std::string mission_file_, map_path_;
  rclcpp::CallbackGroup::SharedPtr sensor_group_, control_group_;
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::mutex mtx_;
  mavros_msgs::msg::State state_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IndoorMission>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
