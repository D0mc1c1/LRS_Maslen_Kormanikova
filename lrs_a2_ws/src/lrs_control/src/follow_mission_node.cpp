// TODO: parametre, MavrosVehicle v namespace, odber target/pose, timer, ~/abort
#include <memory>

#include <rclcpp/rclcpp.hpp>

class FollowMissionNode : public rclcpp::Node
{
public:
  FollowMissionNode()
  : Node("follow_mission")
  {
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowMissionNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
