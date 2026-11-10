// TODO: parametre, odbery kamery a polohy, ArUco detekcia, filter id, publikovat target/pose
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "lrs_vision/marker_pose.hpp"

class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode()
  : Node("aruco_detector")
  {
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDetectorNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
