// TODO: falosny rover, publikovat target/pose
#include <memory>

#include <rclcpp/rclcpp.hpp>

class FakeTargetNode : public rclcpp::Node
{
public:
  FakeTargetNode()
  : Node("fake_target")
  {
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeTargetNode>());
  rclcpp::shutdown();
  return 0;
}
