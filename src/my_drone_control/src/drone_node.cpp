#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/command_bool.hpp>

using namespace std::chrono_literals;

class DroneNode : public rclcpp::Node
{
public:
  DroneNode() : Node("drone_node")
  {
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "mavros/local_position/pose", rclcpp::SensorDataQoS(),
      std::bind(&DroneNode::pose_cb, this, std::placeholders::_1));

    sp_pub_ = create_publisher<mavros_msgs::msg::PositionTarget>(
      "mavros/setpoint_raw/local", 10);
    timer_ = create_wall_timer(50ms, std::bind(&DroneNode::send_setpoint, this));

  }

private:
  void pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "x=%.2f y=%.2f z=%.2f",
      msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  }

  void send_setpoint()
  {
    using PT = mavros_msgs::msg::PositionTarget;
    PT sp;
    sp.header.stamp = now();
    sp.coordinate_frame = PT::FRAME_LOCAL_NED;
    sp.type_mask = PT::IGNORE_VX | PT::IGNORE_VY | PT::IGNORE_VZ |
                   PT::IGNORE_AFX | PT::IGNORE_AFY | PT::IGNORE_AFZ |
                   PT::IGNORE_YAW_RATE;
    sp.position.x = 5.0;
    sp.position.y = 3.0;
    sp.position.z = 4.0;
    sp.yaw = 0.0;
    sp_pub_->publish(sp);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr sp_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_cli_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DroneNode>());
  rclcpp::shutdown();
  return 0;
}