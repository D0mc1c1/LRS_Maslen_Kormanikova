// Falosny rover namiesto detektora: rover ide po kruznici v lokalnom ENU drona a
// target/pose sa publikuje len vtedy, ked je rover v zabere kamery (ako skutocny detektor).
// Stred kruznice = poloha drona pri starte + (offset_x, offset_y).
#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lrs_control/search_pattern.hpp"

namespace
{
double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}

class FakeTargetNode : public rclcpp::Node
{
public:
  FakeTargetNode()
  : Node("fake_target")
  {
    offset_x_ = declare_parameter("offset_x", 10.0);    // [m] stred kruznice od startu drona
    offset_y_ = declare_parameter("offset_y", 8.0);
    radius_ = declare_parameter("radius", 3.0);         // [m]
    speed_ = declare_parameter("speed", 0.3);           // [m/s]
    noise_ = declare_parameter("noise", 0.1);           // [m] sigma sumu detekcie
    const double rate = declare_parameter("rate", 10.0);  // [Hz] ako kamera
    cam_.cam_hfov_deg = declare_parameter("cam_hfov_deg", 60.0);
    cam_.cam_aspect = declare_parameter("cam_aspect", 0.75);

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        drone_ = *m;
        has_drone_ = true;
      });
    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("target/pose", 10);
    truth_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/truth", 10);
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this]() {tick();});

    RCLCPP_INFO(get_logger(), "fake_target: kruznica r %.1f m, %.2f m/s, stred start + (%.1f, %.1f)",
      radius_, speed_, offset_x_, offset_y_);
  }

private:
  void tick()
  {
    if (!has_drone_) {
      return;
    }
    const auto & dp = drone_.pose.position;
    if (!started_) {
      started_ = true;
      t0_ = now().seconds();
      ground_z_ = dp.z;
      cx_ = dp.x + offset_x_;
      cy_ = dp.y + offset_y_;
    }

    // skutocna poloha roveru
    const double a = speed_ / radius_ * (now().seconds() - t0_);
    const lrs_control::Vec3 rover{cx_ + radius_ * std::cos(a), cy_ + radius_ * std::sin(a), ground_z_};

    geometry_msgs::msg::PoseStamped m;
    m.header.stamp = now();
    m.header.frame_id = "map";
    m.pose.position.x = rover.x;
    m.pose.position.y = rover.y;
    m.pose.position.z = rover.z;
    m.pose.orientation.w = 1.0;
    truth_pub_->publish(m);

    // detekcia len v zabere kamery
    const lrs_control::Vec3 rel{rover.x - dp.x, rover.y - dp.y, rover.z - dp.z};
    cam_.altitude = dp.z - ground_z_;
    if (!lrs_control::inFootprint(rel, yawFromQuat(drone_.pose.orientation), cam_.altitude, cam_)) {
      return;
    }
    m.pose.position.x += gauss_(rng_) * noise_;
    m.pose.position.y += gauss_(rng_) * noise_;
    target_pub_->publish(m);
  }

  double offset_x_, offset_y_, radius_, speed_, noise_;
  lrs_control::SearchPatternParams cam_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_, truth_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PoseStamped drone_;
  bool has_drone_{false}, started_{false};
  double t0_{0.0}, ground_z_{0.0}, cx_{0.0}, cy_{0.0};
  std::mt19937 rng_{42};
  std::normal_distribution<double> gauss_{0.0, 1.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeTargetNode>());
  rclcpp::shutdown();
  return 0;
}
