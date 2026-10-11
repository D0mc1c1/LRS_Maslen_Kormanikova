#pragma once
#include <cstdint>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <rclcpp/rclcpp.hpp>

#include "lrs_control/geometry.hpp"
#include "lrs_control/vehicle.hpp"

namespace lrs_control
{

// Vehicle nad MAVROS. Jedine miesto, ktore pozna temy/sluzby MAVROS
// a prevod map frame <-> MAVROS local frame (FrameTransform v geometry.hpp).
class MavrosVehicle : public Vehicle
{
public:
  MavrosVehicle(rclcpp::Node & node, const FrameTransform & tf);

  VehicleState state() const override;
  void requestMode(const std::string & mode) override;
  void requestArm(bool arm) override;
  void requestTakeoff(double z_map) override;
  void requestLand() override;
  RequestStatus requestStatus() const override;
  void sendPositionTarget(const Vec3 & p_map, double yaw) override;

private:
  // Asynchronne volanie sluzby. Odpoved spracuje callback v service_group_,
  // takze riadiaci timer NIKDY neblokuje cakanim na sluzbu.
  template<class SrvT, class OkFn>
  void call(
    const typename rclcpp::Client<SrvT>::SharedPtr & client,
    std::shared_ptr<typename SrvT::Request> req, OkFn ok, const std::string & what);

  rclcpp::Node & node_;
  FrameTransform tf_;

  rclcpp::CallbackGroup::SharedPtr sensor_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;

  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr sp_pub_;

  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_cli_;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr land_cli_;

  mutable std::mutex mtx_;
  VehicleState st_;              // chranene mtx_
  RequestStatus req_status_{RequestStatus::None};
  std::uint64_t req_id_{0};      // odpoved na staru poziadavku sa ignoruje
};

}  // namespace lrs_control
