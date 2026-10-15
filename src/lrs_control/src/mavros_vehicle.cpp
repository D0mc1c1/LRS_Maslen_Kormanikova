#include "lrs_control/mavros_vehicle.hpp"

#include <cmath>

#include <rclcpp/version.h>

namespace lrs_control
{

using mavros_msgs::msg::PositionTarget;

namespace
{
// Humble (rclcpp 16) ma create_client s rmw_qos_profile_t, Jazzy (28+) s rclcpp::QoS
template<class SrvT>
typename rclcpp::Client<SrvT>::SharedPtr makeClient(
  rclcpp::Node & node, const std::string & name, rclcpp::CallbackGroup::SharedPtr group)
{
#if RCLCPP_VERSION_MAJOR >= 28
  return node.create_client<SrvT>(name, rclcpp::ServicesQoS(), group);
#else
  return node.create_client<SrvT>(name, rmw_qos_profile_services_default, group);
#endif
}

double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

MavrosVehicle::MavrosVehicle(rclcpp::Node & node, const FrameTransform & tf)
: node_(node), tf_(tf)
{
  sensor_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions opts;
  opts.callback_group = sensor_group_;
  // SensorDataQoS = best effort: prijme spravy od reliable aj best-effort publishera
  const auto qos = rclcpp::SensorDataQoS();

  state_sub_ = node_.create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", qos,
    [this](const mavros_msgs::msg::State & m) {
      std::lock_guard<std::mutex> lk(mtx_);
      st_.connected = m.connected;
      st_.armed = m.armed;
      st_.mode = m.mode;
    }, opts);

  pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", qos,
    [this](const geometry_msgs::msg::PoseStamped & m) {
      std::lock_guard<std::mutex> lk(mtx_);
      // local -> map
      st_.pos = tf_.toMap({m.pose.position.x, m.pose.position.y, m.pose.position.z});
      st_.yaw = tf_.yawToMap(yawFromQuat(m.pose.orientation));
      st_.has_pose = true;
    }, opts);

  vel_sub_ = node_.create_subscription<geometry_msgs::msg::TwistStamped>(
    "/mavros/local_position/velocity_local", qos,
    [this](const geometry_msgs::msg::TwistStamped & m) {
      std::lock_guard<std::mutex> lk(mtx_);
      st_.vel = tf_.dirToMap({m.twist.linear.x, m.twist.linear.y, m.twist.linear.z});
    }, opts);

  // --- A1.3: GPS, home, kompas -------------------------------------------
  gps_sub_ = node_.create_subscription<sensor_msgs::msg::NavSatFix>(
    "/mavros/global_position/global", qos,
    [this](const sensor_msgs::msg::NavSatFix & m) {
      std::lock_guard<std::mutex> lk(mtx_);
      st_.has_gps = true;
      st_.gps_status = m.status.status;
      st_.gps = {m.latitude, m.longitude, m.altitude};
      // kovariancia [m^2]: diagonala E, N; 0 = COVARIANCE_TYPE_UNKNOWN
      if (m.position_covariance_type != sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) {
        st_.h_acc = std::sqrt((m.position_covariance[0] + m.position_covariance[4]) / 2.0);
      }
    }, opts);

  // pocet satelitov (plugin gps_status; ak MAVROS temu nema, ostane -1 = nezname)
  gps_raw_sub_ = node_.create_subscription<mavros_msgs::msg::GPSRAW>(
    "/mavros/gpsstatus/gps1/raw", qos,
    [this](const mavros_msgs::msg::GPSRAW & m) {
      std::lock_guard<std::mutex> lk(mtx_);
      st_.satellites = m.satellites_visible;
    }, opts);

  rel_alt_sub_ = node_.create_subscription<std_msgs::msg::Float64>(
    "/mavros/global_position/rel_alt", qos,
    [this](const std_msgs::msg::Float64 & m) {
      std::lock_guard<std::mutex> lk(mtx_);
      st_.has_rel_alt = true;
      st_.rel_alt = m.data;
    }, opts);

  compass_sub_ = node_.create_subscription<std_msgs::msg::Float64>(
    "/mavros/global_position/compass_hdg", qos,
    [this](const std_msgs::msg::Float64 & m) {
      std::lock_guard<std::mutex> lk(mtx_);
      st_.has_compass = true;
      st_.compass_hdg_deg = m.data;
    }, opts);

  home_sub_ = node_.create_subscription<mavros_msgs::msg::HomePosition>(
    "/mavros/home_position/home", qos,
    [this](const mavros_msgs::msg::HomePosition & m) {
      std::lock_guard<std::mutex> lk(mtx_);
      st_.has_home = true;
      st_.home = {m.geo.latitude, m.geo.longitude, m.geo.altitude};
    }, opts);

  sp_pub_ = node_.create_publisher<PositionTarget>("/mavros/setpoint_raw/local", 10);

  mode_cli_ = makeClient<mavros_msgs::srv::SetMode>(node_, "/mavros/set_mode", service_group_);
  arm_cli_ = makeClient<mavros_msgs::srv::CommandBool>(node_, "/mavros/cmd/arming", service_group_);
  takeoff_cli_ = makeClient<mavros_msgs::srv::CommandTOL>(node_, "/mavros/cmd/takeoff", service_group_);
  land_cli_ = makeClient<mavros_msgs::srv::CommandTOL>(node_, "/mavros/cmd/land", service_group_);
}

VehicleState MavrosVehicle::state() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return st_;
}

RequestStatus MavrosVehicle::requestStatus() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return req_status_;
}

template<class SrvT, class OkFn>
void MavrosVehicle::call(
  const typename rclcpp::Client<SrvT>::SharedPtr & client,
  std::shared_ptr<typename SrvT::Request> req, OkFn ok, const std::string & what)
{
  std::uint64_t id;
  {
    std::lock_guard<std::mutex> lk(mtx_);
    id = ++req_id_;
    req_status_ = RequestStatus::Pending;
  }
  if (!client->service_is_ready()) {
    RCLCPP_WARN(node_.get_logger(), "%s: sluzba %s nie je dostupna (bezi MAVROS?)",
      what.c_str(), client->get_service_name());
    std::lock_guard<std::mutex> lk(mtx_);
    req_status_ = RequestStatus::Failed;
    return;
  }
  RCLCPP_INFO(node_.get_logger(), "-> %s", what.c_str());
  client->async_send_request(
    req, [this, id, ok, what](typename rclcpp::Client<SrvT>::SharedFuture f) {
      const bool success = ok(*f.get());
      if (!success) {
        RCLCPP_WARN(node_.get_logger(), "<- %s: ZAMIETNUTE", what.c_str());
      }
      std::lock_guard<std::mutex> lk(mtx_);
      if (id == req_id_) {
        req_status_ = success ? RequestStatus::Ok : RequestStatus::Failed;
      }
    });
}

void MavrosVehicle::requestMode(const std::string & mode)
{
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->base_mode = 0;
  req->custom_mode = mode;
  call<mavros_msgs::srv::SetMode>(mode_cli_, req,
    [](const mavros_msgs::srv::SetMode::Response & r) {return r.mode_sent;}, "set_mode " + mode);
}

void MavrosVehicle::requestArm(bool arm)
{
  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = arm;
  call<mavros_msgs::srv::CommandBool>(arm_cli_, req,
    [](const mavros_msgs::srv::CommandBool::Response & r) {return r.success;},
    arm ? "arming true" : "arming false");
}

void MavrosVehicle::requestTakeoff(double z_map)
{
  auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  req->min_pitch = 0.0f;
  req->yaw = 0.0f;  // ArduPilot ho pri GUIDED takeoffe ignoruje
  req->altitude = static_cast<float>(tf_.toLocal({0.0, 0.0, z_map}).z);  // vyska nad home
  call<mavros_msgs::srv::CommandTOL>(takeoff_cli_, req,
    [](const mavros_msgs::srv::CommandTOL::Response & r) {return r.success;},
    "takeoff alt=" + std::to_string(req->altitude));
}

void MavrosVehicle::requestLand()
{
  auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  req->altitude = 0.0f;
  call<mavros_msgs::srv::CommandTOL>(land_cli_, req,
    [](const mavros_msgs::srv::CommandTOL::Response & r) {return r.success;}, "land");
}

void MavrosVehicle::sendPositionTarget(const Vec3 & p_map, double yaw)
{
  TrajectorySetpoint sp;
  sp.p = p_map;
  sp.yaw = yaw;
  sendTrajectorySetpoint(sp);
}

void MavrosVehicle::sendTrajectorySetpoint(const TrajectorySetpoint & t)
{
  PositionTarget sp;
  sp.header.stamp = node_.now();
  sp.header.frame_id = "map";
  // MAVROS cakava ENU data s FRAME_LOCAL_NED a prevod do NED spravi sam
  sp.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
  // vzdy poloha + yaw; rychlost a yaw_rate len ak su zadane; zrychlenie nikdy
  sp.type_mask = PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY |
    PositionTarget::IGNORE_AFZ;
  // map -> local
  const Vec3 l = tf_.toLocal(t.p);
  sp.position.x = l.x;
  sp.position.y = l.y;
  sp.position.z = l.z;
  if (t.v) {
    const Vec3 vl = tf_.dirToLocal(*t.v);
    sp.velocity.x = vl.x;
    sp.velocity.y = vl.y;
    sp.velocity.z = vl.z;
  } else {
    sp.type_mask |= PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY |
      PositionTarget::IGNORE_VZ;
  }
  sp.yaw = static_cast<float>(tf_.yawToLocal(t.yaw));
  if (t.yaw_rate) {
    sp.yaw_rate = static_cast<float>(*t.yaw_rate);  // ENU, MAVROS otoci znamienko do NED
  } else {
    sp.type_mask |= PositionTarget::IGNORE_YAW_RATE;
  }
  sp_pub_->publish(sp);
}

}  // namespace lrs_control
