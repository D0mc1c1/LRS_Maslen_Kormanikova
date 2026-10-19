#include "lrs_control/mavros_vehicle.hpp"

#include <cmath>

#include <rclcpp/version.h>

namespace lrs_control
{

using mavros_msgs::msg::PositionTarget;

namespace
{
// Vyberie sposob vytvorenia klienta podla verzie ROS2
// Humble a Jazzy pouzivaju odlisne rozhranie pre QoS sluzieb
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

// Vypocita yaw z orientacie vyjadrenej kvaternionom
double yawFromQuat(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}

// Pripravi odbery publikovanie a klientov sluzieb MAVROS
MavrosVehicle::MavrosVehicle(rclcpp::Node & node, const FrameTransform & tf)
: node_(node), tf_(tf)
{
  // Oddeli spracovanie senzorov od sluzieb
  sensor_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_group_ = node_.create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions opts;
  opts.callback_group = sensor_group_;
  // SensorDataQoS = best effort: prijme spravy od reliable aj best-effort publishera
  const auto qos = rclcpp::SensorDataQoS();

  // Sleduje stav spojenia rezim a armovanie dronu
  state_sub_ = node_.create_subscription<mavros_msgs::msg::State>(
    "/mavros/state", qos,
    [this](const mavros_msgs::msg::State & m) {
      const double t = node_.now().seconds();
      std::lock_guard<std::mutex> lk(mtx_);
      st_.connected = m.connected;
      st_.armed = m.armed;
      st_.mode = m.mode;
      state_rx_t_ = t;
    }, opts);

  // Nacita polohu a orientaciu a prevedie ich do map frame
  pose_sub_ = node_.create_subscription<geometry_msgs::msg::PoseStamped>(
    "/mavros/local_position/pose", qos,
    [this](const geometry_msgs::msg::PoseStamped & m) {
      const double t = node_.now().seconds();
      std::lock_guard<std::mutex> lk(mtx_);
      // local -> map
      st_.pos = tf_.toMap({m.pose.position.x, m.pose.position.y, m.pose.position.z});
      st_.yaw = tf_.yawToMap(yawFromQuat(m.pose.orientation));
      st_.has_pose = true;
      pose_rx_t_ = t;
    }, opts);

  // Nacita rychlost a prevedie ju do map frame
  vel_sub_ = node_.create_subscription<geometry_msgs::msg::TwistStamped>(
    "/mavros/local_position/velocity_local", qos,
    [this](const geometry_msgs::msg::TwistStamped & m) {
      std::lock_guard<std::mutex> lk(mtx_);
      st_.vel = tf_.dirToMap({m.twist.linear.x, m.twist.linear.y, m.twist.linear.z});
    }, opts);

  // --- A1.3: GPS, home, kompas -------------------------------------------
  // Nacita GPS polohu stav GPS a odhad horizontalnej presnosti
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

  // Nacita pocet viditelnych satelitov
  // Ak MAVROS temu nema hodnota zostane -1
  gps_raw_sub_ = node_.create_subscription<mavros_msgs::msg::GPSRAW>(
    "/mavros/gpsstatus/gps1/raw", qos,
    [this](const mavros_msgs::msg::GPSRAW & m) {
      std::lock_guard<std::mutex> lk(mtx_);
      st_.satellites = m.satellites_visible;
    }, opts);

  // Nacita vysku nad domovskym bodom
  rel_alt_sub_ = node_.create_subscription<std_msgs::msg::Float64>(
    "/mavros/global_position/rel_alt", qos,
    [this](const std_msgs::msg::Float64 & m) {
      std::lock_guard<std::mutex> lk(mtx_);
      st_.has_rel_alt = true;
      st_.rel_alt = m.data;
    }, opts);

  // Nacita smer kompasu v stupnoch
  compass_sub_ = node_.create_subscription<std_msgs::msg::Float64>(
    "/mavros/global_position/compass_hdg", qos,
    [this](const std_msgs::msg::Float64 & m) {
      std::lock_guard<std::mutex> lk(mtx_);
      st_.has_compass = true;
      st_.compass_hdg_deg = m.data;
    }, opts);

  // home MAVROS posiela ako "latched" spravu (reliable + transient_local) len pri zmene.
  // Best-effort/volatile odber by ju po starte nodu nedostal -> rovnake QoS ako publisher.
  // Nacita domovsku GPS polohu dronu
  home_sub_ = node_.create_subscription<mavros_msgs::msg::HomePosition>(
    "/mavros/home_position/home", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
    [this](const mavros_msgs::msg::HomePosition & m) {
      std::lock_guard<std::mutex> lk(mtx_);
      st_.has_home = true;
      st_.home = {m.geo.latitude, m.geo.longitude, m.geo.altitude};
    }, opts);

  // Vytvori publisher cielovych poloh pre riadenie dronu
  sp_pub_ = node_.create_publisher<PositionTarget>("/mavros/setpoint_raw/local", 10);

  // Pripravi sluzby na zmenu rezimu armovanie vzlet a pristatie
  mode_cli_ = makeClient<mavros_msgs::srv::SetMode>(node_, "/mavros/set_mode", service_group_);
  arm_cli_ = makeClient<mavros_msgs::srv::CommandBool>(node_, "/mavros/cmd/arming", service_group_);
  takeoff_cli_ = makeClient<mavros_msgs::srv::CommandTOL>(node_, "/mavros/cmd/takeoff", service_group_);
  land_cli_ = makeClient<mavros_msgs::srv::CommandTOL>(node_, "/mavros/cmd/land", service_group_);
}

// Vrati aktualny stav dronu aj s vekom poslednych sprav
VehicleState MavrosVehicle::state() const
{
  const double now = node_.now().seconds();
  std::lock_guard<std::mutex> lk(mtx_);
  VehicleState s = st_;
  s.state_age = state_rx_t_ < 0.0 ? -1.0 : now - state_rx_t_;
  s.pose_age = pose_rx_t_ < 0.0 ? -1.0 : now - pose_rx_t_;
  return s;
}

// Vrati stav poslednej poziadavky na sluzbu
RequestStatus MavrosVehicle::requestStatus() const
{
  std::lock_guard<std::mutex> lk(mtx_);
  return req_status_;
}

// Spolocne odosle poziadavku sluzbe a spracuje jej odpoved
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
  // Overi ci je sluzba dostupna
  if (!client->service_is_ready()) {
    RCLCPP_WARN(node_.get_logger(), "%s: sluzba %s nie je dostupna (bezi MAVROS?)",
      what.c_str(), client->get_service_name());
    std::lock_guard<std::mutex> lk(mtx_);
    req_status_ = RequestStatus::Failed;
    return;
  }
  // Odosle poziadavku a caka na asynchronnu odpoved
  RCLCPP_INFO(node_.get_logger(), "-> %s", what.c_str());
  client->async_send_request(
    req, [this, id, ok, what](typename rclcpp::Client<SrvT>::SharedFuture f) {
      const bool success = ok(*f.get());
      if (!success) {
        RCLCPP_WARN(node_.get_logger(), "<- %s: ZAMIETNUTE", what.c_str());
      }
      std::lock_guard<std::mutex> lk(mtx_);
      // Aktualizuje stav iba pre najnovsiu poziadavku
      if (id == req_id_) {
        req_status_ = success ? RequestStatus::Ok : RequestStatus::Failed;
      }
    });
}

// Poziada autopilota o zmenu letoveho rezimu
void MavrosVehicle::requestMode(const std::string & mode)
{
  auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
  req->base_mode = 0;
  req->custom_mode = mode;
  call<mavros_msgs::srv::SetMode>(mode_cli_, req,
    [](const mavros_msgs::srv::SetMode::Response & r) {return r.mode_sent;}, "set_mode " + mode);
}

// Poziada o zapnutie alebo vypnutie motorov
void MavrosVehicle::requestArm(bool arm)
{
  auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
  req->value = arm;
  call<mavros_msgs::srv::CommandBool>(arm_cli_, req,
    [](const mavros_msgs::srv::CommandBool::Response & r) {return r.success;},
    arm ? "arming true" : "arming false");
}

// Poziada autopilota o vzlet do zadanej vysky
void MavrosVehicle::requestTakeoff(double z_map)
{
  auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  req->min_pitch = 0.0f;
  req->yaw = 0.0f;  // ArduPilot ho pri GUIDED takeoffe ignoruje
  // Prevedie cielovu vysku z map frame do lokalneho systemu
  req->altitude = static_cast<float>(tf_.toLocal({0.0, 0.0, z_map}).z);  // vyska nad home
  call<mavros_msgs::srv::CommandTOL>(takeoff_cli_, req,
    [](const mavros_msgs::srv::CommandTOL::Response & r) {return r.success;},
    "takeoff alt=" + std::to_string(req->altitude));
}

// Poziada autopilota o pristatie
void MavrosVehicle::requestLand()
{
  auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
  req->altitude = 0.0f;
  call<mavros_msgs::srv::CommandTOL>(land_cli_, req,
    [](const mavros_msgs::srv::CommandTOL::Response & r) {return r.success;}, "land");
}

// Vytvori poziadavku na polohu a yaw
void MavrosVehicle::sendPositionTarget(const Vec3 & p_map, double yaw)
{
  TrajectorySetpoint sp;
  sp.p = p_map;
  sp.yaw = yaw;
  sendTrajectorySetpoint(sp);
}

// Prevedie cielovy bod na spravu MAVROS a odosle ho dronu
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
  // Ak je zadana rychlost prevedie ju do lokalneho systemu
  if (t.v) {
    const Vec3 vl = tf_.dirToLocal(*t.v);
    sp.velocity.x = vl.x;
    sp.velocity.y = vl.y;
    sp.velocity.z = vl.z;
  } else {
    // Oznaci rychlost ako nepouzivanu
    sp.type_mask |= PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY |
      PositionTarget::IGNORE_VZ;
  }
  // Prevedie cielovy yaw do lokalneho systemu
  sp.yaw = static_cast<float>(tf_.yawToLocal(t.yaw));
  // Ak je zadana rychlost otacania nastavi ju
  if (t.yaw_rate) {
    sp.yaw_rate = static_cast<float>(*t.yaw_rate);  // ENU, MAVROS otoci znamienko do NED
  } else {
    // Oznaci rychlost otacania ako nepouzivanu
    sp.type_mask |= PositionTarget::IGNORE_YAW_RATE;
  }
  // Odosle cielovu poziadavku do MAVROS
  sp_pub_->publish(sp);
}
}