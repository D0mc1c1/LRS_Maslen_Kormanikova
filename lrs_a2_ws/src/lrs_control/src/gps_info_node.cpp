// A1.3 cast 1: diagnostika GPS, home a kompasu.
//
//   ros2 run lrs_control gps_info
//
// Raz za sekundu vypise polohu ako lat/lon/alt aj ako ENU metre od home
// (oba smery prevodu), porovna ju s MAVROS local position a vypise kurz
// v oboch konvenciach (kompas aj ENU yaw). Nic neposiela, dron neovlada.
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "lrs_control/geo.hpp"
#include "lrs_control/geometry.hpp"
#include "lrs_control/mavros_vehicle.hpp"

using namespace std::chrono_literals;
using lrs_control::LatLonAlt;
using lrs_control::LocalTangentPlane;

class GpsInfoNode : public rclcpp::Node
{
public:
  GpsInfoNode()
  : Node("gps_info")
  {
    min_sats_ = declare_parameter<int>("min_satellites", 6);
    max_h_acc_ = declare_parameter<double>("max_h_acc", 2.0);
    // outdoor: MAVROS local ENU = nas pracovny ramec, bez rotacie a posunu
    vehicle_ = std::make_unique<lrs_control::MavrosVehicle>(*this, lrs_control::FrameTransform{});
    timer_ = create_wall_timer(1s, [this]() {print();});
  }

private:
  void print()
  {
    const auto st = vehicle_->state();
    if (!st.has_gps) {
      RCLCPP_INFO(get_logger(), "cakam na /mavros/global_position/global ...");
      return;
    }
    std::string why;
    const bool ready = lrs_control::gpsReady(st, min_sats_, max_h_acc_, why);
    char b[256];
    std::string out = "\n";

    std::snprintf(b, sizeof(b),
      "GPS   : status %d, satelity %s, presnost %s -> %s\n", st.gps_status,
      st.satellites >= 0 ? std::to_string(st.satellites).c_str() : "?",
      st.h_acc >= 0 ? (std::to_string(st.h_acc).substr(0, 4) + " m").c_str() : "?",
      ready ? "POUZITELNE NA VZLET" : ("NEPOUZITELNE: " + why).c_str());
    out += b;
    std::snprintf(b, sizeof(b), "poloha: lat %.7f  lon %.7f  alt %.2f m  | rel_alt %s\n",
      st.gps.lat_deg, st.gps.lon_deg, st.gps.alt_m,
      st.has_rel_alt ? (std::to_string(st.rel_alt).substr(0, 5) + " m").c_str() : "?");
    out += b;

    // pociatok ENU: home z MAVROS, inak prva platna GPS poloha (zadanie dovoluje oboje)
    if (st.has_home) {
      origin_ = st.home;
      origin_src_ = "home z /mavros/home_position/home";
      has_origin_ = true;
    } else if (!has_origin_ && ready) {
      origin_ = st.gps;
      origin_src_ = "prva platna GPS poloha (home zatial nie je)";
      has_origin_ = true;
    }

    if (has_origin_) {
      const LocalTangentPlane ltp(origin_);
      std::snprintf(b, sizeof(b), "pociatok: lat %.7f  lon %.7f  alt %.2f m  <- %s\n",
        origin_.lat_deg, origin_.lon_deg, origin_.alt_m, origin_src_.c_str());
      out += b;

      // geodeticke -> ENU (z GPS). Vyska: rel_alt (vyska nad home), nie rozdiel GPS altitude,
      // lebo MAVROS moze mat GPS altitude a home v inom vyskovom systeme (elipsoid vs. AMSL).
      const auto enu = ltp.toEnu(st.gps);
      const double up = st.has_rel_alt ? st.rel_alt : enu.z;
      std::snprintf(b, sizeof(b),
        "ENU z GPS        : E %7.2f  N %7.2f  U %7.2f m (U = rel_alt; rozdiel GPS altitude %.2f m)\n",
        enu.x, enu.y, up, enu.z);
      out += b;

      // MAVROS local position (EKF) na porovnanie
      if (st.has_pose) {
        const double dh = std::hypot(enu.x - st.pos.x, enu.y - st.pos.y);
        std::snprintf(b, sizeof(b),
          "MAVROS local     : E %7.2f  N %7.2f  U %7.2f m  | rozdiel horizontalne %.2f m\n",
          st.pos.x, st.pos.y, st.pos.z, dh);
        out += b;

        // ENU -> geodeticke (opacny smer): kde podla local position sme na Zemi
        const LatLonAlt back = ltp.toLla({st.pos.x, st.pos.y, st.pos.z});
        const auto again = ltp.toEnu(back);
        std::snprintf(b, sizeof(b),
          "local -> LLA     : lat %.7f  lon %.7f (spat do ENU: E %.2f N %.2f)\n",
          back.lat_deg, back.lon_deg, again.x, again.y);
        out += b;
      }
    } else {
      out += "pociatok: zatial nie je (cakam na home alebo platny GPS fix)\n";
    }

    if (st.has_compass) {
      const double yaw_from_compass = lrs_control::compassToEnuYawDeg(st.compass_hdg_deg);
      const double yaw_pose = lrs_control::rad2deg(st.yaw);
      const double diff = lrs_control::rad2deg(
        lrs_control::angleDiff(lrs_control::deg2rad(yaw_from_compass), st.yaw));
      std::snprintf(b, sizeof(b),
        "kurz  : kompas %6.1f deg (0 = sever, CW) -> ENU %6.1f deg | yaw z local_position %6.1f deg ENU"
        " | rozdiel %.1f deg\n", st.compass_hdg_deg, yaw_from_compass, yaw_pose, diff);
      out += b;
    }
    RCLCPP_INFO(get_logger(), "%s", out.c_str());
  }

  int min_sats_{6};
  double max_h_acc_{2.0};
  LatLonAlt origin_{};
  std::string origin_src_;
  bool has_origin_{false};
  std::unique_ptr<lrs_control::MavrosVehicle> vehicle_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GpsInfoNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
