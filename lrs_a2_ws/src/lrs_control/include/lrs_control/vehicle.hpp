#pragma once
#include <optional>
#include <string>

#include "lrs_control/geo.hpp"
#include "lrs_planning/types.hpp"

namespace lrs_control
{

using lrs_planning::Vec3;

// Snimka stavu drona. Poloha je uz prevedena do MAP frame.
struct VehicleState
{
  bool connected{false};
  bool armed{false};
  std::string mode;
  bool has_pose{false};
  Vec3 pos{};        // map frame [m]
  Vec3 vel{};        // [m/s]
  double yaw{0.0};   // ENU [rad]

  // --- GPS, home, kompas (A1.3; indoor sa nepouziva) ---------------------
  bool has_gps{false};          // prisla aspon jedna sprava NavSatFix
  int gps_status{-1};           // NavSatFix.status: -1 bez fixu, 0 fix, 1 SBAS, 2 GBAS
  int satellites{-1};           // pocet satelitov, -1 = nezname (tema gpsstatus nie je)
  double h_acc{-1.0};           // odhad horizontalnej presnosti [m] z kovariancie, -1 = nezname
  LatLonAlt gps{};              // lat/lon/alt z /mavros/global_position/global
  bool has_rel_alt{false};
  double rel_alt{0.0};          // vyska nad home [m] z /mavros/global_position/rel_alt
  bool has_compass{false};
  double compass_hdg_deg{0.0};  // kompas: 0 = sever, v smere hodin [deg]
  bool has_home{false};
  LatLonAlt home{};             // z /mavros/home_position/home

  // --- zivost spojenia (A1.3 cast 5) -------------------------------------
  // Vek poslednej spravy [s], -1 = este neprisla. Ak MAVROS spadne, `connected`
  // ostane na poslednej hodnote, ale tieto casy rastu.
  double state_age{-1.0};       // /mavros/state (MAVROS ho posiela ~1 Hz)
  double pose_age{-1.0};        // /mavros/local_position/pose
};

// Je spojenie s dronom zive? FCU pripojene k MAVROS a spravy z MAVROS chodia.
inline bool linkOk(const VehicleState & st, double state_timeout, double pose_timeout,
  std::string & why)
{
  if (st.state_age < 0.0 || st.state_age > state_timeout) {
    why = "/mavros/state neprislo " + std::to_string(st.state_age).substr(0, 4) + " s (MAVROS bezi?)";
    return false;
  }
  if (!st.connected) {why = "MAVROS hlasi connected=false (FCU odpojene)"; return false;}
  if (st.pose_age < 0.0 || st.pose_age > pose_timeout) {
    why = "/mavros/local_position/pose neprislo " + std::to_string(st.pose_age).substr(0, 4) + " s";
    return false;
  }
  why = "OK";
  return true;
}

// Je GPS pouzitelne na vzlet? Ak nie, `why` povie preco.
//  - fix aspon STATUS_FIX (0)
//  - satelity >= min_sats (ak ich MAVROS publikuje)
//  - horizontalna presnost <= max_h_acc (ak je kovariancia znama)
inline bool gpsReady(const VehicleState & st, int min_sats, double max_h_acc, std::string & why)
{
  if (!st.has_gps) {why = "ziadna sprava z /mavros/global_position/global"; return false;}
  if (st.gps_status < 0) {why = "GPS bez fixu (status " + std::to_string(st.gps_status) + ")"; return false;}
  if (st.satellites >= 0 && st.satellites < min_sats) {
    why = "malo satelitov: " + std::to_string(st.satellites) + " < " + std::to_string(min_sats);
    return false;
  }
  if (st.h_acc >= 0.0 && st.h_acc > max_h_acc) {
    why = "nizka presnost: " + std::to_string(st.h_acc) + " m > " + std::to_string(max_h_acc) + " m";
    return false;
  }
  why = "OK";
  return true;
}

enum class RequestStatus { None, Pending, Ok, Failed };

// A1.3: setpoint casovej trajektorie. Rychlost a yaw_rate su FEEDFORWARD:
// ArduPilot ide s pohybujucim sa setpointom, nie za nim s oneskorenim.
// (yaw + yaw_rate naraz = uhol s feedforward, ArduCopter 4.1+)
struct TrajectorySetpoint
{
  Vec3 p{};                         // poloha (map frame = outdoor local ENU)
  double yaw{0.0};                  // [rad] ENU
  std::optional<Vec3> v;            // [m/s]
  std::optional<double> yaw_rate;   // [rad/s] ENU, kladny = proti smeru hodinovych ruciciek
};

// Abstrakcia nad dronom. Stavovy automat vidi len toto rozhranie:
//  - MavrosVehicle  = skutocny dron cez MAVROS (ROS 2)
////  - SimVehicle     = jednoduchy model v mission_sim (offline test bez ROS)
// Prevod map <-> local (MAVROS) robi implementacia, nie automat.
//
// Poziadavky (request*) su asynchronne a neblokuju. Naraz bezi jedna,
// jej vysledok sa cita cez requestStatus(). Novy request prepise stary.
class Vehicle
{
public:
  virtual ~Vehicle() = default;

  virtual VehicleState state() const = 0;

  virtual void requestMode(const std::string & mode) = 0;
  virtual void requestArm(bool arm) = 0;
  virtual void requestTakeoff(double z_map) = 0;  // vyska ciela v map frame
  virtual void requestLand() = 0;
  virtual RequestStatus requestStatus() const = 0;

  // Pozicny setpoint (map frame) + yaw (ENU, rad). Vola sa kazdy tick.
  virtual void sendPositionTarget(const Vec3 & p_map, double yaw) = 0;

  // A1.3: setpoint casovej trajektorie (poloha + yaw, volitelne feedforward).
  // Bez implementacie sa posle len poloha + yaw.
  virtual void sendTrajectorySetpoint(const TrajectorySetpoint & sp)
  {
    sendPositionTarget(sp.p, sp.yaw);
  }
};

}
