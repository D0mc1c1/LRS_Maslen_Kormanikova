#pragma once
#include <string>

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
};

enum class RequestStatus { None, Pending, Ok, Failed };

// Abstrakcia nad dronom. Stavovy automat vidi len toto rozhranie:
//  - MavrosVehicle  = skutocny dron cez MAVROS (ROS 2)
//  - SimVehicle     = jednoduchy model v mission_sim (offline test bez ROS)
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
};

}  // namespace lrs_control
