#pragma once
// Uhly a vzdialenosti. Vsetko v ENU, radiany:
//   yaw = 0 -> +X (vychod), kladny smer = proti smeru hodinovych ruciciek.
#include <cmath>

#include "lrs_planning/types.hpp"

namespace lrs_control
{

using lrs_planning::Vec3;

constexpr double kPi = 3.14159265358979323846;

inline double deg2rad(double d) {return d * kPi / 180.0;}
inline double rad2deg(double r) {return r * 180.0 / kPi;}

// Zabali uhol do (-pi, pi].
inline double wrapPi(double a)
{
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a <= 0.0) {
    a += 2.0 * kPi;
  }
  return a - kPi;
}

// Najkratsi rozdiel (target - current) v (-pi, pi].
inline double angleDiff(double target, double current) {return wrapPi(target - current);}

// Vrati target + k*2pi najblizsie k reference (unwrap). Zabrani skoku o 2pi,
// ktory by dron otocil o cele kolo opacnym smerom.
inline double unwrapNear(double target, double reference)
{
  return reference + angleDiff(target, reference);
}

inline double dist(const Vec3 & a, const Vec3 & b)
{
  return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

inline double distXY(const Vec3 & a, const Vec3 & b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

inline double norm(const Vec3 & v) {return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);}

// Smer letu z p do q (ENU yaw). Pri cisto vertikalnom useku nema zmysel.
inline double segmentYaw(const Vec3 & p, const Vec3 & q)
{
  return std::atan2(q.y - p.y, q.x - p.x);
}

// Prevod medzi map frame (PCD mapa, CSV misie = svet Gazeba) a MAVROS local ENU:
//
//     local = R(yaw) * map + t
//
// V fei_lrs_gazebo.world je gazeboXYZToNED = (180, 0, 0) -> Gazebo X = SEVER,
// Gazebo Y = ZAPAD. MAVROS local ENU je preto voci mape otoceny o +90 deg:
//     local = (-y_map, x_map, z_map),  t = 0  (EKF origin = pociatok sveta)
// Overenie: spawn (13, 7, 0) v mape -> /mavros/local_position/pose (-7, 13, 0).
struct FrameTransform
{
  double yaw{0.0};   // [rad] otocenie local voci map
  Vec3 t{};          // [m]   posun

  Vec3 toLocal(const Vec3 & m) const
  {
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {c * m.x - s * m.y + t.x, s * m.x + c * m.y + t.y, m.z + t.z};
  }
  Vec3 toMap(const Vec3 & l) const
  {
    const double c = std::cos(yaw), s = std::sin(yaw);
    const double dx = l.x - t.x, dy = l.y - t.y;
    return {c * dx + s * dy, -s * dx + c * dy, l.z - t.z};
  }
  Vec3 dirToMap(const Vec3 & v) const   // vektor (rychlost), bez posunu
  {
    const double c = std::cos(yaw), s = std::sin(yaw);
    return {c * v.x + s * v.y, -s * v.x + c * v.y, v.z};
  }
  double yawToLocal(double y_map) const {return y_map + yaw;}
  double yawToMap(double y_local) const {return wrapPi(y_local - yaw);}
};

}  // namespace lrs_control
