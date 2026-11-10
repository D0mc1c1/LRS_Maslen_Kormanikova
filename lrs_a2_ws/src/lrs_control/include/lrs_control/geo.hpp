#pragma once
// A1.3: prevod zemepisnych suradnic (lat/lon/alt) <-> lokalne ENU metre od home
// a prevod kompasoveho kurzu <-> ENU yaw.
//
// Metoda: lokalna dotycnicova rovina ("flat Earth") s polomermi krivosti
// elipsoidu WGS-84 v bode home. Pre plochu misie (< 50 m od home) je chyba
// voci presnemu prevodu cez ECEF pod 1 mm (pozri test/geo_test.cpp).
#include <cmath>

#include "lrs_control/geometry.hpp"

namespace lrs_control
{

struct LatLonAlt
{
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
};

// Elipsoid WGS-84
constexpr double kWgs84A = 6378137.0;           // hlavna polos [m]
constexpr double kWgs84E2 = 6.69437999014e-3;   // excentricita^2

class LocalTangentPlane
{
public:
  LocalTangentPlane() = default;

  explicit LocalTangentPlane(const LatLonAlt & origin)
  : origin_(origin), valid_(true)
  {
    const double phi = deg2rad(origin.lat_deg);
    const double s = std::sin(phi);
    const double w = std::sqrt(1.0 - kWgs84E2 * s * s);
    r_north_ = kWgs84A * (1.0 - kWgs84E2) / (w * w * w);  // polomer v smere sever-juh (meridian)
    r_east_ = kWgs84A / w;                                // polomer v smere vychod-zapad
    cos_lat_ = std::cos(phi);
  }

  bool valid() const {return valid_;}
  const LatLonAlt & origin() const {return origin_;}

  // geodeticke -> ENU [m] od origin: x = vychod, y = sever, z = hore
  Vec3 toEnu(const LatLonAlt & p) const
  {
    const double h = origin_.alt_m;
    return {
      deg2rad(p.lon_deg - origin_.lon_deg) * (r_east_ + h) * cos_lat_,
      deg2rad(p.lat_deg - origin_.lat_deg) * (r_north_ + h),
      p.alt_m - origin_.alt_m};
  }

  // ENU [m] od origin -> geodeticke
  LatLonAlt toLla(const Vec3 & e) const
  {
    const double h = origin_.alt_m;
    return {
      origin_.lat_deg + rad2deg(e.y / (r_north_ + h)),
      origin_.lon_deg + rad2deg(e.x / ((r_east_ + h) * cos_lat_)),
      origin_.alt_m + e.z};
  }

private:
  LatLonAlt origin_{};
  bool valid_{false};
  double r_north_{0.0};
  double r_east_{0.0};
  double cos_lat_{1.0};
};

// Kompas: 0 deg = sever, kladny smer v smere hodinovych ruciciek.
// ENU yaw: 0 deg = vychod, kladny smer proti smeru hodinovych ruciciek.
//   yaw_ENU = 90 - heading   (zabalene do (-180, 180])
inline double compassToEnuYawDeg(double heading_deg)
{
  return rad2deg(wrapPi(deg2rad(90.0 - heading_deg)));
}

//   heading = 90 - yaw_ENU   (zabalene do [0, 360))
inline double enuYawToCompassDeg(double yaw_enu_deg)
{
  double h = std::fmod(90.0 - yaw_enu_deg, 360.0);
  if (h < 0.0) {
    h += 360.0;
  }
  return h;
}

}  // namespace lrs_control
