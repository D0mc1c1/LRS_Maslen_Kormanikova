#pragma once

// Prevod GPS suradnic (lat/lon/alt) na lokalne ENU suradnice a spat.
// Prevod kompasoveho kurzu na ENU yaw a spat.
// Pouziva lokalnu dotycnicovu rovinu a elipsoid WGS-84.
// Pre vzdialenosti do 50 m je chyba oproti ECEF mensia ako 1 mm.

#include <cmath>

#include "lrs_control/geometry.hpp"

namespace lrs_control
{

// GPS suradnice: zemepisna sirka, dlzka a nadmorska vyska.
struct LatLonAlt
{
  double lat_deg{0.0};
  double lon_deg{0.0};
  double alt_m{0.0};
};

// Parametre elipsoidu WGS-84.
constexpr double kWgs84A = 6378137.0;           // Hlavna polos [m]
constexpr double kWgs84E2 = 6.69437999014e-3;   // Druha mocnina excentricity

class LocalTangentPlane
{
public:
  LocalTangentPlane() = default;

  // Nastavi pociatocny bod a vypocita polomery krivosti.
  explicit LocalTangentPlane(const LatLonAlt & origin)
  : origin_(origin), valid_(true)
  {
    const double phi = deg2rad(origin.lat_deg);
    const double s = std::sin(phi);
    const double w = std::sqrt(1.0 - kWgs84E2 * s * s);
    r_north_ = kWgs84A * (1.0 - kWgs84E2) / (w * w * w);
    r_east_ = kWgs84A / w;
    cos_lat_ = std::cos(phi);
  }

  // Vrati, ci je pociatocny bod nastaveny.
  bool valid() const {return valid_;}

  // Vrati pociatocne GPS suradnice.
  const LatLonAlt & origin() const {return origin_;}

  // Prevedie GPS suradnice na ENU metre od pociatocneho bodu.
  // x = vychod, y = sever, z = hore.
  Vec3 toEnu(const LatLonAlt & p) const
  {
    const double h = origin_.alt_m;
    return {
      deg2rad(p.lon_deg - origin_.lon_deg) * (r_east_ + h) * cos_lat_,
      deg2rad(p.lat_deg - origin_.lat_deg) * (r_north_ + h),
      p.alt_m - origin_.alt_m};
  }

  // Prevedie ENU metre na GPS suradnice.
  LatLonAlt toLla(const Vec3 & e) const
  {
    const double h = origin_.alt_m;
    return {
      origin_.lat_deg + rad2deg(e.y / (r_north_ + h)),
      origin_.lon_deg + rad2deg(e.x / ((r_east_ + h) * cos_lat_)),
      origin_.alt_m + e.z};
  }

private:
  LatLonAlt origin_{};    // Pociatocny GPS bod
  bool valid_{false};     // Platnost pociatocneho bodu
  double r_north_{0.0};   // Polomer krivosti v smere sever-juh
  double r_east_{0.0};    // Polomer krivosti v smere vychod-zapad
  double cos_lat_{1.0};   // Kosinus zemepisnej sirky
};

// Prevod kompasoveho kurzu na ENU yaw.
// Kompas: 0 = sever, rastie v smere hodinovych ruciciek.
// ENU: 0 = vychod, rastie proti smeru hodinovych ruciciek.
// Vysledok je v rozsahu (-180, 180].
inline double compassToEnuYawDeg(double heading_deg)
{
  return rad2deg(wrapPi(deg2rad(90.0 - heading_deg)));
}

// Prevod ENU yaw na kompasovy kurz v rozsahu [0, 360).
inline double enuYawToCompassDeg(double yaw_enu_deg)
{
  double h = std::fmod(90.0 - yaw_enu_deg, 360.0);
  if (h < 0.0) {
    h += 360.0;
  }
  return h;
}

}
