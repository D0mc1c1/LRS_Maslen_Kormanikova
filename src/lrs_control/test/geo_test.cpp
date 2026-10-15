// geo_test: overenie geo.hpp bez ROS.
//  1) chyba flat-Earth prevodu voci presnemu prevodu cez ECEF pre rozne vzdialenosti
//  2) spatny prevod ENU -> LLA -> ENU
//  3) kompas <-> ENU yaw
// Vysledok (chyba na 50 m) patri do dokumentacie A1.3.
#include <cmath>
#include <cstdio>

#include "lrs_control/geo.hpp"

using namespace lrs_control;

// presny prevod: LLA -> ECEF -> ENU (referencia)
static Vec3 ecef(const LatLonAlt & p)
{
  const double phi = deg2rad(p.lat_deg), lam = deg2rad(p.lon_deg);
  const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * std::sin(phi) * std::sin(phi));
  return {(n + p.alt_m) * std::cos(phi) * std::cos(lam),
    (n + p.alt_m) * std::cos(phi) * std::sin(lam),
    (n * (1.0 - kWgs84E2) + p.alt_m) * std::sin(phi)};
}

static Vec3 exactEnu(const LatLonAlt & o, const LatLonAlt & p)
{
  const Vec3 a = ecef(o), b = ecef(p);
  const double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
  const double phi = deg2rad(o.lat_deg), lam = deg2rad(o.lon_deg);
  return {-std::sin(lam) * dx + std::cos(lam) * dy,
    -std::sin(phi) * std::cos(lam) * dx - std::sin(phi) * std::sin(lam) * dy + std::cos(phi) * dz,
    std::cos(phi) * std::cos(lam) * dx + std::cos(phi) * std::sin(lam) * dy + std::sin(phi) * dz};
}

int main()
{
  const LatLonAlt home{48.15144444, 17.07375000, 150.0};  // fei_lrs_outdoor.world
  const LocalTangentPlane ltp(home);
  int fails = 0;

  std::printf("1) chyba flat-Earth voci ECEF (bod na severovychod, 5 m nad home)\n");
  for (double d : {5.0, 10.0, 50.0, 100.0, 500.0, 1000.0}) {
    const Vec3 want{d / std::sqrt(2.0), d / std::sqrt(2.0), 5.0};
    const LatLonAlt p = ltp.toLla(want);
    const Vec3 got = ltp.toEnu(p);
    const Vec3 ref = exactEnu(home, p);
    const double err_h = std::hypot(got.x - ref.x, got.y - ref.y);
    const double err_v = std::abs(got.z - ref.z);
    std::printf("   %6.0f m: horizontalne %.6f m, vertikalne %.6f m\n", d, err_h, err_v);
    if (d <= 50.0 && (err_h > 0.001 || err_v > 0.001)) ++fails;
  }

  std::printf("2) spatny prevod ENU -> LLA -> ENU\n");
  const Vec3 e{3.0, -2.5, 5.0};
  const Vec3 back = ltp.toEnu(ltp.toLla(e));
  const double rt = std::sqrt(std::pow(back.x - e.x, 2) + std::pow(back.y - e.y, 2) + std::pow(back.z - e.z, 2));
  std::printf("   (3.0, -2.5, 5.0) -> (%.9f, %.9f, %.9f), chyba %.2e m\n", back.x, back.y, back.z, rt);
  if (rt > 1e-6) ++fails;

  std::printf("3) kompas -> ENU yaw\n");
  const double cases[][2] = {{0, 90}, {90, 0}, {180, -90}, {270, 180}, {45, 45}, {315, 135}};
  for (const auto & c : cases) {
    const double y = compassToEnuYawDeg(c[0]);
    const double h = enuYawToCompassDeg(y);
    std::printf("   kompas %5.1f deg -> ENU %6.1f deg -> kompas %5.1f deg\n", c[0], y, h);
    if (std::abs(y - c[1]) > 1e-9 || std::abs(h - c[0]) > 1e-9) ++fails;
  }

  std::printf(fails ? "\nCHYBA: %d testov zlyhalo\n" : "\nvsetko OK\n", fails);
  return fails ? 1 : 0;
}
