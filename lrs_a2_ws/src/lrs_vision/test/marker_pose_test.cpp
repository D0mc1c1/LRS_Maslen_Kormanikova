// Offline test marker_pose.hpp (§3), bez ROS a bez simulatora.
//
// Rohy znacky sa vyrobia "dopredu" zo znamej polohy drona a roveru (vlastny jednoduchy
// model kamery, nezavisly od testovanych funkcii) a reťazec solvePnP -> telo -> ENU
// ich musi vratit spat na spravne miesto.
//
//   ros2 run lrs_vision marker_pose_test        # navratovy kod 0 = vsetko PASS
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "lrs_vision/marker_pose.hpp"

using lrs_vision::CameraExtrinsics;
using lrs_vision::CameraIntrinsics;

namespace
{

int g_fails = 0;

void check(bool ok, const char * name, double value, double limit)
{
  std::printf("  %s %-58s %8.4f (limit %.3f)\n", ok ? "PASS" : "FAIL", name, value, limit);
  if (!ok) {
    ++g_fails;
  }
}

double dist(const cv::Vec3d & a, const cv::Vec3d & b) {return cv::norm(a - b);}

// kamera zo sveta: 640x480, HFOV 60° -> fx = 320 / tan(30°) = 554.26 (camera_info dava 554.38)
CameraIntrinsics simCamera()
{
  const double fx = 320.0 / std::tan(M_PI / 6.0);
  const std::array<double, 9> k = {fx, 0, 320.0, 0, fx, 240.0, 0, 0, 1};
  return lrs_vision::intrinsicsFromCameraInfo(k, {});
}

// Rohy znacky v pixeloch pre dron v drone_pos s yaw, znacku so stredom c_enu natocenu o
// marker_yaw (lezi vodorovne, kresba smeruje k oblohe). Vlastny model: ENU -> telo -> optical
// pisany rucne, nie cez testovane funkcie.
std::vector<cv::Point2f> projectMarker(
  const CameraIntrinsics & cam, const cv::Vec3d & drone_pos, double yaw,
  const cv::Vec3d & c_enu, double marker_yaw, double size, double noise_px, std::mt19937 & rng)
{
  const double h = size / 2.0;
  const double offs[4][2] = {{-h, h}, {h, h}, {h, -h}, {-h, -h}};  // poradie ArUco
  std::normal_distribution<double> n(0.0, noise_px);
  std::vector<cv::Point2f> px;
  for (const auto & o : offs) {
    // roh v ENU
    const double ex = c_enu[0] + o[0] * std::cos(marker_yaw) - o[1] * std::sin(marker_yaw);
    const double ey = c_enu[1] + o[0] * std::sin(marker_yaw) + o[1] * std::cos(marker_yaw);
    const double ez = c_enu[2];
    // ENU -> telo (len yaw): otocenie o -yaw
    const double dx = ex - drone_pos[0], dy = ey - drone_pos[1], dz = ez - drone_pos[2];
    const double bx = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double by = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    const double bz = dz;
    // telo -> optical pre kameru 3 cm pod telom, pohlad dole, obraz hore = dopredu
    const double cx = bx, cy = by, cz = bz + 0.03;
    const double xo = -cy, yo = -cx, zo = -cz;
    const double u = cam.K(0, 0) * xo / zo + cam.K(0, 2) + n(rng);
    const double v = cam.K(1, 1) * yo / zo + cam.K(1, 2) + n(rng);
    px.emplace_back(static_cast<float>(u), static_cast<float>(v));
  }
  return px;
}

cv::Matx33d yawRotation(double yaw)
{
  // kvaternion len s yaw, ako ho posle MAVROS pri vznasani
  return lrs_vision::rotationFromQuaternion(0, 0, std::sin(yaw / 2), std::cos(yaw / 2));
}

}  // namespace

int main()
{
  std::mt19937 rng(42);
  const CameraIntrinsics cam = simCamera();
  const CameraExtrinsics ex = lrs_vision::extrinsicsFromPose({0.0, 0.0, -0.03}, {0.0, 90.0, 0.0});
  const double size = 0.638;

  std::puts("== 1. extrinziky z vision.yaml (pitch 90°) ==");
  const cv::Matx33d expected(0, -1, 0, -1, 0, 0, 0, 0, -1);
  check(cv::norm(cv::Mat(ex.R_body_optical - expected)) < 1e-9,
    "R_body_optical = [0 -1 0; -1 0 0; 0 0 -1]",
    cv::norm(cv::Mat(ex.R_body_optical - expected)), 1e-9);
  const cv::Vec3d fwd = lrs_vision::opticalToBody({0.0, -1.0, 5.0}, ex);  // hore v obraze
  check(fwd[0] > 0.99, "obraz hore -> dopredu drona (x_telo)", fwd[0], 1.0);
  const cv::Vec3d right = lrs_vision::opticalToBody({1.0, 0.0, 5.0}, ex);  // vpravo v obraze
  check(right[1] < -0.99, "obraz vpravo -> vpravo od drona (y_telo < 0)", right[1], -1.0);
  const cv::Vec3d down = lrs_vision::opticalToBody({0.0, 0.0, 5.0}, ex);
  check(std::abs(down[2] + 5.03) < 1e-9, "stred obrazu 5 m -> z_telo = -5.03", down[2], -5.03);

  std::puts("== 2. solvePnP: znacka 9.68 m pod kamerou ==");
  {
    const cv::Vec3d drone(0, 0, 10.03), marker(0.6, -0.4, 0.32);
    auto px = projectMarker(cam, drone, 0.0, marker, 0.3, size, 0.0, rng);
    auto p = lrs_vision::markerInOptical(px, cam, size);
    check(p.has_value(), "solvePnP vratil vysledok", p ? 1.0 : 0.0, 1.0);
    if (p) {
      check(std::abs((*p)[2] - 9.68) < 0.01, "vzdialenost z_opt [m] bez sumu", (*p)[2], 9.68);
    }
  }

  std::puts("== 3. cely reťazec rohy -> ENU (dron natoceny, sum 0.5 px) ==");
  struct Case {const char * name; cv::Vec3d drone; double yaw; cv::Vec3d marker; double myaw;};
  const Case cases[] = {
    {"vznasanie nad roverom 1 (h=10 m, yaw 0)", {-7.0, 2.0, 10.0}, 0.0, {-7.2, 2.1, 0.32}, 0.0},
    {"rover vpredu vlavo (h=10 m, yaw 30°)", {-7.0, 2.0, 10.0}, M_PI / 6, {-5.5, 3.5, 0.32}, 1.0},
    {"rover pri okraji obrazu (h=8 m, yaw -120°)", {13.0, 2.0, 8.0}, -2.0944, {11.0, 0.5, 0.32}, -0.7},
    {"sledovanie nizko (h=4 m, yaw 170°)", {10.0, -3.0, 4.0}, 2.967, {10.4, -3.3, 0.32}, 2.0},
  };
  for (const auto & c : cases) {
    auto px = projectMarker(cam, c.drone, c.yaw, c.marker, c.myaw, size, 0.5, rng);
    auto p_opt = lrs_vision::markerInOptical(px, cam, size);
    if (!p_opt) {
      check(false, c.name, -1.0, 0.25);
      continue;
    }
    const cv::Vec3d p_enu = lrs_vision::bodyToEnu(
      lrs_vision::opticalToBody(*p_opt, ex), c.drone, yawRotation(c.yaw));
    const double e = dist(p_enu, c.marker);
    check(e < 0.25, c.name, e, 0.25);
    std::printf("       odhad ENU (%.2f, %.2f, %.2f), skutocne (%.2f, %.2f, %.2f)\n",
      p_enu[0], p_enu[1], p_enu[2], c.marker[0], c.marker[1], c.marker[2]);
  }

  std::puts("== 4. zla velkost znacky preskaluje vzdialenost (len informacia) ==");
  {
    const cv::Vec3d drone(0, 0, 10.03), marker(0, 0, 0.32);
    auto px = projectMarker(cam, drone, 0.0, marker, 0.0, size, 0.0, rng);
    for (double s : {0.638, 0.85, 1.0}) {
      auto p = lrs_vision::markerInOptical(px, cam, s);
      if (p) {
        std::printf("       marker_size %.3f m -> vzdialenost %.2f m (skutocna 9.68 m)\n", s, (*p)[2]);
      }
    }
  }

  std::puts("== 5. neplatne vstupy ==");
  {
    CameraIntrinsics bad;  // valid = false
    std::vector<cv::Point2f> four(4, cv::Point2f(320, 240));
    check(!lrs_vision::markerInOptical(four, bad, size), "bez camera_info -> ziadny vysledok", 0, 0);
    std::vector<cv::Point2f> three(3, cv::Point2f(320, 240));
    check(!lrs_vision::markerInOptical(three, cam, size), "3 rohy -> ziadny vysledok", 0, 0);
  }

  std::printf("\n%s (%d FAIL)\n", g_fails == 0 ? "Vsetko PASS" : "NEPRESLO", g_fails);
  return g_fails == 0 ? 0 : 1;
}
