#pragma once
// Poloha ArUco znacky vo svete (§3): rohy v pixeloch -> solvePnP -> optical frame kamery
// -> telo drona (FLU) -> lokalne ENU drona. Bez ROS, aby sa dalo testovat offline
// (marker_pose_test).
//
// Ramce:
//   optical (REP-103): x vpravo v obraze, y dole v obraze, z smer pohladu
//   telo (FLU):        x dopredu, y dolava, z hore (tak ho dava MAVROS local_position/pose)
//   ENU:               lokalny ramec MAVROS, x vychod, y sever, z hore
#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

namespace lrs_vision
{

struct CameraIntrinsics
{
  cv::Matx33d K{cv::Matx33d::eye()};
  cv::Mat D;  // moze byt prazdne (simulator nema skreslenie)
  bool valid{false};
};

struct CameraExtrinsics
{
  cv::Vec3d t_body_cam{0.0, 0.0, 0.0};                 // poloha kamery v tele drona [m]
  cv::Matx33d R_body_optical{cv::Matx33d::eye()};      // stlpce = osi optical v tele
};

// Intrinziky z camera_info: k = 9 cisel po riadkoch, d = koeficienty skreslenia.
inline CameraIntrinsics intrinsicsFromCameraInfo(
  const std::array<double, 9> & k, const std::vector<double> & d)
{
  CameraIntrinsics cam;
  cam.K = cv::Matx33d(k[0], k[1], k[2], k[3], k[4], k[5], k[6], k[7], k[8]);
  if (!d.empty()) {
    cam.D = cv::Mat(d, true).reshape(1, 1);
  }
  cam.valid = k[0] > 0.0 && k[4] > 0.0;
  return cam;
}

// Rotacia z roll/pitch/yaw (SDF/URDF konvencia: R = Rz(yaw) * Ry(pitch) * Rx(roll)).
inline cv::Matx33d rotationFromRpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll), sr = std::sin(roll);
  const double cp = std::cos(pitch), sp = std::sin(pitch);
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const cv::Matx33d Rx(1, 0, 0, 0, cr, -sr, 0, sr, cr);
  const cv::Matx33d Ry(cp, 0, sp, 0, 1, 0, -sp, 0, cp);
  const cv::Matx33d Rz(cy, -sy, 0, sy, cy, 0, 0, 0, 1);
  return Rz * Ry * Rx;
}

// Extrinziky z pozy linku kamery v tele (rovnako ako <pose> v SDF: xyz [m], rpy [stupne]).
// Kamera v Gazebe (gz) pozera pozdlz svojej +x, y dolava, z hore. Optical frame z nej:
//   x_opt = -y_cam, y_opt = -z_cam, z_opt = +x_cam.
// Pre nasu montaz (pitch +90°) vyjde R_body_optical = [0 -1 0; -1 0 0; 0 0 -1], teda
// p_telo = (-y_opt, -x_opt, -z_opt): obraz vpravo = vpravo od drona, obraz hore = dopredu.
inline CameraExtrinsics extrinsicsFromPose(const cv::Vec3d & xyz, const cv::Vec3d & rpy_deg)
{
  constexpr double kDeg = M_PI / 180.0;
  const cv::Matx33d R_body_cam =
    rotationFromRpy(rpy_deg[0] * kDeg, rpy_deg[1] * kDeg, rpy_deg[2] * kDeg);
  // stlpce = osi optical vyjadrene v ramci kamery gz
  const cv::Matx33d R_cam_optical(
    0, 0, 1,
    -1, 0, 0,
    0, -1, 0);
  CameraExtrinsics ex;
  ex.t_body_cam = xyz;
  ex.R_body_optical = R_body_cam * R_cam_optical;
  return ex;
}

// Stred znacky v optical frame kamery [m]. Rohy v poradi z ArUco: vlavo hore, vpravo hore,
// vpravo dole, vlavo dole. marker_size = strana CIERNEHO stvorca (0.638 m), nie dosky.
// SOLVEPNP_IPPE_SQUARE je presna metoda pre stvorec; estimatePoseSingleMarkers v OpenCV 4.7+ nie je.
inline std::optional<cv::Vec3d> markerInOptical(
  const std::vector<cv::Point2f> & corners, const CameraIntrinsics & cam, double marker_size)
{
  if (!cam.valid || corners.size() != 4 || marker_size <= 0.0) {
    return std::nullopt;
  }
  const double h = marker_size / 2.0;
  const std::vector<cv::Point3f> object = {
    {static_cast<float>(-h), static_cast<float>(h), 0.0f},
    {static_cast<float>(h), static_cast<float>(h), 0.0f},
    {static_cast<float>(h), static_cast<float>(-h), 0.0f},
    {static_cast<float>(-h), static_cast<float>(-h), 0.0f}};
  cv::Vec3d rvec, tvec;
  bool ok = false;
  try {
    ok = cv::solvePnP(
      object, corners, cv::Mat(cam.K), cam.D, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);
  } catch (const cv::Exception &) {
    return std::nullopt;
  }
  if (!ok || tvec[2] <= 0.0) {  // znacka musi byt pred kamerou
    return std::nullopt;
  }
  return tvec;
}

// Optical -> telo drona (FLU).
inline cv::Vec3d opticalToBody(const cv::Vec3d & p_opt, const CameraExtrinsics & ex)
{
  return ex.R_body_optical * p_opt + ex.t_body_cam;
}

// Kvaternion (x, y, z, w) z MAVROS local_position/pose -> rotacia telo -> ENU.
inline cv::Matx33d rotationFromQuaternion(double x, double y, double z, double w)
{
  const double n = std::sqrt(x * x + y * y + z * z + w * w);
  if (n < 1e-9) {
    return cv::Matx33d::eye();
  }
  x /= n; y /= n; z /= n; w /= n;
  return cv::Matx33d(
    1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w),
    2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
    2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y));
}

// Telo -> lokalne ENU: p_enu = poloha drona + R(q) * p_telo, q v okamihu snimky.
inline cv::Vec3d bodyToEnu(
  const cv::Vec3d & p_body, const cv::Vec3d & drone_pos, const cv::Matx33d & R_enu_body)
{
  return drone_pos + R_enu_body * p_body;
}

}  // namespace lrs_vision
