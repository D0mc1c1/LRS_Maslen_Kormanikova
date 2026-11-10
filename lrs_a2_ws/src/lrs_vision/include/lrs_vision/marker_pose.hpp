#pragma once
#include <optional>
#include <vector>

#include <opencv2/core.hpp>

namespace lrs_vision
{

struct CameraIntrinsics
{
  cv::Matx33d K{cv::Matx33d::eye()};
  cv::Mat D;
  bool valid{false};
};

struct CameraExtrinsics
{
  cv::Vec3d t_body_cam{0.0, 0.0, 0.0};
  cv::Matx33d R_body_optical{cv::Matx33d::eye()};
};

// TODO: solvePnP
inline std::optional<cv::Vec3d> markerInOptical(
  const std::vector<cv::Point2f> & corners, const CameraIntrinsics & cam, double marker_size)
{
  (void)corners;
  (void)cam;
  (void)marker_size;
  return std::nullopt;
}

// TODO
inline cv::Vec3d opticalToBody(const cv::Vec3d & p_opt, const CameraExtrinsics & ex)
{
  (void)ex;
  return p_opt;
}

// TODO
inline cv::Vec3d bodyToEnu(
  const cv::Vec3d & p_body, const cv::Vec3d & drone_pos, const cv::Matx33d & R_enu_body)
{
  (void)drone_pos;
  (void)R_enu_body;
  return p_body;
}

}  // namespace lrs_vision
