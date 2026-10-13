#include "lrs_planning/voxel_map.hpp"

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <algorithm>
#include <cmath>

namespace lrs_planning
{

bool VoxelMap::load(const std::string & pcd_path, double voxel_size)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, cloud) < 0) {
    return false;
  }
  voxel_ = voxel_size;
  point_count_ = cloud.size();

  pcl::PointXYZ mn, mx;
  pcl::getMinMax3D(cloud, mn, mx);
  min_ = {mn.x, mn.y, mn.z};
  max_ = {mx.x, mx.y, mx.z};
  
  //pocet voxelov
  nx_ = static_cast<size_t>(std::ceil((max_.x - min_.x) / voxel_size));
  ny_ = static_cast<size_t>(std::ceil((max_.y - min_.y) / voxel_size));
  nz_ = static_cast<size_t>(std::ceil((max_.z - min_.z) / voxel_size));
  
  nx_ = std::max(nx_, size_t(1));
  ny_ = std::max(ny_, size_t(1));
  nz_ = std::max(nz_, size_t(1));

  //vytvorenie gridu
  occupancy_.assign(nx_ * ny_ * nz_, 0);

  //naplnenie gridu a oznacenie voxelov, ze su obsadene
  for (const auto& point : cloud) {
    int ix, iy, iz;
    if (coordToIndex(point.x, point.y, point.z, ix, iy, iz)) {
      size_t idx = indexToLinear(ix, iy, iz);
      occupancy_[idx] = 1;
    }
  }

  return true;
}

bool VoxelMap::coordToIndex(double x, double y, double z, int& ix, int& iy, int& iz) const
{
  ix = static_cast<int>((x - min_.x) / voxel_);
  iy = static_cast<int>((y - min_.y) / voxel_);
  iz = static_cast<int>((z - min_.z) / voxel_);

  //overenie ci sa nachadza bod v gride
  if (ix < 0 || ix >= static_cast<int>(nx_) ||
      iy < 0 || iy >= static_cast<int>(ny_) ||
      iz < 0 || iz >= static_cast<int>(nz_)) {
    return false;
  }
  return true;
}

size_t VoxelMap::indexToLinear(int ix, int iy, int iz) const
{
  return ix + iy * nx_ + iz * nx_ * ny_;
}


bool VoxelMap::isOccupied(double x, double y, double z, double safety_radius) const
{
  // kvader zvacseny o safety_radius v kazdej osi (inflacia boxu)
  for (const auto & b : boxes_) {
    if (x > b.lo.x - safety_radius && x < b.hi.x + safety_radius &&
        y > b.lo.y - safety_radius && y < b.hi.y + safety_radius &&
        z > b.lo.z - safety_radius && z < b.hi.z + safety_radius) {
      return true;
    }
  }

  int ix, iy, iz;
  
  //bez safety radius
  if (safety_radius <= 0.0) {
    if (!coordToIndex(x, y, z, ix, iy, iz)) {
      return true;  // Mimo gridu = obsadené
    }
    return occupancy_[indexToLinear(ix, iy, iz)] != 0;
  }

  //safety radius
  int radius_voxels = static_cast<int>(std::ceil(safety_radius / voxel_));
  
  if (!coordToIndex(x, y, z, ix, iy, iz)) {
    return true;
  }

  for (int dx = -radius_voxels; dx <= radius_voxels; ++dx) {
    for (int dy = -radius_voxels; dy <= radius_voxels; ++dy) {
      for (int dz = -radius_voxels; dz <= radius_voxels; ++dz) {
        int cx = ix + dx, cy = iy + dy, cz = iz + dz;
        
        if (cx < 0 || cx >= static_cast<int>(nx_) ||
            cy < 0 || cy >= static_cast<int>(ny_) ||
            cz < 0 || cz >= static_cast<int>(nz_)) {
          continue;
        }
        
        if (occupancy_[indexToLinear(cx, cy, cz)] != 0) {

          Vec3 voxel_center = {
            min_.x + (cx + 0.5) * voxel_,
            min_.y + (cy + 0.5) * voxel_,
            min_.z + (cz + 0.5) * voxel_
          };
          double dx_real = x - voxel_center.x;
          double dy_real = y - voxel_center.y;
          double dz_real = z - voxel_center.z;
          double dist = std::sqrt(dx_real*dx_real + dy_real*dy_real + dz_real*dz_real);
          
          if (dist < safety_radius) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

bool VoxelMap::isOccupiedCloud(double x, double y, double z) const
{
  int ix, iy, iz;
  if (!coordToIndex(x, y, z, ix, iy, iz)) {
    return false;
  }
  return occupancy_[indexToLinear(ix, iy, iz)] != 0;
}

bool VoxelMap::isOccupied(const Vec3 & p, double safety_radius) const
{
  return isOccupied(p.x, p.y, p.z, safety_radius);
}

}  // namespace lrs_planning
