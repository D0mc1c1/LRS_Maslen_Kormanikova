#include "lrs_planning/voxel_map.hpp"

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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
  // TODO 1.1: nx, ny, nz, vector<uint8_t> occ, naplnit z bodov
  return true;
}

}  // namespace lrs_planning
