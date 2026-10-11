#pragma once
#include "lrs_planning/types.hpp"
#include "lrs_planning/voxel_map.hpp"

namespace lrs_planning
{
bool astar(
  const VoxelMap& map,
  Vec3 start,
  Vec3 goal,
  double safety_radius,
  double voxel_size,
  Path& out_path,
  double& out_time_ms);

}  // namespace lrs_planning