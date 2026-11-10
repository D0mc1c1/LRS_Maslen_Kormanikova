#pragma once
#include "lrs_planning/types.hpp"
#include "lrs_planning/voxel_map.hpp"

namespace lrs_planning
{

class InflatedMap
{
public:
  InflatedMap(const VoxelMap & base_map, double safety_radius);
  
  bool isOccupied(double x, double y, double z) const;
  bool isOccupied(const Vec3 & p) const;
  
  double safetyRadius() const { return safety_radius_; }
  const VoxelMap & baseMap() const { return base_map_; }

private:
  const VoxelMap & base_map_;
  double safety_radius_;
};

}