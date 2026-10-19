#include "lrs_planning/inflation.hpp"

namespace lrs_planning
{

InflatedMap::InflatedMap(const VoxelMap & base_map, double safety_radius)
  : base_map_(base_map), safety_radius_(safety_radius)
{
}

bool InflatedMap::isOccupied(double x, double y, double z) const
{
  return base_map_.isOccupied(x, y, z, safety_radius_);
}

bool InflatedMap::isOccupied(const Vec3 & p) const
{
  return isOccupied(p.x, p.y, p.z);
}

}