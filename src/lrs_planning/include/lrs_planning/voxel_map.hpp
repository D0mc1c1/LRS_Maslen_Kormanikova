#pragma once
#include <cstddef>
#include <string>

#include "lrs_planning/types.hpp"

namespace lrs_planning
{

// Husta 3D mriezka obsadenosti. Zatial len kostra: load() nacita PCD
// a zapamata si bounding box. Mriezku doplnis vo faze 1.1.
class VoxelMap
{
public:
  bool load(const std::string & pcd_path, double voxel_size);

  std::size_t pointCount() const {return point_count_;}
  Vec3 minBound() const {return min_;}
  Vec3 maxBound() const {return max_;}

private:
  std::size_t point_count_{0};
  double voxel_{0.1};
  Vec3 min_{}, max_{};
};

}  // namespace lrs_planning
