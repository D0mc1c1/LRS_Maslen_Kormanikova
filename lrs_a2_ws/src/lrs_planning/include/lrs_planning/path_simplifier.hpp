#pragma once
#include "lrs_planning/types.hpp"
#include "lrs_planning/voxel_map.hpp"

namespace lrs_planning
{

// kontroluje ci je usecka p1 -> p2 volna, vzorkuje po voxel/2 a kontroluje isOccupied(..)
bool isLineFree(
  const Vec3 & p1, const Vec3 & p2, const VoxelMap & map,
  double safety_radius, double voxel_size);

// line-of-sight: z bodu i skoci na najvzdialenejsi bod j, ku ktoremu vedie volna usecka
Path lineOfSightShortcut(
  const Path & in_path, const VoxelMap & map, double safety_radius, double voxel_size);

// polomer pre kontrolu po zjednoduseni: r - voxel*sqrt(3)/2 (A* overuje len stredy voxelov)
double checkRadius(double r, double voxel);

// kontroluje ci je cela cesta volna
bool pathIsFree(
  const Path & path, const VoxelMap & map, double safety_radius, double voxel_size);

}
