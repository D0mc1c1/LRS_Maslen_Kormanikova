// Zjednodusenie cesty (line-of-sight) a kontrola kolizii usekov.
// Presunute z apps/plan_cli.cpp do kniznice, aby ho pouzival aj lrs_control.
#include "lrs_planning/path_simplifier.hpp"

#include <cmath>

namespace lrs_planning
{

bool isLineFree(
  const Vec3& p1,
  const Vec3& p2,
  const VoxelMap& map,
  double safety_radius,
  double voxel_size)
{
  // Počet krokov pre interpoláciu
  double dist = std::sqrt(
    (p2.x-p1.x)*(p2.x-p1.x) +
    (p2.y-p1.y)*(p2.y-p1.y) +
    (p2.z-p1.z)*(p2.z-p1.z)
  );
  int steps = static_cast<int>(dist / (voxel_size * 0.5)) + 1;

  
  for (int i = 0; i <= steps; ++i) {
    double t = (steps > 0) ? static_cast<double>(i) / steps : 0.0;
    Vec3 p{
      p1.x + t * (p2.x - p1.x),
      p1.y + t * (p2.y - p1.y),
      p1.z + t * (p2.z - p1.z)
    };

    if (map.isOccupied(p.x, p.y, p.z, safety_radius)) {
      return false;
    }
  }
  return true;
}


Path lineOfSightShortcut(
  const Path& in_path,
  const VoxelMap& map,
  double safety_radius,
  double voxel_size)
{
  if (in_path.size() <= 2) {
    return in_path;
  }

  Path out_path;
  out_path.push_back(in_path[0]);

  size_t i = 0;
  while (i < in_path.size() - 1) {
    size_t best = i + 1;

    for (size_t j = in_path.size() - 1; j > i; --j) {
      if (isLineFree(in_path[i], in_path[j], map, safety_radius, voxel_size)) {
        best = j;
        break;
      }
    }

    if (best != i) {
      out_path.push_back(in_path[best]);
      i = best;
    } else {
      break;
    }
  }

  return out_path;
}


// Kontrola po zjednoduseni: KAZDY usek (nielen body) musi byt volny.
// A* overuje len stredy voxelov; usecka medzi dvoma susednymi stredmi moze
// prejst k prekazke blizsie o max. polovicu uhlopriecky voxelu (voxel*sqrt(3)/2,
// 8.7 cm pri 10 cm voxeli). Kontrola preto pouziva r - tato tolerancia.
double checkRadius(double r, double voxel) {return r - voxel * std::sqrt(3.0) / 2.0;}

bool pathIsFree(
  const Path & path, const VoxelMap & map,
  double safety_radius, double voxel_size)
{
  for (size_t k = 0; k + 1 < path.size(); ++k) {
    if (!isLineFree(path[k], path[k + 1], map, safety_radius, voxel_size)) {
      return false;
    }
  }
  return true;
}

}  // namespace lrs_planning
