#include <iostream>
#include <string>

#include "lrs_planning/voxel_map.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: plan_cli <map.pcd> [voxel=0.1]\n";
    return 1;
  }
  const double voxel = argc > 2 ? std::stod(argv[2]) : 0.1;

  lrs_planning::VoxelMap map;
  if (!map.load(argv[1], voxel)) {
    std::cerr << "Nepodarilo sa nacitat " << argv[1] << "\n";
    return 1;
  }
  const auto mn = map.minBound();
  const auto mx = map.maxBound();
  std::cout << "points: " << map.pointCount() << "\n"
            << "bbox min: " << mn.x << " " << mn.y << " " << mn.z << "\n"
            << "bbox max: " << mx.x << " " << mx.y << " " << mx.z << "\n";
  // TODO: start/goal z argumentov, A*, zjednodusenie, vypis cesty a casu
  return 0;
}
