#include <iostream>
#include <string>
#include <algorithm>
#include <cmath>


#include "lrs_planning/voxel_map.hpp"
#include "lrs_planning/inflation.hpp"
#include "lrs_planning/planning_algorithm.hpp"

static bool isLineFree(
  const lrs_planning::Vec3& p1,
  const lrs_planning::Vec3& p2,
  const lrs_planning::VoxelMap& map,
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
    lrs_planning::Vec3 p{
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


static lrs_planning::Path lineOfSightShortcut(
  const lrs_planning::Path& in_path,
  const lrs_planning::VoxelMap& map,
  double safety_radius,
  double voxel_size)
{
  if (in_path.size() <= 2) {
    return in_path;
  }

  lrs_planning::Path out_path;
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
  std::cout << "=== MAP LOADED ===\n";
  std::cout << "points: " << map.pointCount() << "\n"
            << "bbox min: " << mn.x << " " << mn.y << " " << mn.z << "\n"
            << "bbox max: " << mx.x << " " << mx.y << " " << mx.z << "\n"
            << "voxel count: " << map.gridX() * map.gridY() * map.gridZ() << "\n"
            << "grid size: " << map.gridX() << " x " << map.gridY() << " x " << map.gridZ() << "\n\n";
  
  
  lrs_planning::Vec3 start{1.0, 1.0, 0.5};
  lrs_planning::Vec3 goal{12.0, 9.0, 2.0};
  double safety_radius = 0.2;
  
  std::cout << "=== PLANNING ===\n";
  std::cout << "Start: " << start.x << " " << start.y << " " << start.z << "\n";
  std::cout << "Goal: " << goal.x << " " << goal.y << " " << goal.z << "\n";
  std::cout << "Safety radius: " << safety_radius << " m\n\n";
  
  lrs_planning::InflatedMap inflated_map(map, safety_radius);

  lrs_planning::Path path;
  double planning_time;
  
  bool found = lrs_planning::astar(inflated_map.baseMap(), start, goal, safety_radius, voxel, path, planning_time);
  
    std::cout << "\n=== RESULT ===\n";
  if (found) {
    std::cout << "✓ Path found!\n";
    std::cout << "Planning time: " << planning_time << " ms\n";

    std::cout << "Path simplification:\n";
    std::cout << "  Before: " << path.size() << " waypoints\n";
    

    lrs_planning::Path simplified = lineOfSightShortcut(path, map, safety_radius, voxel);
    std::cout << "  After (line-of-sight): " << simplified.size() << " waypoints\n";
    
    std::cout << "  Removed: " << (path.size() - simplified.size()) << " points\n";
    std::cout << "  Reduction: " << (100.0 * (path.size() - simplified.size()) / path.size()) << "%\n\n";

    // Skontroluj že simplified cesta je bezpečná
    bool safe = true;
    for (const auto& p : simplified) {
      if (map.isOccupied(p.x, p.y, p.z, safety_radius)) {
        safe = false;
        break;
      }
    }
    std::cout << "Safety check: " << (safe ? "✓ PASSED" : "✗ FAILED") << "\n\n";

    std::cout << "Simplified path waypoints:\n";
    for (size_t i = 0; i < simplified.size(); ++i) {
      std::cout << i << ": (" << simplified[i].x << ", " << simplified[i].y << ", " << simplified[i].z << ")\n";
    }
  } else {
    std::cout << "✗ Path NOT found!\n";
    std::cout << "Planning time: " << planning_time << " ms\n";
  }
  
  return 0;
}
