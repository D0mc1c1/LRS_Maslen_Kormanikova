#include "lrs_planning/astar_planner.hpp"

#include <cmath>
#include <stdexcept>

#include "lrs_planning/path_simplifier.hpp"
#include "lrs_planning/planning_algorithm.hpp"

namespace lrs_planning
{

AStarPlanner::AStarPlanner(
  const std::string & map_path, double voxel_size, double safety_radius,
  const std::vector<Box> & solid_boxes, double max_z)
: voxel_(voxel_size), radius_(safety_radius)
{
  if (!map_.load(map_path, voxel_size)) {
    throw std::runtime_error("nepodarilo sa nacitat mapu: " + map_path);
  }
  for (const auto & b : solid_boxes) {
    map_.addSolidBox(b.lo, b.hi);
  }
  if (std::isfinite(max_z)) {
    // strop = plny box nad celou mapou. Spodok je o safety_radius vyssie,
    // lebo isOccupied box zvacsi o safety_radius -> stred drona moze ist az po max_z.
    const auto mn = map_.minBound();
    const auto mx = map_.maxBound();
    map_.addSolidBox({mn.x - 10.0, mn.y - 10.0, max_z + safety_radius},
      {mx.x + 10.0, mx.y + 10.0, mx.z + 100.0});
  }
}

std::optional<PlanResult> AStarPlanner::plan(const Vec3 & start, const Vec3 & goal)
{
  Path raw;
  double ms = 0.0;
  if (!astar(map_, start, goal, radius_, voxel_, raw, ms)) {
    return std::nullopt;
  }
  PlanResult r;
  r.time_ms = ms;
  r.raw_points = raw.size();
  r.path = lineOfSightShortcut(raw, map_, radius_, voxel_);
  // kontrola po zjednoduseni: kazdy usek musi byt volny
  if (!pathIsFree(r.path, map_, checkRadius(radius_, voxel_), voxel_)) {
    return std::nullopt;
  }
  return r;
}

}  // namespace lrs_planning
