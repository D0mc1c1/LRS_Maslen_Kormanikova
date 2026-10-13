#pragma once
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "lrs_planning/planner.hpp"
#include "lrs_planning/voxel_map.hpp"

namespace lrs_planning
{

// Adapter: A* (planning_algorithm) + line-of-sight zjednodusenie (path_simplifier)
// za rozhranim Planner, ktore pouziva stavovy automat v lrs_control.
class AStarPlanner : public Planner
{
public:
  struct Box { Vec3 lo, hi; };

  // Nacita mapu a prida plne boxy (regaly). Pri chybe vyhodi std::runtime_error.
  // max_z = najvyssia povolena vyska STREDU drona [m] (strop letu). Nad regalmi
  // v mape chybaju body, bez stropu by A* naplanoval let ponad regaly.
  AStarPlanner(
    const std::string & map_path, double voxel_size, double safety_radius,
    const std::vector<Box> & solid_boxes,
    double max_z = std::numeric_limits<double>::infinity());

  std::optional<PlanResult> plan(const Vec3 & start, const Vec3 & goal) override;

  const VoxelMap & map() const {return map_;}

private:
  VoxelMap map_;
  double voxel_;
  double radius_;
};

}  // namespace lrs_planning
