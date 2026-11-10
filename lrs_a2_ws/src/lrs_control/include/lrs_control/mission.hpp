#pragma once
#include <string>
#include <vector>

#include "lrs_planning/types.hpp"

namespace lrs_control
{

enum class Precision { Soft, Hard };
enum class TaskType { None, Takeoff, Land, LandTakeoff, Yaw };

struct Waypoint
{
  lrs_planning::Vec3 p;          // map frame, metre
  Precision precision{Precision::Soft};
  TaskType task{TaskType::None};
  double yaw_deg{0.0};           // len pre TaskType::Yaw, ENU absolutne
  int line{0};                   // riadok v CSV (pre logy)
};

using Mission = std::vector<Waypoint>;

// Nacita CSV "x,y,z,precision,task". Ignoruje prazdne riadky, riadky s '#'
// a hlavicku. Pri chybe vyhodi std::runtime_error s cislom riadku.
Mission loadMission(const std::string & path);

std::string toString(Precision p);
std::string toString(TaskType t);
std::string describe(const Waypoint & wp);  // "WP(8.65, 2.02, 1.00) soft yaw180"

}
