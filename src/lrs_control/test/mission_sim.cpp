// mission_sim <mission.csv>
//
// Offline test stavoveho automatu BEZ ROS a bez Gazeba (simulovany dron: sim_vehicle.hpp).
// SIM_DETOUR=1 mission_sim ... -> namiesto priamok cesta so 4 medzibodmi (ako z A*)
// SIM_MAP=<map.pcd> mission_sim ... -> skutocny A* z A1.1
// Pouzitie: rychlo overit logiku (poradie taskov, landtakeoff, yaw) pred letom.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <thread>
#include <chrono>

#include "lrs_control/geometry.hpp"
#include "lrs_control/mission_executor.hpp"
#include "sim_vehicle.hpp"
#include "lrs_planning/astar_planner.hpp"
#include "lrs_planning/planner.hpp"

using namespace lrs_control;

// Napodobni vystup A*: medzi start a goal vlozi 4 medzibody s bocnym ofsetom.
// Overuje, ze cez medzibody dron preleti bez zastavenia.
class DetourPlanner : public lrs_planning::Planner
{
public:
  std::optional<lrs_planning::PlanResult> plan(const Vec3 & a, const Vec3 & b) override
  {
    lrs_planning::PlanResult r;
    r.path.push_back(a);
    const double L = distXY(a, b);
    const double nx = L > 1e-6 ? -(b.y - a.y) / L : 0.0, ny = L > 1e-6 ? (b.x - a.x) / L : 0.0;
    for (int i = 1; i <= 4; ++i) {
      const double t = i / 5.0, off = (i % 2 ? 0.8 : -0.8) * (L > 2.0);
      r.path.push_back({a.x + t * (b.x - a.x) + off * nx, a.y + t * (b.y - a.y) + off * ny,
          a.z + t * (b.z - a.z)});
    }
    r.path.push_back(b);
    r.raw_points = 40;
    return r;
  }
};

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cerr << "usage: mission_sim <mission.csv> [spawn_x spawn_y]\n";
    return 1;
  }
  Mission mission;
  try {
    mission = loadMission(argv[1]);
  } catch (const std::exception & e) {
    std::cerr << e.what() << "\n";
    return 1;
  }
  const Vec3 spawn{argc > 3 ? std::stod(argv[2]) : 13.0, argc > 3 ? std::stod(argv[3]) : 7.0, 0.0};

  SimVehicle veh(spawn);
  lrs_planning::StraightLinePlanner straight;
  DetourPlanner detour;
  const bool use_detour = std::getenv("SIM_DETOUR") != nullptr;
  // SIM_MAP=<map.pcd> -> skutocny A* z A1.1 (s regalmi ako boxy, inflacia 0.7 m, strop 3.0 m)
  std::unique_ptr<lrs_planning::AStarPlanner> astar;
  if (const char * map = std::getenv("SIM_MAP")) {
    // rovnake hodnoty ako v config/indoor.yaml (inflacia 0.7 m, strop 3.0 m)
    astar = std::make_unique<lrs_planning::AStarPlanner>(map, 0.10, 0.70,
        std::vector<lrs_planning::AStarPlanner::Box>{
          {{1.07, 6.29, 0.0}, {7.57, 7.34, 4.10}}, {{1.07, 10.84, 0.0}, {7.57, 11.89, 4.10}}}, 3.0);
  }
  lrs_planning::Planner & planner = astar ? static_cast<lrs_planning::Planner &>(*astar) :
    use_detour ? static_cast<lrs_planning::Planner &>(detour) : straight;
  MissionExecutor ex(veh, planner, mission, ExecutorParams{},
    [&veh](MissionExecutor::LogLevel l, const std::string & m) {
      const char * tag = l == MissionExecutor::LogLevel::Error ? "ERROR" :
      l == MissionExecutor::LogLevel::Warn ? "WARN " : "INFO ";
      std::cout << tag << " " << m << "\n";
    });

  const double dt = 0.01;   // simulacia 100 Hz
  int k = 0;
  while (!ex.finished() && veh.t() < 900.0) {
    if (ex.state() == MissionExecutor::State::PLAN) {
      // planovanie bezi vo vlakne v realnom case -> simulacia pocka
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    veh.step(dt);
    if (++k % 5 == 0) {     // riadenie 20 Hz
      ex.tick(veh.t());
    }
  }
  const auto st = veh.state();
  std::printf("\nkoniec: stav=%s t=%.1fs armed=%d pos=(%.2f, %.2f, %.2f) setpointov=%ld\n",
    MissionExecutor::name(ex.state()), veh.t(), st.armed, st.pos.x, st.pos.y, st.pos.z,
    veh.setpoints());
  return ex.state() == MissionExecutor::State::DONE ? 0 : 2;
}
