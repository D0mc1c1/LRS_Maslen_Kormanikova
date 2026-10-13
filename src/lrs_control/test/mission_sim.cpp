// mission_sim <mission.csv>
//
// Offline test stavoveho automatu BEZ ROS a bez Gazeba. SimVehicle je hruby
// kinematicky model drona, ktory sa sprava ako ArduPilot v podstatnych veciach:
//  - arm len v GUIDED, takeoff len ked je armnuty
//  - pozicny setpoint je latched, rychlost a zrychlenie su obmedzene
//  - LAND: klesa a po dosadnuti sa SAM disarmne (presne ako ArduPilot)
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
#include "lrs_planning/astar_planner.hpp"
#include "lrs_planning/planner.hpp"

using namespace lrs_control;

class SimVehicle : public Vehicle
{
public:
  explicit SimVehicle(const Vec3 & spawn) {s_.pos = spawn; s_.mode = "STABILIZE";}

  VehicleState state() const override {return s_;}

  void requestMode(const std::string & m) override {pend("mode", [this, m] {s_.mode = m; return true;});}
  void requestArm(bool a) override
  {
    pend("arm", [this, a] {
        if (s_.mode != "GUIDED") {return false;}
        s_.armed = a;
        armed_t_ = t_;
        return true;
      });
  }
  void requestTakeoff(double z) override
  {
    pend("takeoff", [this, z] {
        if (!s_.armed || s_.mode != "GUIDED" || airborne_) {return false;}
        target_ = {s_.pos.x, s_.pos.y, z};
        airborne_ = true;
        return true;
      });
  }
  void requestLand() override {pend("land", [this] {s_.mode = "LAND"; return true;});}
  RequestStatus requestStatus() const override {return status_;}

  void sendPositionTarget(const Vec3 & p, double yaw) override
  {
    if (s_.mode == "GUIDED" && s_.armed && airborne_) {
      target_ = p;
      yaw_target_ = yaw;
    }
    ++setpoints_;
  }

  void step(double dt)
  {
    t_ += dt;
    s_.connected = t_ > 0.5;
    s_.has_pose = t_ > 0.5;
    if (pending_ && t_ >= pending_t_) {   // odpoved sluzby po 0.2 s
      status_ = pending_fn_() ? RequestStatus::Ok : RequestStatus::Failed;
      pending_ = false;
    }
    // ArduPilot: armnuty na zemi bez vzletu -> po 10 s disarm
    if (s_.armed && !airborne_ && t_ - armed_t_ > 10.0) {s_.armed = false;}

    Vec3 v_des{0, 0, 0};
    if (s_.mode == "LAND" && s_.armed) {
      v_des = {0, 0, -0.5};
      if (s_.pos.z <= 0.01) {
        s_.pos.z = 0;
        land_t_ += dt;
        if (land_t_ > 1.0) {s_.armed = false; airborne_ = false; land_t_ = 0;}
      }
    } else if (s_.mode == "GUIDED" && airborne_) {
      // P regulator polohy s obmedzenim rychlosti (vmax 1.5 m/s)
      const double k = 1.2, vmax = 1.5;
      v_des = {k * (target_.x - s_.pos.x), k * (target_.y - s_.pos.y), k * (target_.z - s_.pos.z)};
      const double n = norm(v_des);
      if (n > vmax) {v_des = {v_des.x * vmax / n, v_des.y * vmax / n, v_des.z * vmax / n};}
      // yaw rychlostou max 60 deg/s
      const double e = angleDiff(yaw_target_, s_.yaw);
      const double step = deg2rad(60) * dt;
      s_.yaw = wrapPi(s_.yaw + std::clamp(e, -step, step));
    }
    // obmedzenie zrychlenia 2 m/s^2
    const double amax = 2.0 * dt;
    auto lim = [&](double v, double d) {return v + std::clamp(d - v, -amax, amax);};
    s_.vel = {lim(s_.vel.x, v_des.x), lim(s_.vel.y, v_des.y), lim(s_.vel.z, v_des.z)};
    s_.pos = {s_.pos.x + s_.vel.x * dt, s_.pos.y + s_.vel.y * dt,
      std::max(0.0, s_.pos.z + s_.vel.z * dt)};
  }

  long setpoints() const {return setpoints_;}
  double t() const {return t_;}

private:
  template<class F>
  void pend(const char *, F fn)
  {
    pending_ = true;
    pending_t_ = t_ + 0.2;
    pending_fn_ = fn;
    status_ = RequestStatus::Pending;
  }

  VehicleState s_;
  double t_{0}, armed_t_{0}, land_t_{0}, pending_t_{0};
  bool airborne_{false}, pending_{false};
  std::function<bool()> pending_fn_;
  RequestStatus status_{RequestStatus::None};
  Vec3 target_{};
  double yaw_target_{0};
  long setpoints_{0};
};

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
