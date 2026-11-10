#pragma once
#include <functional>
#include <string>
#include <vector>

#include "lrs_control/vehicle.hpp"

namespace lrs_control
{

struct TargetObs
{
  double t{0.0};
  Vec3 p{};
};

struct FollowParams
{
  int sysid{1};
  double search_alt{10.0};
  double follow_alt{6.0};
  double lost_timeout{1.5};
  int confirm_count{3};
  double confirm_window{1.0};
  std::string abort_mode{"LOITER"};
  // TODO: dalsie parametre
};

class FollowExecutor
{
public:
  enum class State { WAIT_CONNECTION, SET_MODE, ARM, TAKEOFF, SEARCH, TRACK, ABORT };

  using Logger = std::function<void(const std::string &)>;

  FollowExecutor(Vehicle & vehicle, FollowParams params, Logger log);

  void tick(double now, const std::vector<TargetObs> & obs);
  void abort(const std::string & why);

  State state() const {return state_;}
  static const char * name(State s);

private:
  void onWaitConnection(const VehicleState & st);
  void onSetMode(const VehicleState & st, double now);
  void onArm(const VehicleState & st, double now);
  void onTakeoff(const VehicleState & st, double now);
  void onSearch(const VehicleState & st, double now, const std::vector<TargetObs> & obs);
  void onTrack(const VehicleState & st, double now, const std::vector<TargetObs> & obs);
  void onAbort(const VehicleState & st, double now);

  Vehicle & vehicle_;
  FollowParams p_;
  Logger log_;
  State state_{State::WAIT_CONNECTION};
};

}  // namespace lrs_control
