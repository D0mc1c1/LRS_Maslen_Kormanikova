#include "lrs_control/follow_executor.hpp"

#include <utility>

namespace lrs_control
{

FollowExecutor::FollowExecutor(Vehicle & vehicle, FollowParams params, Logger log)
: vehicle_(vehicle), p_(std::move(params)), log_(std::move(log))
{
}

const char * FollowExecutor::name(State s)
{
  switch (s) {
    case State::WAIT_CONNECTION: return "WAIT_CONNECTION";
    case State::SET_MODE: return "SET_MODE";
    case State::ARM: return "ARM";
    case State::TAKEOFF: return "TAKEOFF";
    case State::SEARCH: return "SEARCH";
    case State::TRACK: return "TRACK";
    case State::ABORT: return "ABORT";
  }
  return "?";
}

void FollowExecutor::tick(double now, const std::vector<TargetObs> & obs)
{
  // TODO
  (void)now;
  (void)obs;
}

void FollowExecutor::abort(const std::string & why)
{
  // TODO
  (void)why;
}

void FollowExecutor::onWaitConnection(const VehicleState & st)
{
  // TODO
  (void)st;
}

void FollowExecutor::onSetMode(const VehicleState & st, double now)
{
  // TODO
  (void)st;
  (void)now;
}

void FollowExecutor::onArm(const VehicleState & st, double now)
{
  // TODO
  (void)st;
  (void)now;
}

void FollowExecutor::onTakeoff(const VehicleState & st, double now)
{
  // TODO
  (void)st;
  (void)now;
}

void FollowExecutor::onSearch(
  const VehicleState & st, double now, const std::vector<TargetObs> & obs)
{
  // TODO
  (void)st;
  (void)now;
  (void)obs;
}

void FollowExecutor::onTrack(
  const VehicleState & st, double now, const std::vector<TargetObs> & obs)
{
  // TODO
  (void)st;
  (void)now;
  (void)obs;
}

void FollowExecutor::onAbort(const VehicleState & st, double now)
{
  // TODO
  (void)st;
  (void)now;
}

}  // namespace lrs_control
