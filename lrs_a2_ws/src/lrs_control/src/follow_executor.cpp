#include "lrs_control/follow_executor.hpp"

#include <cstdio>
#include <utility>

namespace lrs_control
{

namespace
{
std::string fmt(const char * f, double v)
{
  char b[32];
  std::snprintf(b, sizeof(b), f, v);
  return b;
}
}

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
  now_ = now;
  const VehicleState st = vehicle_.state();

  // spojenie stratene -> prestat riadit
  if (controlState()) {
    std::string why;
    if (!linkOk(st, p_.state_timeout, p_.pose_timeout, why)) {
      enter(State::ABORT, "spojenie stratene: " + why);
    }
  }
  // pilot prevzal riadenie -> nebojovat s nim
  if (airborneState() && (st.mode != "GUIDED" || !st.armed)) {
    enter(State::ABORT, "mod zmeneny zvonka na '" + st.mode + "'");
  }

  switch (state_) {
    case State::WAIT_CONNECTION: onWaitConnection(st); break;
    case State::SET_MODE: onSetMode(st, now); break;
    case State::ARM: onArm(st, now); break;
    case State::TAKEOFF: onTakeoff(st, now); break;
    case State::SEARCH: onSearch(st, now, obs); break;
    case State::TRACK: onTrack(st, now, obs); break;
    case State::ABORT: onAbort(st, now); break;
  }
}

void FollowExecutor::abort(const std::string & why)
{
  // TODO: abort_mode (LOITER / BRAKE / RTL)
  enter(State::ABORT, why);
}

// ---------------------------------------------------------------------------
// pomocne
// ---------------------------------------------------------------------------
void FollowExecutor::enter(State s, const std::string & why)
{
  if (state_ == State::ABORT) {
    return;
  }
  if (log_) {
    log_(std::string(name(state_)) + " -> " + name(s) + "  (" + why + ")");
  }
  state_ = s;
  state_t0_ = now_;
  req_sent_t_ = -1.0;
}

bool FollowExecutor::requestDue() const
{
  if (req_sent_t_ < 0.0) {
    return true;
  }
  const double since = now_ - req_sent_t_;
  return (vehicle_.requestStatus() == RequestStatus::Failed && since > 1.0) ||
         since > p_.request_retry;
}

void FollowExecutor::markRequestSent()
{
  req_sent_t_ = now_;
}

bool FollowExecutor::controlState() const
{
  return state_ == State::SET_MODE || state_ == State::ARM || state_ == State::TAKEOFF ||
         airborneState();
}

bool FollowExecutor::airborneState() const
{
  return state_ == State::SEARCH || state_ == State::TRACK;
}

// ---------------------------------------------------------------------------
// stavy
// ---------------------------------------------------------------------------
void FollowExecutor::onWaitConnection(const VehicleState & st)
{
  if (now_ - wait_log_t_ > 5.0) {
    wait_log_t_ = now_;
    log_(std::string("cakam na MAVROS: connected ") + (st.connected ? "ano" : "nie") +
      ", poloha " + (st.has_pose ? "ano" : "nie"));
  }
  if (st.armed) {
    enter(State::ABORT, "dron je uz armnuty, spusti program s dronom na zemi");
    return;
  }
  if (st.connected && st.has_pose) {
    launch_ = st.pos;
    launch_yaw_ = st.yaw;
    hover_ = {launch_.x, launch_.y, launch_.z + p_.search_alt};
    enter(State::SET_MODE, "MAVROS connected, miesto vzletu x " + fmt("%.2f", launch_.x) +
      " y " + fmt("%.2f", launch_.y));
  }
}

void FollowExecutor::onSetMode(const VehicleState & st, double now)
{
  (void)now;
  if (st.mode == "GUIDED") {
    enter(State::ARM, "GUIDED potvrdeny");
    return;
  }
  if (inState() > p_.mode_timeout) {
    enter(State::ABORT, "GUIDED sa nepodarilo nastavit");
    return;
  }
  if (requestDue()) {
    vehicle_.requestMode("GUIDED");
    markRequestSent();
  }
}

void FollowExecutor::onArm(const VehicleState & st, double now)
{
  (void)now;
  if (st.armed) {
    enter(State::TAKEOFF, "armed potvrdeny, vzlet na " + fmt("%.1f m", p_.search_alt));
    return;
  }
  if (st.mode != "GUIDED") {
    enter(State::SET_MODE, "mod sa zmenil na '" + st.mode + "'");
    return;
  }
  if (inState() > p_.arm_timeout) {
    enter(State::ABORT, "arm sa nepodaril (prearm checky v konzole SITL)");
    return;
  }
  if (requestDue()) {
    vehicle_.requestArm(true);
    markRequestSent();
  }
}

void FollowExecutor::onTakeoff(const VehicleState & st, double now)
{
  (void)now;
  const double h = st.pos.z - launch_.z;
  if (h >= p_.search_alt - p_.takeoff_tol) {
    startSearch(st, "vyska dosiahnuta " + fmt("%.2f m", h));
    return;
  }
  if (st.mode != "GUIDED" && inState() > 1.0) {
    enter(State::ABORT, "pocas vzletu sa mod zmenil na '" + st.mode + "'");
    return;
  }
  if (!st.armed && inState() > 2.0) {
    enter(State::SET_MODE, "dron sa disarmol pred vzletom, opakujem arm");
    return;
  }
  if (inState() > p_.takeoff_timeout) {
    enter(State::ABORT, "vzlet nedosiahol " + fmt("%.1f m", p_.search_alt) +
      " (je " + fmt("%.2f m", h) + ")");
    return;
  }
  // takeoff sa neposiela znova, ked ho ArduPilot prijal a dron stupa
  const auto rs = vehicle_.requestStatus();
  const bool resend = req_sent_t_ < 0.0 ||
    (rs == RequestStatus::Failed && now_ - req_sent_t_ > 1.0) ||
    (rs == RequestStatus::Pending && now_ - req_sent_t_ > p_.request_retry);
  if (resend) {
    vehicle_.requestTakeoff(hover_.z);
    markRequestSent();
  }
}

// ---------------------------------------------------------------------------
// hladanie
// ---------------------------------------------------------------------------
void FollowExecutor::startSearch(const VehicleState & st, const std::string & why)
{
  // stred vzoru = aktualna poloha drona vo vyske hladania
  const Vec3 center{st.pos.x, st.pos.y, launch_.z + p_.search_alt};
  SearchPatternParams sp;
  sp.altitude = p_.search_alt;
  sp.overlap = p_.search_overlap;
  sp.max_radius = p_.search_max_radius;
  sp.cam_hfov_deg = p_.cam_hfov_deg;
  sp.cam_aspect = p_.cam_aspect;
  pattern_ = expandingSquare(center, sp);
  ++pattern_version_;
  startLeg(0, st.pos);
  enter(State::SEARCH, why + ", vzor " + std::to_string(pattern_.size()) + " bodov, rozostup " +
    fmt("%.1f m", searchSpacing(sp)));
}

void FollowExecutor::startLeg(std::size_t i, const Vec3 & from)
{
  leg_i_ = i;
  leg_ = StraightLeg(from, pattern_[i], p_.search_speed, p_.search_accel);
  leg_t0_ = now_;
}

void FollowExecutor::onSearch(
  const VehicleState & st, double now, const std::vector<TargetObs> & obs)
{
  // TODO: potvrdenie cielu -> TRACK
  (void)st;
  (void)obs;

  // setpoint sa posuva po useku, rychlost ide ako feedforward
  const double t = now - leg_t0_;
  TrajectorySetpoint sp;
  sp.p = leg_.pos(t);
  sp.v = leg_.vel(t);
  sp.yaw = launch_yaw_;
  vehicle_.sendTrajectorySetpoint(sp);

  if (t < leg_.duration() || pattern_.size() < 2) {
    return;
  }
  if (leg_i_ + 1 < pattern_.size()) {
    startLeg(leg_i_ + 1, leg_.b);
    log_("SEARCH: roh " + std::to_string(leg_i_) + "/" + std::to_string(pattern_.size() - 1) +
      " x " + fmt("%.1f", leg_.b.x) + " y " + fmt("%.1f", leg_.b.y));
  } else {
    log_("SEARCH: vzor prejdeny, opakujem od stredu");
    startLeg(0, leg_.b);
  }
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
