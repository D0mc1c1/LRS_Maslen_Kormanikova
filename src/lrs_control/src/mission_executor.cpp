#include "lrs_control/mission_executor.hpp"

#include <cmath>
#include <cstdio>
#include <sstream>
#include <utility>

#include "lrs_control/geometry.hpp"

namespace lrs_control
{

namespace
{
std::string fmt(const char * f, double a)
{
  char b[64];
  std::snprintf(b, sizeof(b), f, a);
  return b;
}
std::string pt(const Vec3 & p)
{
  char b[64];
  std::snprintf(b, sizeof(b), "(%.2f, %.2f, %.2f)", p.x, p.y, p.z);
  return b;
}
}  // namespace

MissionExecutor::MissionExecutor(
  Vehicle & vehicle, lrs_planning::Planner & planner, Mission mission,
  ExecutorParams params, Logger log)
: vehicle_(vehicle), planner_(planner), mission_(std::move(mission)), p_(params),
  logger_(std::move(log))
{
}

const char * MissionExecutor::name(State s)
{
  switch (s) {
    case State::WAIT_CONNECTION: return "WAIT_CONNECTION";
    case State::PLAN: return "PLAN";
    case State::SET_MODE: return "SET_MODE";
    case State::ARM: return "ARM";
    case State::TAKEOFF: return "TAKEOFF";
    case State::ROTATE: return "ROTATE";
    case State::NAVIGATE: return "NAVIGATE";
    case State::ARRIVE: return "ARRIVE";
    case State::TASK_YAW: return "TASK_YAW";
    case State::LAND: return "LAND";
    case State::GROUND_WAIT: return "GROUND_WAIT";
    case State::NEXT_WAYPOINT: return "NEXT_WAYPOINT";
    case State::DONE: return "DONE";
    case State::ABORT: return "ABORT";
  }
  return "?";
}

// ---------------------------------------------------------------------------
// hlavna slucka
// ---------------------------------------------------------------------------
void MissionExecutor::tick(double now)
{
  now_ = now;
  if (mission_t0_ < 0.0) {
    mission_t0_ = now;
    state_t0_ = now;
    log(LogLevel::Info, std::string("start v stave ") + name(state_));
  }
  sent_this_tick_ = false;
  const VehicleState st = vehicle_.state();

  // --- strazcovia platni pre viac stavov -----------------------------------
  if (state_ != State::WAIT_CONNECTION && !finished() && !st.connected &&
    now - last_disconnect_warn_ > 1.0)
  {
    log(LogLevel::Warn, "MAVROS hlasi connected=false");
    last_disconnect_warn_ = now;
  }
  if (airborneState() && (st.mode != "GUIDED" || !st.armed)) {
    abort("mod zmeneny zvonka na '" + st.mode + "', armed=" + (st.armed ? "1" : "0") +
      " -> prestavam posielat setpointy");
  }

  switch (state_) {
    case State::WAIT_CONNECTION: onWaitConnection(st); break;
    case State::PLAN: onPlan(st); break;
    case State::SET_MODE: onSetMode(st, now); break;
    case State::ARM: onArm(st, now); break;
    case State::TAKEOFF: onTakeoff(st, now); break;
    case State::ROTATE: onRotate(st, now); break;
    case State::NAVIGATE: onNavigate(st, now); break;
    case State::ARRIVE: onArrive(st, now); break;
    case State::TASK_YAW: onTaskYaw(st, now); break;
    case State::LAND: onLand(st, now); break;
    case State::GROUND_WAIT: onGroundWait(now); break;
    case State::NEXT_WAYPOINT: onNextWaypoint(st); break;
    case State::DONE: break;
    case State::ABORT: onAbort(st, now); break;
  }

  // Vo vzduchu posielame setpoint KAZDY tick, aj v ticku, kde sa menil stav.
  if (airborneState() && !sent_this_tick_) {
    send(last_sp_, yaw_sp_);
  }
}

void MissionExecutor::enter(State s, const std::string & why)
{
  char head[96];
  std::snprintf(head, sizeof(head), "[t=%6.2fs] WP %zu/%zu  %s -> %s", now_ - mission_t0_,
    std::min(wp_idx_ + 1, mission_.size()), mission_.size(), name(state_), name(s));
  log(s == State::ABORT ? LogLevel::Error : LogLevel::Info, std::string(head) + "  (" + why + ")");
  state_ = s;
  state_t0_ = now_;
  req_sent_t_ = -1.0;
  req_attempts_ = 0;
  cond_t0_.reset();
  if (s == State::DONE) {
    logSummary();
  }
}

void MissionExecutor::abort(const std::string & why)
{
  if (state_ != State::ABORT) {
    enter(State::ABORT, why);
  }
}

bool MissionExecutor::airborneState() const
{
  return state_ == State::ROTATE || state_ == State::NAVIGATE || state_ == State::ARRIVE ||
         state_ == State::TASK_YAW;
}

void MissionExecutor::send(const Vec3 & p, double yaw)
{
  vehicle_.sendPositionTarget(p, yaw);
  last_sp_ = p;
  sent_this_tick_ = true;
}

void MissionExecutor::log(LogLevel l, const std::string & msg)
{
  if (logger_) {
    logger_(l, msg);
  }
}

bool MissionExecutor::requestDue(double now) const
{
  if (req_sent_t_ < 0.0) {
    return true;
  }
  const auto s = vehicle_.requestStatus();
  const double since = now - req_sent_t_;
  return (s == RequestStatus::Failed && since > 1.0) || since > p_.request_retry;
}

void MissionExecutor::markRequestSent(double now)
{
  if (req_attempts_ > 0) {
    const bool failed = vehicle_.requestStatus() == RequestStatus::Failed;
    log(LogLevel::Warn, std::string(name(state_)) + ": opakujem poziadavku (pokus " +
      std::to_string(req_attempts_ + 1) + ", predosla " + (failed ? "zamietnuta" : "bez odozvy") +
      ")");
  }
  req_sent_t_ = now;
  ++req_attempts_;
}

// ---------------------------------------------------------------------------
// stavy
// ---------------------------------------------------------------------------
void MissionExecutor::onWaitConnection(const VehicleState & st)
{
  if (st.connected && st.has_pose) {
    enter(State::PLAN, "MAVROS connected, poloha " + pt(st.pos) + " v map frame");
  }
}

void MissionExecutor::onPlan(const VehicleState & st)
{
  if (st.armed) {
    abort("dron je uz armnuty, misiu spusti s dronom na zemi");
    return;
  }

  // kontrola prevodu map <-> local: dron by mal stat pod prvym (takeoff) waypointom
  const auto & wp0 = mission_.front();
  if (wp0.task == TaskType::Takeoff && distXY(st.pos, wp0.p) > p_.start_check_dist) {
    log(LogLevel::Warn, "dron stoji v " + pt(st.pos) + ", ale takeoff WP1 je " + pt(wp0.p) +
      " (" + fmt("%.1f", distXY(st.pos, wp0.p)) + " m). Ak dron naozaj stoji na WP1, "
      "je zly prevod map->local (local_from_map_yaw_deg / local_from_map_xyz)!");
  }

  segments_.clear();
  const Vec3 first_start{st.pos.x, st.pos.y, mission_.front().p.z};
  log(LogLevel::Info, "planujem " + std::to_string(mission_.size()) + " segmentov");
  for (std::size_t i = 0; i < mission_.size(); ++i) {
    const Vec3 a = i == 0 ? first_start : mission_[i - 1].p;
    const Vec3 b = mission_[i].p;
    const auto r = planner_.plan(a, b);
    if (!r || r->path.empty()) {
      abort("segment " + std::to_string(i) + ": " + pt(a) + " -> " + pt(b) +
        " NEMA CESTU (riadok " + std::to_string(mission_[i].line) + " v misii)");
      return;
    }
    char b2[160];
    std::snprintf(b2, sizeof(b2), "  segment %zu -> WP%zu %s: %zu bodov (pred zjednodusenim %zu), %.1f ms",
      i, i + 1, describe(mission_[i]).c_str(), r->path.size(), r->raw_points, r->time_ms);
    log(LogLevel::Info, b2);
    segments_.push_back(r->path);
  }
  startTakeoffSequence(mission_.front().p.z, false);
}

void MissionExecutor::onSetMode(const VehicleState & st, double now)
{
  if (st.mode == "GUIDED") {
    enter(State::ARM, "mod GUIDED potvrdeny v /mavros/state");
    return;
  }
  if (inState(now) > p_.mode_timeout) {
    abort("GUIDED sa nepodarilo nastavit (pozri konzolu SITL)");
    return;
  }
  if (requestDue(now)) {
    vehicle_.requestMode("GUIDED");
    markRequestSent(now);
  }
}

void MissionExecutor::onArm(const VehicleState & st, double now)
{
  if (st.armed) {
    enter(State::TAKEOFF, "armed potvrdeny, vzlet na z=" + fmt("%.2f", takeoff_z_));
    return;
  }
  if (st.mode != "GUIDED") {
    enter(State::SET_MODE, "mod sa zmenil na '" + st.mode + "'");
    return;
  }
  if (inState(now) > p_.arm_timeout) {
    abort("arm sa nepodaril (prearm checky v konzole SITL)");
    return;
  }
  if (requestDue(now)) {
    vehicle_.requestArm(true);
    markRequestSent(now);
  }
}

void MissionExecutor::onTakeoff(const VehicleState & st, double now)
{
  if (st.pos.z >= takeoff_z_ - p_.takeoff_tol) {
    hold_pos_ = {st.pos.x, st.pos.y, takeoff_z_};
    last_sp_ = hold_pos_;
    yaw_sp_ = st.yaw;
    stopped_ = true;
    if (resume_next_after_takeoff_) {
      // landtakeoff: aktualny waypoint je hotovy, pokracujeme dalsim
      enter(State::NEXT_WAYPOINT, "vo vzduchu z=" + fmt("%.2f", st.pos.z) + ", landtakeoff hotovy");
    } else {
      // prvy vzlet: dolet do WP1 (ak takeoff nebol presne nad nim)
      log(LogLevel::Info, "vo vzduchu z=" + fmt("%.2f", st.pos.z));
      beginSegment(st);
    }
    return;
  }
  if (st.mode != "GUIDED" && inState(now) > 1.0) {
    abort("pocas vzletu sa mod zmenil na '" + st.mode + "'");
    return;
  }
  if (!st.armed && inState(now) > 2.0) {
    // ArduPilot disarmuje, ak po arme dlho nevzlietne
    enter(State::SET_MODE, "dron sa disarmol pred vzletom, opakujem arm");
    return;
  }
  if (inState(now) > p_.takeoff_timeout) {
    abort("vzlet nedosiahol z=" + fmt("%.2f", takeoff_z_));
    return;
  }
  // takeoff posielame raz; znova len ak bol zamietnuty alebo neprisla odpoved
  const auto rs = vehicle_.requestStatus();
  const bool resend = req_sent_t_ < 0.0 ||
    (rs == RequestStatus::Failed && now - req_sent_t_ > 1.0) ||
    (rs == RequestStatus::Pending && now - req_sent_t_ > p_.request_retry);
  if (resend) {
    vehicle_.requestTakeoff(takeoff_z_);
    markRequestSent(now);
  }
}

void MissionExecutor::beginSegment(const VehicleState & st)
{
  const auto & path = segments_[wp_idx_];
  seg_t0_ = now_;
  last_sp_ = hold_pos_;

  if (path.size() < 2) {
    path_idx_ = path.size() - 1;
    enter(State::ARRIVE, "segment ma jediny bod");
    return;
  }

  // smer prveho useku, ktory nie je cisto vertikalny
  std::optional<double> desired;
  for (std::size_t k = 1; k < path.size() && !desired; ++k) {
    desired = legYaw(k);
  }
  if (desired) {
    yaw_sp_ = unwrapNear(*desired, st.yaw);
  }
  path_idx_ = 1;

  const double err = std::abs(angleDiff(yaw_sp_, st.yaw));
  const std::string info = "-> WP" + std::to_string(wp_idx_ + 1) + " " +
    describe(mission_[wp_idx_]) + ", " + std::to_string(path.size()) + " bodov";
  if (stopped_ && err > deg2rad(p_.yaw_tol_deg)) {
    enter(State::ROTATE, info + ", otocenie o " + fmt("%.0f", rad2deg(err)) + " deg");
  } else {
    enter(State::NAVIGATE, info);
  }
}

std::optional<double> MissionExecutor::legYaw(std::size_t to_idx) const
{
  const auto & path = segments_[wp_idx_];
  if (to_idx == 0 || to_idx >= path.size()) {
    return std::nullopt;
  }
  const Vec3 & a = path[to_idx - 1];
  const Vec3 & b = path[to_idx];
  if (distXY(a, b) < p_.min_leg_xy) {
    return std::nullopt;  // vertikalny usek: atan2 je nestabilny, yaw nemenime
  }
  return segmentYaw(a, b);
}

void MissionExecutor::onRotate(const VehicleState & st, double now)
{
  send(hold_pos_, yaw_sp_);
  const double err = std::abs(angleDiff(yaw_sp_, st.yaw));
  if (err < deg2rad(p_.yaw_tol_deg)) {
    if (!cond_t0_) {
      cond_t0_ = now;
    }
    if (now - *cond_t0_ >= p_.rotate_settle) {
      enter(State::NAVIGATE, "natoceny v smere letu");
    }
  } else {
    cond_t0_.reset();
  }
  if (state_ == State::ROTATE && inState(now) > p_.rotate_timeout) {
    log(LogLevel::Warn, "otocenie nedokoncene (chyba " + fmt("%.1f", rad2deg(err)) +
      " deg), pokracujem");
    enter(State::NAVIGATE, "timeout otocenia");
  }
}

void MissionExecutor::onNavigate(const VehicleState & st, double now)
{
  const auto & path = segments_[wp_idx_];
  const std::size_t last = path.size() - 1;
  if (path_idx_ >= last) {
    enter(State::ARRIVE, "posledny bod segmentu = waypoint misie");
    return;
  }
  const Vec3 & target = path[path_idx_];
  if (const auto ly = legYaw(path_idx_)) {
    yaw_sp_ = unwrapNear(*ly, st.yaw);
  }
  send(target, yaw_sp_);

  // medzibod je pass-through: ziadne zastavenie, len priblizenie
  if (dist(st.pos, target) < p_.tol_pass) {
    ++path_idx_;
    log(LogLevel::Info, "  medzibod " + std::to_string(path_idx_ - 1) + "/" +
      std::to_string(last - 1) + " " + pt(target) + " preleteny");
  }
  if (now - seg_t0_ > p_.segment_timeout) {
    abort("segment trva dlhsie ako " + fmt("%.0f", p_.segment_timeout) + " s");
  }
}

void MissionExecutor::onArrive(const VehicleState & st, double now)
{
  const auto & path = segments_[wp_idx_];
  const Vec3 target = path.back();
  if (const auto ly = legYaw(path.size() - 1)) {
    yaw_sp_ = unwrapNear(*ly, st.yaw);
  }
  send(target, yaw_sp_);

  const double d = dist(st.pos, target);
  const double v = norm(st.vel);
  const auto & wp = mission_[wp_idx_];

  if (wp.precision == Precision::Soft) {
    if (d < p_.tol_soft) {
      // po vzlete dron stoji -> dalsi segment zacne otocenim na mieste
      stopped_ = wp.task == TaskType::Takeoff;
      waypointReached(st, now);
      return;
    }
  } else {
    if (d < p_.tol_hard && v < p_.v_settle) {
      if (!cond_t0_) {
        cond_t0_ = now;
      }
      if (now - *cond_t0_ >= p_.settle_time) {
        stopped_ = true;
        waypointReached(st, now);
        return;
      }
    } else {
      cond_t0_.reset();
    }
  }
  if (now - seg_t0_ > p_.segment_timeout) {
    abort("waypoint nedosiahnuty, chyba " + fmt("%.2f", d) + " m");
  }
}

void MissionExecutor::waypointReached(const VehicleState & st, double now)
{
  const auto & wp = mission_[wp_idx_];
  const double err = dist(st.pos, wp.p);
  arrivals_.push_back({wp_idx_, err, now - seg_t0_});
  hold_pos_ = wp.p;
  last_sp_ = wp.p;

  const std::string why = "WP" + std::to_string(wp_idx_ + 1) + " dosiahnuty, chyba " +
    fmt("%.3f", err) + " m, v=" + fmt("%.2f", norm(st.vel)) + " m/s";
  switch (wp.task) {
    case TaskType::None:
    case TaskType::Takeoff:
      enter(State::NEXT_WAYPOINT, why);
      break;
    case TaskType::Yaw:
      yaw_sp_ = unwrapNear(deg2rad(wp.yaw_deg + p_.yaw_task_offset_deg), st.yaw);
      stopped_ = true;
      enter(State::TASK_YAW, why + ", task yaw" + fmt("%g", wp.yaw_deg));
      break;
    case TaskType::Land:
      log(LogLevel::Info, why);
      startLanding(AfterLand::Done);
      break;
    case TaskType::LandTakeoff:
      log(LogLevel::Info, why);
      startLanding(AfterLand::GroundWait);
      break;
  }
}

void MissionExecutor::onTaskYaw(const VehicleState & st, double now)
{
  send(hold_pos_, yaw_sp_);
  const double err = std::abs(angleDiff(yaw_sp_, st.yaw));
  if (err < deg2rad(p_.yaw_tol_deg)) {
    if (!cond_t0_) {
      cond_t0_ = now;
    }
    if (now - *cond_t0_ >= p_.yaw_hold_time) {
      enter(State::NEXT_WAYPOINT, "yaw " + fmt("%.1f", rad2deg(wrapPi(st.yaw))) +
        " deg drzany " + fmt("%.1f", p_.yaw_hold_time) + " s");
    }
  } else {
    cond_t0_.reset();
  }
  if (state_ == State::TASK_YAW && inState(now) > p_.yaw_task_timeout) {
    log(LogLevel::Warn, "yaw task nedosiahnuty (chyba " + fmt("%.1f", rad2deg(err)) +
      " deg), pokracujem");
    enter(State::NEXT_WAYPOINT, "timeout yaw tasku");
  }
}

void MissionExecutor::startLanding(AfterLand after)
{
  after_land_ = after;
  enter(State::LAND, after == AfterLand::GroundWait ? "landtakeoff: pristatie" : "pristatie");
}

void MissionExecutor::onLand(const VehicleState & st, double now)
{
  if (!st.armed) {
    if (after_land_ == AfterLand::GroundWait) {
      enter(State::GROUND_WAIT, "na zemi a disarmnuty, cakam " +
        fmt("%.1f", p_.landtakeoff_wait) + " s");
    } else {
      enter(State::DONE, "na zemi a disarmnuty");
    }
    return;
  }
  if (inState(now) > p_.land_timeout) {
    abort("pristatie trva dlhsie ako " + fmt("%.0f", p_.land_timeout) + " s");
    return;
  }
  if (st.mode != "LAND" && requestDue(now)) {
    vehicle_.requestLand();
    markRequestSent(now);
  }
}

void MissionExecutor::onGroundWait(double now)
{
  if (inState(now) >= p_.landtakeoff_wait) {
    startTakeoffSequence(mission_[wp_idx_].p.z, true);
  }
}

void MissionExecutor::startTakeoffSequence(double z_map, bool resume_next)
{
  takeoff_z_ = z_map;
  resume_next_after_takeoff_ = resume_next;
  enter(State::SET_MODE, "vzletova sekvencia na z=" + fmt("%.2f", z_map));
}

void MissionExecutor::onNextWaypoint(const VehicleState & st)
{
  ++wp_idx_;
  if (wp_idx_ >= mission_.size()) {
    wp_idx_ = mission_.size() - 1;
    log(LogLevel::Warn, "posledny waypoint nema task land -> pristavam");
    startLanding(AfterLand::Done);
    return;
  }
  beginSegment(st);
}

void MissionExecutor::onAbort(const VehicleState & st, double now)
{
  // vo vzduchu: prepnut do LAND (indoor je RTL nebezpecne, stupa do vysky)
  if (st.armed && st.mode != "LAND" && requestDue(now) && req_attempts_ < 5) {
    vehicle_.requestMode("LAND");
    markRequestSent(now);
  }
}

void MissionExecutor::logSummary()
{
  std::ostringstream s;
  s << "SUHRN MISIE: cas " << fmt("%.1f", now_ - mission_t0_) << " s\n"
    << "  WP  precision  task          chyba[m]  segment[s]\n";
  for (const auto & a : arrivals_) {
    const auto & wp = mission_[a.wp];
    char b[128];
    std::snprintf(b, sizeof(b), "  %-3zu %-10s %-13s %8.3f  %9.1f\n", a.wp + 1,
      toString(wp.precision).c_str(), toString(wp.task).c_str(), a.err, a.seg_time);
    s << b;
  }
  log(LogLevel::Info, s.str());
}

}  // namespace lrs_control
