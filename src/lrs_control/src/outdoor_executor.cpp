#include "lrs_control/outdoor_executor.hpp"

#include <cmath>
#include <cstdio>
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

OutdoorExecutor::OutdoorExecutor(Vehicle & vehicle, OutdoorParams params, Logger log)
: vehicle_(vehicle), p_(params), logger_(std::move(log))
{
}

const char * OutdoorExecutor::name(State s)
{
  switch (s) {
    case State::WAIT_CONNECTION: return "WAIT_CONNECTION";
    case State::WAIT_GPS: return "WAIT_GPS";
    case State::WAIT_ORIGIN: return "WAIT_ORIGIN";
    case State::SET_MODE: return "SET_MODE";
    case State::ARM: return "ARM";
    case State::TAKEOFF: return "TAKEOFF";
    case State::HOLD: return "HOLD";
    case State::LAND: return "LAND";
    case State::DONE: return "DONE";
    case State::ABORT: return "ABORT";
  }
  return "?";
}

// ---------------------------------------------------------------------------
// hlavna slucka
// ---------------------------------------------------------------------------
void OutdoorExecutor::tick(double now)
{
  now_ = now;
  if (mission_t0_ < 0.0) {
    mission_t0_ = now;
    state_t0_ = now;
    log(LogLevel::Info, std::string("start v stave ") + name(state_));
  }
  sent_this_tick_ = false;
  const VehicleState st = vehicle_.state();

  // Pilot prevzal riadenie (iny mod z RC) alebo sa dron disarmol vo vzduchu:
  // NEBOJOVAT s nim -> len prestat posielat setpointy, ziadny prikaz na mod.
  if (airborneState() && (st.mode != "GUIDED" || !st.armed)) {
    doAbort("mod zmeneny zvonka na '" + st.mode + "', armed=" + (st.armed ? "1" : "0") +
      " -> riadenie odovzdane pilotovi", false);
  }
  if (airborneState() && !st.connected) {
    doAbort("MAVROS stratil spojenie s dronom", false);
  }

  switch (state_) {
    case State::WAIT_CONNECTION: onWaitConnection(st); break;
    case State::WAIT_GPS: onWaitGps(st, now); break;
    case State::WAIT_ORIGIN: onWaitOrigin(st, now); break;
    case State::SET_MODE: onSetMode(st, now); break;
    case State::ARM: onArm(st, now); break;
    case State::TAKEOFF: onTakeoff(st, now); break;
    case State::HOLD: onHold(st, now); break;
    case State::LAND: onLand(st, now); break;
    case State::DONE: break;
    case State::ABORT: onAbort(st, now); break;
  }

  // vo vzduchu (riadenom programom) ide setpoint kazdy tick
  if (airborneState() && !sent_this_tick_) {
    send(last_sp_, yaw_sp_);
  }
}

void OutdoorExecutor::enter(State s, const std::string & why)
{
  char head[64];
  std::snprintf(head, sizeof(head), "[t=%6.2fs] %s -> %s", now_ - mission_t0_, name(state_), name(s));
  log(s == State::ABORT ? LogLevel::Error : LogLevel::Info, std::string(head) + "  (" + why + ")");
  state_ = s;
  state_t0_ = now_;
  req_sent_t_ = -1.0;
  req_attempts_ = 0;
  cond_t0_.reset();
  last_wait_log_ = now_;
}

void OutdoorExecutor::abort(const std::string & why)
{
  doAbort(why, true);
}

void OutdoorExecutor::doAbort(const std::string & why, bool land)
{
  if (state_ == State::ABORT || state_ == State::DONE) {
    return;
  }
  land_on_abort_ = land;
  enter(State::ABORT, why + (land ? " -> LAND" : ""));
}

// Stavy, v ktorych program riadi dron vo vzduchu a posiela setpointy.
// TAKEOFF sem nepatri: pocas vzletu ArduPilot ide na ciel z cmd/takeoff.
bool OutdoorExecutor::airborneState() const
{
  return state_ == State::HOLD;
}

void OutdoorExecutor::send(const Vec3 & p, double yaw)
{
  vehicle_.sendPositionTarget(p, yaw);
  last_sp_ = p;
  sent_this_tick_ = true;
}

double OutdoorExecutor::heightAboveLaunch(const VehicleState & st) const
{
  // rel_alt = vyska nad home podla ArduPilota; local z - launch z = to iste z EKF.
  // Riadi sa local z (tak posielame setpointy), rel_alt sa vypisuje na kontrolu.
  return st.pos.z - launch_.z;
}

bool OutdoorExecutor::requestDue(double now) const
{
  if (req_sent_t_ < 0.0) {
    return true;
  }
  const double since = now - req_sent_t_;
  return (vehicle_.requestStatus() == RequestStatus::Failed && since > 1.0) || since > p_.request_retry;
}

void OutdoorExecutor::markRequestSent(double now)
{
  if (req_attempts_ > 0) {
    log(LogLevel::Warn, std::string(name(state_)) + ": opakujem poziadavku (pokus " +
      std::to_string(req_attempts_ + 1) + ")");
  }
  req_sent_t_ = now;
  ++req_attempts_;
}

void OutdoorExecutor::log(LogLevel l, const std::string & msg)
{
  if (logger_) {
    logger_(l, msg);
  }
}

// ---------------------------------------------------------------------------
// stavy
// ---------------------------------------------------------------------------
void OutdoorExecutor::onWaitConnection(const VehicleState & st)
{
  if (st.armed) {
    doAbort("dron je uz armnuty, misiu spusti s dronom na zemi", false);
    return;
  }
  if (st.connected && st.has_pose) {
    enter(State::WAIT_GPS, "MAVROS connected");
  }
}

void OutdoorExecutor::onWaitGps(const VehicleState & st, double now)
{
  std::string why;
  if (gpsReady(st, p_.min_satellites, p_.max_h_acc, why)) {
    if (!cond_t0_) {
      cond_t0_ = now;
    }
    if (now - *cond_t0_ >= p_.gps_settle_time) {
      enter(State::WAIT_ORIGIN, "GPS OK " + fmt("%.0f", p_.gps_settle_time) + " s: status " +
        std::to_string(st.gps_status) + ", presnost " +
        (st.h_acc >= 0 ? fmt("%.2f m", st.h_acc) : std::string("?")) + ", satelity " +
        (st.satellites >= 0 ? std::to_string(st.satellites) : std::string("?")));
    }
    return;
  }
  cond_t0_.reset();
  if (now - last_wait_log_ >= 5.0) {
    log(LogLevel::Warn, "cakam na GPS, NEARMUJEM: " + why);
    last_wait_log_ = now;
  }
}

void OutdoorExecutor::onWaitOrigin(const VehicleState & st, double now)
{
  LatLonAlt origin;
  if (st.has_home) {
    origin = st.home;
    origin_src_ = "home z /mavros/home_position/home";
  } else if (inState(now) >= p_.home_wait) {
    origin = st.gps;
    origin_src_ = "GPS fix na mieste vzletu (home neprisiel za " + fmt("%.0f", p_.home_wait) + " s)";
  } else {
    return;
  }
  ltp_ = LocalTangentPlane(origin);
  launch_ = st.pos;
  yaw_sp_ = st.yaw;

  char b[200];
  std::snprintf(b, sizeof(b), "pociatok lat %.7f lon %.7f (%s)", origin.lat_deg, origin.lon_deg,
    origin_src_.c_str());
  log(LogLevel::Info, b);
  // kontrola: dron by mal stat blizko pociatku
  const Vec3 gps_enu = ltp_.toEnu(st.gps);
  std::snprintf(b, sizeof(b), "miesto vzletu: local %s, z GPS E %.2f N %.2f, kompas %.1f deg",
    pt(launch_).c_str(), gps_enu.x, gps_enu.y, st.compass_hdg_deg);
  log(LogLevel::Info, b);
  if (std::hypot(gps_enu.x, gps_enu.y) > 5.0) {
    log(LogLevel::Warn, "dron je " + fmt("%.1f", std::hypot(gps_enu.x, gps_enu.y)) +
      " m od pociatku (home nastaveny inde?)");
  }
  enter(State::SET_MODE, "vzlet na " + fmt("%.1f", p_.altitude) + " m nad miestom vzletu");
}

void OutdoorExecutor::onSetMode(const VehicleState & st, double now)
{
  if (st.mode == "GUIDED") {
    enter(State::ARM, "mod GUIDED potvrdeny");
    return;
  }
  if (inState(now) > p_.mode_timeout) {
    doAbort("GUIDED sa nepodarilo nastavit", false);
    return;
  }
  if (requestDue(now)) {
    vehicle_.requestMode("GUIDED");
    markRequestSent(now);
  }
}

void OutdoorExecutor::onArm(const VehicleState & st, double now)
{
  if (st.armed) {
    enter(State::TAKEOFF, "armed potvrdeny");
    return;
  }
  if (st.mode != "GUIDED") {
    enter(State::SET_MODE, "mod sa zmenil na '" + st.mode + "'");
    return;
  }
  if (inState(now) > p_.arm_timeout) {
    doAbort("arm sa nepodaril (prearm checky v konzole SITL)", false);
    return;
  }
  if (requestDue(now)) {
    vehicle_.requestArm(true);
    markRequestSent(now);
  }
}

void OutdoorExecutor::onTakeoff(const VehicleState & st, double now)
{
  const double h = heightAboveLaunch(st);
  if (h >= p_.altitude - p_.takeoff_tol) {
    hold_target_ = {launch_.x, launch_.y, launch_.z + p_.altitude};
    last_sp_ = hold_target_;
    yaw_sp_ = st.yaw;
    enter(State::HOLD, "vyska dosiahnuta: local z - launch " + fmt("%.2f m", h) +
      ", rel_alt " + (st.has_rel_alt ? fmt("%.2f m", st.rel_alt) : std::string("?")));
    return;
  }
  if (st.mode != "GUIDED" && inState(now) > 1.0) {
    doAbort("pocas vzletu sa mod zmenil na '" + st.mode + "'", false);
    return;
  }
  if (!st.armed && inState(now) > 2.0) {
    enter(State::SET_MODE, "dron sa disarmol pred vzletom, opakujem arm");
    return;
  }
  if (inState(now) > p_.takeoff_timeout) {
    doAbort("vzlet nedosiahol " + fmt("%.1f m", p_.altitude) + " (je " + fmt("%.2f m", h) + ")", true);
    return;
  }
  const auto rs = vehicle_.requestStatus();
  const bool resend = req_sent_t_ < 0.0 ||
    (rs == RequestStatus::Failed && now - req_sent_t_ > 1.0) ||
    (rs == RequestStatus::Pending && now - req_sent_t_ > p_.request_retry);
  if (resend) {
    vehicle_.requestTakeoff(launch_.z + p_.altitude);
    markRequestSent(now);
  }
}

void OutdoorExecutor::onHold(const VehicleState & st, double now)
{
  send(hold_target_, yaw_sp_);
  const double err = dist(st.pos, hold_target_);
  const double v = norm(st.vel);
  if (err < p_.hold_tol && v < p_.hold_v) {
    if (!cond_t0_) {
      cond_t0_ = now;
    }
    if (now - *cond_t0_ >= p_.hold_time) {
      // cast 2: po stabilnom drzani pristat. V casti 3 sem pride stvorec s POI.
      enter(State::LAND, "stabilne " + fmt("%.0f s", p_.hold_time) + ", chyba " +
        fmt("%.2f m", err) + ", vyska " + fmt("%.2f m", heightAboveLaunch(st)));
      return;
    }
  } else {
    cond_t0_.reset();
  }
  if (inState(now) > p_.hold_timeout) {
    doAbort("dron sa neustalil za " + fmt("%.0f s", p_.hold_timeout) + " (chyba " +
      fmt("%.2f m", err) + ")", true);
  }
}

void OutdoorExecutor::onLand(const VehicleState & st, double now)
{
  if (!st.armed) {
    const Vec3 d{st.pos.x - launch_.x, st.pos.y - launch_.y, 0.0};
    enter(State::DONE, "na zemi a disarmnuty, " + fmt("%.2f m", norm(d)) + " od miesta vzletu");
    return;
  }
  if (inState(now) > p_.land_timeout) {
    doAbort("pristatie trva dlhsie ako " + fmt("%.0f s", p_.land_timeout), false);
    return;
  }
  // GUIDED = este sme neprepli -> poziadat o LAND; LAND = cakat na disarm;
  // iny mod = pilot prevzal riadenie -> uz nic neposielat
  if (st.mode == "GUIDED") {
    if (requestDue(now)) {
      vehicle_.requestLand();
      markRequestSent(now);
    }
  } else if (st.mode != "LAND") {
    doAbort("pocas pristatia mod zmeneny na '" + st.mode + "' -> riadenie odovzdane pilotovi",
      false);
  }
}

void OutdoorExecutor::onAbort(const VehicleState & st, double now)
{
  // land_on_abort_ = false: pilot prevzal riadenie, NIC neposielame.
  // Ked je LAND raz potvrdeny, dalsie zmeny modu su pilotove -> uz neposielame nic.
  if (st.mode == "LAND") {
    land_on_abort_ = false;
  }
  if (land_on_abort_ && st.armed && requestDue(now) && req_attempts_ < 5) {
    vehicle_.requestMode("LAND");
    markRequestSent(now);
  }
}

}  // namespace lrs_control
