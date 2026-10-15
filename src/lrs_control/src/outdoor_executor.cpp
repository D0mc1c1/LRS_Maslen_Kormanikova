#include "lrs_control/outdoor_executor.hpp"

#include <algorithm>
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
// nazov ciela v zozname square_ (W1, W2, W3, W4, W1)
std::string wpName(std::size_t i)
{
  return "W" + std::to_string(i % 4 + 1);
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
    case State::POI_ALIGN: return "POI_ALIGN";
    case State::SQUARE: return "SQUARE";
    case State::RETURN: return "RETURN";
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
    case State::POI_ALIGN: onPoiAlign(st, now); break;
    case State::SQUARE: onSquare(st, now); break;
    case State::RETURN: onReturn(st, now); break;
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
  return state_ == State::HOLD || state_ == State::POI_ALIGN || state_ == State::SQUARE ||
         state_ == State::RETURN;
}

void OutdoorExecutor::send(const Vec3 & p, double yaw)
{
  vehicle_.sendPositionTarget(p, yaw);
  last_sp_ = p;
  sent_this_tick_ = true;
}

void OutdoorExecutor::sendTraj(const Vec3 & p, const Vec3 & v, double yaw, double yaw_rate)
{
  TrajectorySetpoint sp;
  sp.p = p;
  sp.yaw = yaw;
  if (p_.velocity_ff) {sp.v = v;}
  if (p_.yaw_rate_ff) {sp.yaw_rate = yaw_rate;}
  vehicle_.sendTrajectorySetpoint(sp);
  last_sp_ = p;
  yaw_sp_ = yaw;
  sent_this_tick_ = true;
}

double OutdoorExecutor::poiYaw(const Vec3 & p, double fallback) const
{
  return distXY(p, poi_) < p_.poi_min_dist ? fallback : bearingXY(p, poi_);
}

// psi = atan2(dy, dx), d = POI - p, d' = -v  =>  psi' = (dy*vx - dx*vy) / |d|^2
double OutdoorExecutor::poiYawRate(const Vec3 & p, const Vec3 & v) const
{
  const double dx = poi_.x - p.x, dy = poi_.y - p.y;
  const double r2 = dx * dx + dy * dy;
  if (r2 < p_.poi_min_dist * p_.poi_min_dist) {
    return 0.0;
  }
  return (dy * v.x - dx * v.y) / r2;
}

Vec3 OutdoorExecutor::llaToLocal(const LatLonAlt & g) const
{
  const Vec3 e = ltp_.toEnu(g);
  return {e.x + geo_offset_.x, e.y + geo_offset_.y, e.z + geo_offset_.z};
}

LatLonAlt OutdoorExecutor::localToLla(const Vec3 & p) const
{
  return ltp_.toLla({p.x - geo_offset_.x, p.y - geo_offset_.y, p.z - geo_offset_.z});
}

// POI a rohy stvorca. Volane raz vo WAIT_ORIGIN, ked je znamy pociatok aj miesto vzletu.
bool OutdoorExecutor::setupMissionGeometry(const VehicleState & st)
{
  // MAVROS local (EKF origin) a ENU od home (ltp_) su rovnobezne ramce, mozu byt posunute
  const Vec3 gps_enu = ltp_.toEnu(st.gps);
  geo_offset_ = {launch_.x - gps_enu.x, launch_.y - gps_enu.y, launch_.z - gps_enu.z};

  char b[200];
  if (p_.poi_use_latlon) {
    poi_ = llaToLocal({p_.poi_lat, p_.poi_lon, st.gps.alt_m});
  } else {
    poi_ = {launch_.x + p_.poi_east, launch_.y + p_.poi_north, launch_.z};
  }
  poi_.z = launch_.z;   // POI je na zemi
  const LatLonAlt poi_g = localToLla(poi_);
  std::snprintf(b, sizeof(b), "POI lat %.7f lon %.7f = od miesta vzletu E %.2f N %.2f (%s)",
    poi_g.lat_deg, poi_g.lon_deg, poi_.x - launch_.x, poi_.y - launch_.y,
    p_.poi_use_latlon ? "parameter lat/lon" : "parameter posun");
  log(LogLevel::Info, b);

  // Rohy: posun +-S/2 od POI -> lat/lon (tak ako by ich zadal clovek) -> local ENU.
  // Pouzivaju sa oba smery prevodu; vysledok je ten isty ako priamy posun v ENU.
  const double h = p_.square_side / 2.0, z = launch_.z + p_.altitude;
  const double off[4][2] = {{-h, h}, {h, h}, {h, -h}, {-h, -h}};   // W1 SZ, W2 SV, W3 JV, W4 JZ
  square_.clear();
  for (int k = 0; k < 4; ++k) {
    const LatLonAlt g = localToLla({poi_.x + off[k][0], poi_.y + off[k][1], z});
    Vec3 w = llaToLocal(g);
    w.z = z;
    square_.push_back(w);
    std::snprintf(b, sizeof(b), "W%d lat %.7f lon %.7f -> od miesta vzletu E %.2f N %.2f, %.1f m od POI",
      k + 1, g.lat_deg, g.lon_deg, w.x - launch_.x, w.y - launch_.y, distXY(w, poi_));
    log(LogLevel::Info, b);
  }
  square_.push_back(square_[0]);   // uzavriet stvorec

  double r_max = 0.0;
  for (const auto & w : square_) {
    r_max = std::max(r_max, distXY(w, launch_));
  }
  std::snprintf(b, sizeof(b), "geofence: misia max. %.2f m od miesta vzletu (limit %.1f m), "
    "vyska %.1f m", r_max, p_.geofence_radius, p_.altitude);
  log(r_max > p_.geofence_radius ? LogLevel::Error : LogLevel::Info, b);
  return r_max <= p_.geofence_radius;
}

void OutdoorExecutor::startLeg(std::size_t i, const Vec3 & from)
{
  leg_i_ = i;
  leg_ = StraightLeg(from, square_[i], p_.speed, p_.accel);
  leg_t0_ = now_;
  leg_err_max_ = 0.0;
  char b[160];
  std::snprintf(b, sizeof(b), "SQUARE: %s %s (%.2f m, %.1f s)%s",
    i == 0 ? "presun na" : (wpName(i - 1) + " ->").c_str(), wpName(i).c_str(),
    leg_.length(), leg_.duration(), i == 0 ? ", nos na POI" : "");
  log(LogLevel::Info, b);
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
  poi_err_deg_ = -1.0;

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
  if (!setupMissionGeometry(st)) {
    doAbort("misia je mimo geofence, NEVZLIETAM (skontroluj POI)", false);
    return;
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
      enter(State::POI_ALIGN, "stabilne " + fmt("%.0f s", p_.hold_time) + ", chyba " +
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

void OutdoorExecutor::onPoiAlign(const VehicleState & st, double now)
{
  // smer na POI z bodu drzania; ak je dron priamo nad POI, natoci sa tak,
  // ako to bude treba vo W1
  const double target = poiYaw(hold_target_, poiYaw(square_[0], yaw_sp_));
  sendTraj(hold_target_, {}, unwrapNear(target, yaw_sp_), 0.0);
  const double err = std::abs(rad2deg(angleDiff(target, st.yaw)));
  if (err < p_.align_tol_deg) {
    if (!cond_t0_) {
      cond_t0_ = now;
    }
    if (now - *cond_t0_ >= p_.align_time) {
      enter(State::SQUARE, "nos na POI, chyba " + fmt("%.1f deg", err));
      startLeg(0, hold_target_);
      return;
    }
  } else {
    cond_t0_.reset();
  }
  if (inState(now) > p_.hold_timeout) {
    doAbort("natocenie na POI sa nepodarilo (chyba " + fmt("%.1f deg)", err), true);
  }
}

void OutdoorExecutor::onSquare(const VehicleState & st, double now)
{
  const double t = now - leg_t0_;
  const Vec3 p = leg_.pos(t);
  const Vec3 v = leg_.vel(t);

  // yaw na POI sa pocita KAZDY tick (nie raz na roh)
  const Vec3 & src = p_.poi_yaw_from_setpoint ? p : st.pos;
  const double yaw = unwrapNear(poiYaw(src, yaw_sp_), yaw_sp_);
  sendTraj(p, v, yaw, poiYawRate(src, v));

  // chyba natocenia: nos drona vs. smer zo skutocnej polohy na POI
  poi_err_deg_ = -1.0;
  if (distXY(st.pos, poi_) >= p_.poi_min_dist) {
    poi_err_deg_ = std::abs(rad2deg(angleDiff(bearingXY(st.pos, poi_), st.yaw)));
    if (leg_i_ >= 1) {   // hodnoti sa stvorec, nie presun na W1
      leg_err_max_ = std::max(leg_err_max_, poi_err_deg_);
      poi_stats_.max_deg = std::max(poi_stats_.max_deg, poi_err_deg_);
      poi_stats_.sum_sq += poi_err_deg_ * poi_err_deg_;
      ++poi_stats_.n;
    }
  }

  const Vec3 goal = square_[leg_i_];
  if (t >= leg_.duration() && dist(st.pos, goal) < p_.corner_tol) {
    ++corners_reached_;
    const LatLonAlt g = localToLla(st.pos);
    char b[220];
    std::snprintf(b, sizeof(b),
      "%s dosiahnuty: lat %.7f lon %.7f, od vzletu E %.2f N %.2f U %.2f, chyba %.2f m",
      wpName(leg_i_).c_str(), g.lat_deg, g.lon_deg, st.pos.x - launch_.x, st.pos.y - launch_.y,
      st.pos.z - launch_.z, dist(st.pos, goal));
    std::string msg = b;
    if (leg_i_ >= 1) {
      msg += ", yaw na POI max " + fmt("%.1f deg", leg_err_max_);
    }
    log(LogLevel::Info, msg);
    if (leg_i_ + 1 < square_.size()) {
      startLeg(leg_i_ + 1, goal);
    } else {
      std::snprintf(b, sizeof(b), "stvorec hotovy: chyba natocenia na POI max %.1f deg, "
        "RMS %.1f deg (%ld vzoriek)", poi_stats_.max_deg, poi_stats_.rmsDeg(), poi_stats_.n);
      log(LogLevel::Info, b);
      enter(State::RETURN, "navrat nad miesto vzletu");
      leg_ = StraightLeg(goal, hold_target_, p_.speed, p_.accel);
      leg_t0_ = now;
    }
    return;
  }
  if (t > leg_.duration() + p_.leg_timeout_extra) {
    doAbort(wpName(leg_i_) + " nedosiahnuty (chyba " + fmt("%.2f m)", dist(st.pos, goal)), true);
  }
}

void OutdoorExecutor::onReturn(const VehicleState & st, double now)
{
  const double t = now - leg_t0_;
  sendTraj(leg_.pos(t), leg_.vel(t), yaw_sp_, 0.0);
  const double err = dist(st.pos, leg_.b);
  if (t >= leg_.duration() && err < p_.hold_tol && norm(st.vel) < p_.hold_v) {
    if (!cond_t0_) {
      cond_t0_ = now;
    }
    if (now - *cond_t0_ >= 2.0) {
      enter(State::LAND, "nad miestom vzletu, chyba " + fmt("%.2f m", err));
      return;
    }
  } else {
    cond_t0_.reset();
  }
  if (t > leg_.duration() + p_.leg_timeout_extra) {
    doAbort("navrat nad miesto vzletu sa nepodaril (chyba " + fmt("%.2f m)", err), true);
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
