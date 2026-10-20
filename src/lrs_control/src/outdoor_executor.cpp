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
}

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
    case State::FIG8_ENTRY: return "FIG8_ENTRY";
    case State::FIG8_ALIGN: return "FIG8_ALIGN";
    case State::FIG8: return "FIG8";
    case State::RETURN: return "RETURN";
    case State::LAND: return "LAND";
    case State::DONE: return "DONE";
    case State::ABORT: return "ABORT";
  }
  return "?";
}


// Hlavna slucka misie, ktora spracuje aktualny stav dronu a riadi prechod medzi stavmi. (20hz)
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

  // Spojenie stratene (MAVROS spadol alebo FCU odpojene): prikazy by aj tak nedosli
  // -> prestat riadit. Dron drzi poslednu poziciu v GUIDED, dalej rozhoduje pilot
  // a failsafe ArduPilota.
  if (controlState()) {
    std::string why;
    if (!linkOk(st, p_.state_timeout, p_.pose_timeout, why)) {
      doAbort("spojenie stratene: " + why, false);
    }
  }
  // Pilot prevzal riadenie (iny mod z RC) alebo sa dron disarmol vo vzduchu:
  // NEBOJOVAT s nim -> len prestat posielat setpointy, ziadny prikaz na mod.
  if (airborneState() && (st.mode != "GUIDED" || !st.armed)) {
    doAbort("mod zmeneny zvonka na '" + st.mode + "', armed=" + (st.armed ? "1" : "0"), false);
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
    case State::FIG8_ENTRY: onFig8Entry(st, now); break;
    case State::FIG8_ALIGN: onFig8Align(st, now); break;
    case State::FIG8: onFig8(st, now); break;
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
// Vrati nazov aktualneho stavu misie.
std::string OutdoorExecutor::phase() const
{
  std::string s = name(state_);
  if (state_ == State::SQUARE) {
    s += ":" + std::to_string(leg_i_);
  }
  return s;
}

// Vytvori zoznam bodov planovanej trajektorie.
std::vector<Vec3> OutdoorExecutor::plannedPath() const
{
  std::vector<Vec3> path;
  if (!ltp_.valid() || square_.empty()) {
    return path;
  }
  const Vec3 above{launch_.x, launch_.y, launch_.z + p_.altitude};
  path.push_back(launch_);
  path.push_back(above);
  for (const auto & w : square_) {
    path.push_back(w);
  }
  const int n = static_cast<int>(std::ceil(p_.fig8_laps * 360.0));
  for (int k = 0; k <= n; ++k) {
    path.push_back(fig8_.posAt(deg2rad(k)));
  }
  path.push_back(above);
  path.push_back(launch_);
  return path;
}
// Prepne misiu do noveho stavu a zapise zmenu do logu.
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
  poi_err_deg_ = -1.0;
  track_err_ = -1.0;
}

// Prerusi misiu a nastavi sposob dalsieho postupu.
void OutdoorExecutor::abort(const std::string & why)
{
  doAbort(why, true);
}
// Spracuje prerusenie misie s moznostou vydat bezpecnostny prikaz.
void OutdoorExecutor::doAbort(const std::string & why, bool command)
{
  if (state_ == State::ABORT || state_ == State::DONE) {
    return;
  }
  command_on_abort_ = command;
  enter(State::ABORT, why + (command ? " -> " + p_.abort_mode :
    std::string(" -> prestavam riadit, nic neprikazujem")));
}
// Urci, ci sa ma kontrolovat spojenie pocas aktivneho riadenia.
bool OutdoorExecutor::controlState() const
{
  return state_ == State::SET_MODE || state_ == State::ARM || state_ == State::TAKEOFF ||
         state_ == State::LAND || airborneState();
}

// Stavy, v ktorych program riadi dron vo vzduchu a posiela setpointy.
// TAKEOFF sem nepatri: pocas vzletu ArduPilot ide na ciel z cmd/takeoff.
// Urci, ci sa dron nachadza v stave aktivneho riadenia pocas letu.
bool OutdoorExecutor::airborneState() const
{
  return state_ == State::HOLD || state_ == State::POI_ALIGN || state_ == State::SQUARE ||
         state_ == State::FIG8_ENTRY || state_ == State::FIG8_ALIGN || state_ == State::FIG8 ||
         state_ == State::RETURN;
}
// Odošle cielovu polohu a natocenie dronu.
void OutdoorExecutor::send(const Vec3 & p, double yaw)
{
  vehicle_.sendPositionTarget(p, yaw);
  last_sp_ = p;
  sent_this_tick_ = true;
}
// Odošle cielovu polohu, rychlost a udaje o natacani.
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
// Vypocita smer natocenia dronu smerom k bodu POI.
double OutdoorExecutor::poiYaw(const Vec3 & p, double fallback) const
{
  return distXY(p, poi_) < p_.poi_min_dist ? fallback : bearingXY(p, poi_);
}

// psi = atan2(dy, dx), d = POI - p, d' = -v  =>  psi' = (dy*vx - dx*vy) / |d|^2

// Vypocita uhlovu rychlost potrebnu na sledovanie POI.
double OutdoorExecutor::poiYawRate(const Vec3 & p, const Vec3 & v) const
{
  const double dx = poi_.x - p.x, dy = poi_.y - p.y;
  const double r2 = dx * dx + dy * dy;
  if (r2 < p_.poi_min_dist * p_.poi_min_dist) {
    return 0.0;
  }
  return (dy * v.x - dx * v.y) / r2;
}
// Prevedie GPS suradnice na lokalne suradnice misie.
Vec3 OutdoorExecutor::llaToLocal(const LatLonAlt & g) const
{
  const Vec3 e = ltp_.toEnu(g);
  return {e.x + geo_offset_.x, e.y + geo_offset_.y, e.z + geo_offset_.z};
}
// Prevedie lokalne suradnice misie na GPS suradnice.
LatLonAlt OutdoorExecutor::localToLla(const Vec3 & p) const
{
  return ltp_.toLla({p.x - geo_offset_.x, p.y - geo_offset_.y, p.z - geo_offset_.z});
}

// POI a rohy stvorca. Volane raz vo WAIT_ORIGIN, ked je znamy pociatok aj miesto vzletu.
// Pripravi POI, rohy stvorca a trajektoriu osmicky.
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

  // Rohy: posun +-S/2 od POI -> lat/lon -> local ENU.
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

  // osmicka so stredom nad POI
  fig8_ = Figure8({poi_.x, poi_.y, z}, p_.fig8_amplitude, p_.fig8_omega, p_.fig8_laps,
    p_.fig8_ramp, deg2rad(p_.fig8_rotation_deg));
  std::snprintf(b, sizeof(b), "osmicka: A %.1f m, omega %.3f rad/s, max rychlost A*omega*sqrt2 = "
    "%.2f m/s (limit %.1f), %.1f kola za %.1f s (rozbeh/dobeh %.1f s), otocenie %.0f deg",
    fig8_.A, fig8_.omega(), fig8_.maxSpeed(), p_.fig8_max_speed, p_.fig8_laps,
    fig8_.duration(), p_.fig8_ramp, p_.fig8_rotation_deg);
  log(fig8_.maxSpeed() > p_.fig8_max_speed ? LogLevel::Error : LogLevel::Info, b);
  if (fig8_.maxSpeed() > p_.fig8_max_speed) {
    return false;
  }

  double r_max = 0.0;
  for (const auto & w : square_) {
    r_max = std::max(r_max, distXY(w, launch_));
  }
  for (int k = 0; k < 360; ++k) {
    r_max = std::max(r_max, distXY(fig8_.posAt(deg2rad(k)), launch_));
  }
  std::snprintf(b, sizeof(b), "geofence: misia max. %.2f m od miesta vzletu (limit %.1f m), "
    "vyska %.1f m", r_max, p_.geofence_radius, p_.altitude);
  log(r_max > p_.geofence_radius ? LogLevel::Error : LogLevel::Info, b);
  return r_max <= p_.geofence_radius;
}
// Pripravi novy priamy usek letu.
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
// Vypocita vysku dronu nad miestom vzletu.
double OutdoorExecutor::heightAboveLaunch(const VehicleState & st) const
{
  // rel_alt = vyska nad home podla ArduPilota; local z - launch z = to iste z EKF.
  // Riadi sa local z (tak posielame setpointy), rel_alt sa vypisuje na kontrolu.
  return st.pos.z - launch_.z;
}
// Skontroluje, ci je mozne odoslat alebo zopakovat poziadavku.
bool OutdoorExecutor::requestDue(double now) const
{
  if (req_sent_t_ < 0.0) {
    return true;
  }
  const double since = now - req_sent_t_;
  return (vehicle_.requestStatus() == RequestStatus::Failed && since > 1.0) || since > p_.request_retry;
}
// Zaznamena odoslanie poziadavky a zvysi pocet pokusov.
void OutdoorExecutor::markRequestSent(double now)
{
  if (req_attempts_ > 0) {
    log(LogLevel::Warn, std::string(name(state_)) + ": opakujem poziadavku (pokus " +
      std::to_string(req_attempts_ + 1) + ")");
  }
  req_sent_t_ = now;
  ++req_attempts_;
}
// Zapise spravu do logu, ak je dostupny zapisovac.
void OutdoorExecutor::log(LogLevel l, const std::string & msg)
{
  if (logger_) {
    logger_(l, msg);
  }
}

// stavy
// Caka na spojenie s dronom.
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

// Caka na splnenie poziadaviek na kvalitu GPS.
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
// Nastavi pociatocny bod a pripravi geometriu misie.
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
    doAbort("misia je mimo geofence alebo prilis rychla, NEVZLIETAM (skontroluj parametre)", false);
    return;
  }
  enter(State::SET_MODE, "vzlet na " + fmt("%.1f", p_.altitude) + " m nad miestom vzletu");
}
// Nastavi rezim GUIDED.
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
// Aktivuje motory dronu.
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
// Riadi vzlet do pozadovanej vysky.
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
// Udrziava dron na cielovej pozicii.
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
// Nataca dron na pozadovany smer.
bool OutdoorExecutor::alignStep(const VehicleState & st, double now, const Vec3 & pos,
  double target, const std::string & what)
{
  sendTraj(pos, {}, unwrapNear(target, yaw_sp_), 0.0);
  const double err = std::abs(rad2deg(angleDiff(target, st.yaw)));
  if (err < p_.align_tol_deg) {
    if (!cond_t0_) {
      cond_t0_ = now;
    }
    if (now - *cond_t0_ >= p_.align_time) {
      log(LogLevel::Info, what + ": natoceny, chyba " + fmt("%.1f deg", err));
      return true;
    }
  } else {
    cond_t0_.reset();
  }
  if (inState(now) > p_.hold_timeout) {
    doAbort(what + ": natocenie sa nepodarilo (chyba " + fmt("%.1f deg)", err), true);
  }
  return false;
}

// Natoci dron smerom k bodu POI.
void OutdoorExecutor::onPoiAlign(const VehicleState & st, double now)
{
  // smer na POI z bodu drzania; ak je dron priamo nad POI, natoci sa tak,
  // ako to bude treba vo W1
  const double target = poiYaw(hold_target_, poiYaw(square_[0], yaw_sp_));
  if (alignStep(st, now, hold_target_, target, "nos na POI")) {
    enter(State::SQUARE, "zaciatok stvorca");
    startLeg(0, hold_target_);
  }
}
// Riadi let po stvorci a kontroluje presnost pohybu.
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
      startFig8Entry(goal);
    }
    return;
  }
  if (t > leg_.duration() + p_.leg_timeout_extra) {
    doAbort(wpName(leg_i_) + " nedosiahnuty (chyba " + fmt("%.2f m)", dist(st.pos, goal)), true);
  }
}
// Pripravi presun do stredu osmicky.
void OutdoorExecutor::startFig8Entry(const Vec3 & from)
{
  enter(State::FIG8_ENTRY, "presun do stredu osmicky nad POI");
  leg_ = StraightLeg(from, fig8_.c, p_.speed, p_.accel);
  leg_t0_ = now_;
  // pocas presunu sa yaw plynulo stoci na tangentu osmicky v t = 0
  entry_yaw0_ = yaw_sp_;
  entry_yaw1_ = unwrapNear(fig8_.tangentYawAt(0.0), yaw_sp_);
}
// Riadi presun do stredu osmicky.
void OutdoorExecutor::onFig8Entry(const VehicleState & st, double now)
{
  const double t = now - leg_t0_;
  const double L = leg_.length();
  const double frac = L > 1e-6 ? leg_.prof.s(t) / L : 1.0;
  const double dyaw = entry_yaw1_ - entry_yaw0_;
  sendTraj(leg_.pos(t), leg_.vel(t), entry_yaw0_ + dyaw * frac,
    L > 1e-6 ? dyaw / L * leg_.prof.v(t) : 0.0);
  const double err = dist(st.pos, fig8_.c);
  if (t >= leg_.duration() && err < p_.corner_tol) {
    enter(State::FIG8_ALIGN, "v strede osmicky, chyba " + fmt("%.2f m", err));
    return;
  }
  if (t > leg_.duration() + p_.leg_timeout_extra) {
    doAbort("stred osmicky nedosiahnuty (chyba " + fmt("%.2f m)", err), true);
  }
}
// Nastavi pociatocne natocenie pred letom po osmicke.
void OutdoorExecutor::onFig8Align(const VehicleState & st, double now)
{
  if (alignStep(st, now, fig8_.c, fig8_.tangentYawAt(0.0), "tangenta osmicky")) {
    enter(State::FIG8, "start casovej trajektorie, t = omega * (now - t_start)");
    fig8_t0_ = now;
    fig8_quarter_ = 0;
    fig8_stats_ = Fig8Stats{};
  }
}
// Riadi let po osmicke a sleduje odchylku od trajektorie.
void OutdoorExecutor::onFig8(const VehicleState & st, double now)
{
  // Novy setpoint z rovnic kazdy tick (tick = ROS timer v node, 20 Hz).
  const double t = now - fig8_t0_;
  const double th = fig8_.theta(t);
  const Vec3 p = fig8_.pos(t);
  // yaw = tangenta; unwrap -> setpoint sa nikdy neskoci o 2*pi
  const double yaw = unwrapNear(fig8_.yaw(t), yaw_sp_);
  auto & fs = fig8_stats_;
  fs.max_yaw_step_deg = std::max(fs.max_yaw_step_deg, std::abs(rad2deg(yaw - yaw_sp_)));
  sendTraj(p, fig8_.vel(t), yaw, fig8_.yawRate(t));

  // statistiky: skutocna poloha vs. setpoint, nos vs. tangenta, rozmery
  track_err_ = dist(st.pos, p);
  const double yaw_err = std::abs(rad2deg(angleDiff(yaw, st.yaw)));
  fs.track_max = std::max(fs.track_max, track_err_);
  fs.track_sum_sq += track_err_ * track_err_;
  fs.yaw_max_deg = std::max(fs.yaw_max_deg, yaw_err);
  fs.yaw_sum_sq += yaw_err * yaw_err;
  ++fs.n;
  fs.x_min = std::min(fs.x_min, st.pos.x - fig8_.c.x);
  fs.x_max = std::max(fs.x_max, st.pos.x - fig8_.c.x);
  fs.y_min = std::min(fs.y_min, st.pos.y - fig8_.c.y);
  fs.y_max = std::max(fs.y_max, st.pos.y - fig8_.c.y);

  if (track_err_ > p_.max_track_err) {
    doAbort("dron sa odchylil od osmicky o " + fmt("%.2f m", track_err_), true);
    return;
  }

  // priebeh po stvrtinach kola
  const int q = static_cast<int>(th / (kPi / 2.0) + 1e-9);
  if (q > fig8_quarter_) {
    fig8_quarter_ = q;
    char b[200];
    std::snprintf(b, sizeof(b), "FIG8 th = %.2f pi: od POI E %.2f N %.2f, rychlost %.2f m/s, "
      "odchylka %.2f m, yaw %.0f deg (ciel %.0f)", th / kPi, st.pos.x - fig8_.c.x,
      st.pos.y - fig8_.c.y, norm(st.vel), track_err_, rad2deg(wrapPi(st.yaw)),
      rad2deg(wrapPi(yaw)));
    log(LogLevel::Info, b);
  }

  if (t >= fig8_.duration()) {
    const double err = dist(st.pos, fig8_.c);
    if (err < p_.corner_tol) {
      fs.done = true;
      char b[220];
      std::snprintf(b, sizeof(b), "osmicka hotova za %.1f s: odchylka polohy max %.2f m RMS %.2f m,"
        " yaw vs tangenta max %.1f deg RMS %.1f deg, max skok yaw setpointu %.1f deg",
        t, fs.track_max, fs.trackRms(), fs.yaw_max_deg, fs.yawRmsDeg(), fs.max_yaw_step_deg);
      log(LogLevel::Info, b);
      std::snprintf(b, sizeof(b), "osmicka rozmery: E %.2f .. %.2f m, N %.2f .. %.2f m "
        "(teoria +-%.2f, +-%.2f)", fs.x_min, fs.x_max, fs.y_min, fs.y_max, fig8_.A, fig8_.A / 2.0);
      log(LogLevel::Info, b);
      startReturn(fig8_.c);
    } else if (t > fig8_.duration() + p_.leg_timeout_extra) {
      doAbort("po osmicke sa dron nevratil do stredu (chyba " + fmt("%.2f m)", err), true);
    }
  }
}

// Pripravi navrat na miesto vzletu.
void OutdoorExecutor::startReturn(const Vec3 & from)
{
  enter(State::RETURN, "navrat nad miesto vzletu");
  leg_ = StraightLeg(from, hold_target_, p_.speed, p_.accel);
  leg_t0_ = now_;
}
// Riadi navrat dronu na miesto vzletu.
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
// Riadi pristatie a kontroluje ukoncenie letu.
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
// Spracuje prerusenie misie a pripadnu zmenu rezimu.
void OutdoorExecutor::onAbort(const VehicleState & st, double now)
{
  // command_on_abort_ = false: pilot prevzal / spojenie stratene -> NIC neposielame.
  if (!command_on_abort_) {
    return;
  }
  if (!st.armed) {
    command_on_abort_ = false;
    log(LogLevel::Info, "ABORT: dron je disarmnuty, nie je co prikazat");
    return;
  }
  // Mod potvrdeny -> dalej riadi ArduPilot. Dalsie zmeny modu su pilotove,
  // takze uz neposielame nic.
  if (st.mode == p_.abort_mode) {
    command_on_abort_ = false;
    log(LogLevel::Info, "ABORT: mod " + p_.abort_mode + " potvrdeny, dalej riadi ArduPilot");
    return;
  }
  if (requestDue(now)) {
    if (req_attempts_ >= 5) {
      command_on_abort_ = false;
      log(LogLevel::Error, "ABORT: " + p_.abort_mode + " sa nepodarilo nastavit, riadi pilot");
      return;
    }
    vehicle_.requestMode(p_.abort_mode);
    markRequestSent(now);
  }
}

}
