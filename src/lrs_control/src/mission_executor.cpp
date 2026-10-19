#include "lrs_control/mission_executor.hpp"

#include <chrono>
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
}

MissionExecutor::MissionExecutor(
  Vehicle & vehicle, lrs_planning::Planner & planner, Mission mission,
  ExecutorParams params, Logger log)
: vehicle_(vehicle), planner_(planner), mission_(std::move(mission)), p_(params),
  logger_(std::move(log))
{
}

const char * MissionExecutor::name(State s)
{
  // Prevod stavov na text pre vypis do konzoly
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

// Hlavna riadiaca slucka

void MissionExecutor::tick(double now)
{
  now_ = now;

  // Inicializacia casovacov pri prvom spusteni
  if (mission_t0_ < 0.0) {
    mission_t0_ = now;
    state_t0_ = now;
    log(LogLevel::Info, std::string("start v stave ") + name(state_));
  }

  sent_this_tick_ = false;
  const VehicleState st = vehicle_.state();

  // Kontrola spojenia pocas riadenia
  if (controlState()) {
    std::string why;
    if (!linkOk(st, p_.state_timeout, p_.pose_timeout, why)) {
      release("spojenie stratene: " + why);
    }
  } else if (state_ != State::WAIT_CONNECTION && !finished() && !st.connected &&
    now - last_disconnect_warn_ > 1.0)
  {
    log(LogLevel::Warn, "MAVROS hlasi connected=false");
    last_disconnect_warn_ = now;
  }

  // Kontrola ci pilot nezmenil rezim alebo nevypol motory
  if (airborneState() && (st.mode != "GUIDED" || !st.armed)) {
    release("mod zmeneny zvonka na '" + st.mode + "', armed=" + (st.armed ? "1" : "0"));
  }

  // Vykonanie funkcie podla aktualneho stavu
  switch (state_) {
    case State::WAIT_CONNECTION: onWaitConnection(st); break;
    case State::PLAN: onPlan(st, now); break;
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

  // Posielanie cielovej polohy pocas letu: KAZDY tick, aj v ticku, kde sa menil stav
  if (airborneState() && !sent_this_tick_) {
    send(last_sp_, yaw_sp_);
  }
}

void MissionExecutor::enter(State s, const std::string & why)
{
  // Zaznamenanie prechodu do noveho stavu
  char head[96];
  std::snprintf(head, sizeof(head), "[t=%6.2fs] WP %zu/%zu  %s -> %s", now_ - mission_t0_,
    std::min(wp_idx_ + 1, mission_.size()), mission_.size(), name(state_), name(s));
  log(s == State::ABORT ? LogLevel::Error : LogLevel::Info, std::string(head) + "  (" + why + ")");
  state_ = s;
  state_t0_ = now_;
  req_sent_t_ = -1.0;
  req_attempts_ = 0;
  cond_t0_.reset();

  // Vypisanie suhrnu po dokonceni misie
  if (s == State::DONE) {
    logSummary();
  }
}

void MissionExecutor::abort(const std::string & why)
{
  // Prerusenie misie s poziadavkou na pristatie
  if (state_ != State::ABORT && state_ != State::DONE) {
    command_on_abort_ = true;
    enter(State::ABORT, why + " -> LAND");
  }
}

void MissionExecutor::release(const std::string & why)
{
  // Uvolnenie riadenia bez posielania dalsich prikazov
  if (state_ != State::ABORT && state_ != State::DONE) {
    command_on_abort_ = false;
    enter(State::ABORT, why + " -> prestavam riadit, nic neprikazujem");
  }
}

bool MissionExecutor::controlState() const
{
  // Stavy v ktorych sa kontroluje spojenie
  return state_ == State::SET_MODE || state_ == State::ARM || state_ == State::TAKEOFF ||
         state_ == State::LAND || airborneState();
}

bool MissionExecutor::airborneState() const
{
  // Stavy pocas ktorych je dron vo vzduchu
  return state_ == State::ROTATE || state_ == State::NAVIGATE || state_ == State::ARRIVE ||
         state_ == State::TASK_YAW;
}

void MissionExecutor::send(const Vec3 & p, double yaw)
{
  // Odoslanie cielovej polohy a orientacie drona
  vehicle_.sendPositionTarget(p, yaw);
  last_sp_ = p;
  sent_this_tick_ = true;
}

void MissionExecutor::log(LogLevel l, const std::string & msg)
{
  // Zapis spravy do logu
  if (logger_) {
    logger_(l, msg);
  }
}

bool MissionExecutor::requestDue(double now) const
{
  // Kontrola ci treba poziadavku zopakovat
  if (req_sent_t_ < 0.0) {
    return true;
  }
  const auto s = vehicle_.requestStatus();
  const double since = now - req_sent_t_;
  return (s == RequestStatus::Failed && since > 1.0) || since > p_.request_retry;
}

void MissionExecutor::markRequestSent(double now)
{
  // Ulozenie casu a poctu odoslanych poziadaviek
  if (req_attempts_ > 0) {
    const bool failed = vehicle_.requestStatus() == RequestStatus::Failed;
    log(LogLevel::Warn, std::string(name(state_)) + ": opakujem poziadavku (pokus " +
      std::to_string(req_attempts_ + 1) + ", predosla " + (failed ? "zamietnuta" : "bez odozvy") +
      ")");
  }
  req_sent_t_ = now;
  ++req_attempts_;
}

// Jednotlive stavy misie

void MissionExecutor::onWaitConnection(const VehicleState & st)
{
  // Cakanie na spojenie s dronom a dostupnu polohu
  if (st.connected && st.has_pose) {
    enter(State::PLAN, "MAVROS connected, poloha " + pt(st.pos) + " v map frame");
  }
}

void MissionExecutor::onPlan(const VehicleState & st, double now)
{
  // Spustenie planovania trasy v samostatnom vlakne
  if (!plan_future_.valid()) {
    if (st.armed) {
      release("dron je uz armnuty, misiu spusti s dronom na zemi");
      return;
    }

    // Kontrola ci dron stoji na ocakavanom mieste vzletu
    const auto & wp0 = mission_.front();
    if (wp0.task == TaskType::Takeoff && distXY(st.pos, wp0.p) > p_.start_check_dist) {
      log(LogLevel::Warn, "dron stoji v " + pt(st.pos) + ", ale takeoff WP1 je " + pt(wp0.p) +
        " (" + fmt("%.1f", distXY(st.pos, wp0.p)) + " m). Ak dron naozaj stoji na WP1, "
        "je zly prevod map->local (local_from_map_yaw_deg / local_from_map_xyz)!");
    }

    // Nastavenie zaciatku trasy podla aktualnej polohy drona
    const Vec3 first_start{st.pos.x, st.pos.y, mission_.front().p.z};
    log(LogLevel::Info, "planujem " + std::to_string(mission_.size()) + " segmentov (vo vlakne)");
    last_plan_log_ = now;

    // Planovanie kazdeho segmentu misie v samostatnom vlakne.
    // mission_ a planner_ pouziva pocas planovania len toto vlakno (netreba mutex)
    plan_future_ = std::async(std::launch::async, [this, first_start]() {
        PlanOutcome out;
        for (std::size_t i = 0; i < mission_.size(); ++i) {
          const Vec3 a = i == 0 ? first_start : mission_[i - 1].p;
          const Vec3 b = mission_[i].p;
          const auto r = planner_.plan(a, b);

          // Ukoncenie planovania ak sa cesta nenasla
          if (!r || r->path.empty()) {
            out.error = "segment " + std::to_string(i) + ": " + pt(a) + " -> " + pt(b) +
            " NEMA CESTU (riadok " + std::to_string(mission_[i].line) + " v misii)";
            return out;
          }

          // Ulozenie informacii o naplanovanom segmente
          char line[160];
          std::snprintf(line, sizeof(line),
          "  segment %zu -> WP%zu %s: %zu bodov (pred zjednodusenim %zu), %.1f ms",
          i, i + 1, describe(mission_[i]).c_str(), r->path.size(), r->raw_points, r->time_ms);
          out.lines.push_back(line);
          out.segments.push_back(r->path);
        }
        return out;
      });
    return;
  }

  // Cakanie na dokoncenie planovania bez blokovania riadiacej slucky
  if (plan_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
    if (now - last_plan_log_ >= 5.0) {
      log(LogLevel::Info, "  planovanie bezi " + fmt("%.0f", inState(now)) + " s ...");
      last_plan_log_ = now;
    }
    return;
  }

  // Prevzatie vysledku planovania
  PlanOutcome out = plan_future_.get();
  for (const auto & l : out.lines) {
    log(LogLevel::Info, l);
  }
  if (!out.error.empty()) {
    abort(out.error);
    return;
  }
  segments_ = std::move(out.segments);
  log(LogLevel::Info, "planovanie hotove za " + fmt("%.1f", inState(now)) + " s");
  startTakeoffSequence(mission_.front().p.z, false);
}

void MissionExecutor::onSetMode(const VehicleState & st, double now)
{
  // Cakanie na potvrdenie rezimu GUIDED
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
  // Cakanie na aktivovanie motorov
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
  // Ukoncenie vzletu po dosiahnuti cielovej vysky
  if (st.pos.z >= takeoff_z_ - p_.takeoff_tol) {
    hold_pos_ = {st.pos.x, st.pos.y, takeoff_z_};
    last_sp_ = hold_pos_;
    yaw_sp_ = st.yaw;
    stopped_ = true;

    if (resume_next_after_takeoff_) {
      // Pokracovanie dalsim bodom po pristati a opatovnom vzlete
      enter(State::NEXT_WAYPOINT, "vo vzduchu z=" + fmt("%.2f", st.pos.z) + ", landtakeoff hotovy");
    } else {
      // Pokracovanie k prvemu bodu misie
      log(LogLevel::Info, "vo vzduchu z=" + fmt("%.2f", st.pos.z));
      beginSegment(st);
    }
    return;
  }
  if (st.mode != "GUIDED" && inState(now) > 1.0) {
    release("pocas vzletu sa mod zmenil na '" + st.mode + "'");
    return;
  }
  if (!st.armed && inState(now) > 2.0) {
    // Opakovanie aktivacie motorov po automatickom disarme
    enter(State::SET_MODE, "dron sa disarmol pred vzletom, opakujem arm");
    return;
  }
  if (inState(now) > p_.takeoff_timeout) {
    abort("vzlet nedosiahol z=" + fmt("%.2f", takeoff_z_));
    return;
  }

  // Opakovanie poziadavky na vzlet pri chybe alebo chybajucej odozve
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
  // Priprava na let po naplanovanej trase k dalsiemu waypointu
  const auto & path = segments_[wp_idx_];
  seg_t0_ = now_;
  last_sp_ = hold_pos_;

  if (path.size() < 2) {
    path_idx_ = path.size() - 1;
    enter(State::ARRIVE, "segment ma jediny bod");
    return;
  }

  // Zistenie smeru prveho vodorovneho useku
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

  // Otocenie na mieste pred zaciatkom letu ak je potrebne:
  // dron stoji (hard bod, vzlet, yaw uloha), alebo je zakruta ostra aj po soft bode
  if ((stopped_ && err > deg2rad(p_.yaw_tol_deg)) || err > deg2rad(p_.turn_stop_deg)) {
    enter(State::ROTATE, info + ", otocenie o " + fmt("%.0f", rad2deg(err)) + " deg");
  } else {
    enter(State::NAVIGATE, info);
  }
}

std::optional<double> MissionExecutor::legYaw(std::size_t to_idx) const
{
  // Vypocet smeru letu medzi dvoma bodmi trasy
  const auto & path = segments_[wp_idx_];
  if (to_idx == 0 || to_idx >= path.size()) {
    return std::nullopt;
  }
  const Vec3 & a = path[to_idx - 1];
  const Vec3 & b = path[to_idx];

  // Preskocenie vertikalnych usekov pri vypocte yaw
  if (distXY(a, b) < p_.min_leg_xy) {
    return std::nullopt;
  }
  return segmentYaw(a, b);
}

void MissionExecutor::onRotate(const VehicleState & st, double now)
{
  // Otacanie drona na mieste do smeru dalsieho useku
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

  // Pokracovanie aj pri prekroceni casoveho limitu otacania
  if (state_ == State::ROTATE && inState(now) > p_.rotate_timeout) {
    log(LogLevel::Warn, "otocenie nedokoncene (chyba " + fmt("%.1f", rad2deg(err)) +
      " deg), pokracujem");
    enter(State::NAVIGATE, "timeout otocenia");
  }
}

void MissionExecutor::onNavigate(const VehicleState & st, double now)
{
  // Let k jednotlivym bodom naplanovanej trasy
  const auto & path = segments_[wp_idx_];
  const std::size_t last = path.size() - 1;

  if (path_idx_ >= last) {
    enter(State::ARRIVE, "posledny bod segmentu = waypoint misie");
    return;
  }

  const Vec3 & target = path[path_idx_];

  // Nastavenie orientacie podla aktualneho useku
  if (const auto ly = legYaw(path_idx_)) {
    yaw_sp_ = unwrapNear(*ly, st.yaw);
  }
  send(target, yaw_sp_);

  // Prelet cez medzibody bez zastavenia
  if (dist(st.pos, target) < p_.tol_pass) {
    ++path_idx_;
    log(LogLevel::Info, "  medzibod " + std::to_string(path_idx_ - 1) + "/" +
      std::to_string(last - 1) + " " + pt(target) + " preleteny");

    // Ostra zakruta v medzibode: zastavit v nom a otocit sa na mieste,
    // aby dron neletel dalsi usek bokom (nos vzdy v smere letu)
    if (const auto ny = legYaw(path_idx_)) {
      const double turn = std::abs(angleDiff(*ny, st.yaw));
      if (turn > deg2rad(p_.turn_stop_deg)) {
        hold_pos_ = target;
        yaw_sp_ = unwrapNear(*ny, st.yaw);
        enter(State::ROTATE, "zakruta " + fmt("%.0f", rad2deg(turn)) +
          " deg v medzibode, otocenie na mieste");
        return;
      }
    }
  }

  // Kontrola casoveho limitu segmentu
  if (now - seg_t0_ > p_.segment_timeout) {
    abort("segment trva dlhsie ako " + fmt("%.0f", p_.segment_timeout) + " s");
  }
}

void MissionExecutor::onArrive(const VehicleState & st, double now)
{
  // Doletenie na cielovy waypoint misie
  const auto & path = segments_[wp_idx_];
  const Vec3 target = path.back();

  if (const auto ly = legYaw(path.size() - 1)) {
    yaw_sp_ = unwrapNear(*ly, st.yaw);
  }
  send(target, yaw_sp_);

  const double d = dist(st.pos, target);
  const double v = norm(st.vel);
  const auto & wp = mission_[wp_idx_];

  // Soft waypoint vyzaduje iba priblizenie k cielu
  if (wp.precision == Precision::Soft) {
    if (d < p_.tol_soft) {
      stopped_ = wp.task == TaskType::Takeoff;
      waypointReached(st, now);
      return;
    }
  } else {
    // Hard waypoint vyzaduje presnost a nizku rychlost
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

  // Ukoncenie letu pri nedosiahnuti ciela v limite
  if (now - seg_t0_ > p_.segment_timeout) {
    abort("waypoint nedosiahnuty, chyba " + fmt("%.2f", d) + " m");
  }
}

void MissionExecutor::waypointReached(const VehicleState & st, double now)
{
  // Zaznamenanie dosiahnutia waypointu
  const auto & wp = mission_[wp_idx_];
  const double err = dist(st.pos, wp.p);
  arrivals_.push_back({wp_idx_, err, now - seg_t0_});
  hold_pos_ = wp.p;
  last_sp_ = wp.p;

  const std::string why = "WP" + std::to_string(wp_idx_ + 1) + " dosiahnuty, chyba " +
    fmt("%.3f", err) + " m, v=" + fmt("%.2f", norm(st.vel)) + " m/s";

  // Vykonanie ulohy priradenej waypointu
  switch (wp.task) {
    case TaskType::None:
    case TaskType::Takeoff:
      enter(State::NEXT_WAYPOINT, why);
      break;

    case TaskType::Yaw:
      // Otocenie na pozadovany uhol
      yaw_sp_ = unwrapNear(deg2rad(wp.yaw_deg + p_.yaw_task_offset_deg), st.yaw);
      stopped_ = true;
      enter(State::TASK_YAW, why + ", task yaw" + fmt("%g", wp.yaw_deg));
      break;

    case TaskType::Land:
      log(LogLevel::Info, why);
      startLanding(AfterLand::Done);
      break;

    case TaskType::LandTakeoff:
      // Pristatie a opatovny vzlet pri ulohe landtakeoff
      log(LogLevel::Info, why);
      startLanding(AfterLand::GroundWait);
      break;
  }
}

void MissionExecutor::onTaskYaw(const VehicleState & st, double now)
{
  // Drzanie polohy pocas otacania
  send(hold_pos_, yaw_sp_);
  const double err = std::abs(angleDiff(yaw_sp_, st.yaw));

  // Kontrola dosiahnutia pozadovanej orientacie
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

  // Pokracovanie misie aj pri nedosiahnuti uhla v limite
  if (state_ == State::TASK_YAW && inState(now) > p_.yaw_task_timeout) {
    log(LogLevel::Warn, "yaw task nedosiahnuty (chyba " + fmt("%.1f", rad2deg(err)) +
      " deg), pokracujem");
    enter(State::NEXT_WAYPOINT, "timeout yaw tasku");
  }
}

void MissionExecutor::startLanding(AfterLand after)
{
  // Spustenie procesu pristatia
  after_land_ = after;
  enter(State::LAND, after == AfterLand::GroundWait ? "landtakeoff: pristatie" : "pristatie");
}

void MissionExecutor::onLand(const VehicleState & st, double now)
{
  // Ukoncenie pristatia po deaktivacii motorov
  if (!st.armed) {
    if (after_land_ == AfterLand::GroundWait) {
      enter(State::GROUND_WAIT, "na zemi a disarmnuty, cakam " +
        fmt("%.1f", p_.landtakeoff_wait) + " s");
    } else {
      enter(State::DONE, "na zemi a disarmnuty");
    }
    return;
  }

  // Kontrola casoveho limitu pristatia
  if (inState(now) > p_.land_timeout) {
    abort("pristatie trva dlhsie ako " + fmt("%.0f", p_.land_timeout) + " s");
    return;
  }

  // Odoslanie poziadavky na pristatie
  if (st.mode == "GUIDED") {
    if (requestDue(now)) {
      vehicle_.requestLand();
      markRequestSent(now);
    }
  } else if (st.mode != "LAND") {
    release("pocas pristatia mod zmeneny na '" + st.mode + "'");
  }
}

void MissionExecutor::onGroundWait(double now)
{
  // Cakanie na zemi pred dalsim vzletom
  if (inState(now) >= p_.landtakeoff_wait) {
    startTakeoffSequence(mission_[wp_idx_].p.z, true);
  }
}

void MissionExecutor::startTakeoffSequence(double z_map, bool resume_next)
{
  // Nastavenie vysky vzletu a dalsieho postupu misie
  takeoff_z_ = z_map;
  resume_next_after_takeoff_ = resume_next;
  enter(State::SET_MODE, "vzletova sekvencia na z=" + fmt("%.2f", z_map));
}

void MissionExecutor::onNextWaypoint(const VehicleState & st)
{
  // Prechod na dalsi waypoint misie
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
  // Pri uvolnenom riadeni sa neposielaju ziadne prikazy
  if (!command_on_abort_) {
    return;
  }

  if (!st.armed) {
    command_on_abort_ = false;
    log(LogLevel::Info, "ABORT: dron je disarmnuty, nie je co prikazat");
    return;
  }

  // Po potvrdeni LAND prebera riadenie autopilot
  // (indoor nie RTL: stupa do RTL_ALT, v hale nebezpecne)
  if (st.mode == "LAND") {
    command_on_abort_ = false;
    log(LogLevel::Info, "ABORT: LAND potvrdeny, dalej riadi ArduPilot");
    return;
  }

  // Opakovanie poziadavky na bezpecne pristatie
  if (requestDue(now)) {
    if (req_attempts_ >= 5) {
      command_on_abort_ = false;
      log(LogLevel::Error, "ABORT: LAND sa nepodarilo nastavit, riadi pilot");
      return;
    }
    vehicle_.requestMode("LAND");
    markRequestSent(now);
  }
}

void MissionExecutor::logSummary()
{
  // Vytvorenie zaverecneho prehladu misie
  std::ostringstream s;
  s << "SUHRN MISIE: cas " << fmt("%.1f", now_ - mission_t0_) << " s\n"
    << "  WP  precision  task          chyba[m]  segment[s]\n";

  // Vypis chyby a casu pri jednotlivych waypointoch
  for (const auto & a : arrivals_) {
    const auto & wp = mission_[a.wp];
    char b[128];
    std::snprintf(b, sizeof(b), "  %-3zu %-10s %-13s %8.3f  %9.1f\n", a.wp + 1,
      toString(wp.precision).c_str(), toString(wp.task).c_str(), a.err, a.seg_time);
    s << b;
  }
  log(LogLevel::Info, s.str());
}

}