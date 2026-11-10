#pragma once
#include <functional>
#include <future>
#include <optional>
#include <string>
#include <vector>

#include "lrs_control/mission.hpp"
#include "lrs_control/vehicle.hpp"
#include "lrs_planning/planner.hpp"

namespace lrs_control
{

struct ExecutorParams
{
  // presnost prichodu [m]
  double tol_hard{0.15};       // hard waypoint
  double tol_soft{0.50};       // soft waypoint (prelet bez zastavenia)
  double tol_pass{0.40};       // medzibod cesty z planovaca
  double v_settle{0.10};       // hard: max rychlost [m/s] ...
  double settle_time{1.0};     // ... nepretrzite tolko sekund

  // natocenie
  double yaw_tol_deg{5.0};
  double yaw_hold_time{1.0};   // yawN: drzat uhol tolko sekund
  double yaw_task_offset_deg{0.0};  // yawN v map frame = N + offset (0 = uhly v map frame)
  double rotate_settle{0.3};   // otocenie pred segmentom: drzat tolko sekund
  double min_leg_xy{0.20};     // kratsi usek v XY = vertikalny, yaw sa nemeni
  double turn_stop_deg{60.0};  // zakruta ostrejsia ako toto -> najprv otocit na mieste, potom letiet

  // vzlet / pristatie
  double takeoff_tol{0.15};    // vzlet hotovy ked z >= ciel - takeoff_tol
  double start_check_dist{1.0};  // varovanie, ak dron na starte nestoji pod takeoff WP
  double landtakeoff_wait{3.0};

  // poziadavky a timeouty [s]
  double request_retry{3.0};   // znova poslat poziadavku po tolkych sekundach
  double mode_timeout{20.0};
  double arm_timeout{60.0};    // EKF/prearm checky mozu chvilu trvat
  double takeoff_timeout{30.0};
  double rotate_timeout{15.0};
  double segment_timeout{120.0};
  double yaw_task_timeout{20.0};
  double land_timeout{60.0};
  // strata spojenia: /mavros/state chodi ~1 Hz, poloha v SITL ~4 Hz
  double state_timeout{3.0};
  double pose_timeout{1.0};
};

// tick(now) sa vola periodicky (napr. 20 Hz) z riadiaceho timera.
class MissionExecutor
{
public:
  enum class State
  {
    WAIT_CONNECTION,  // cakam na /mavros/state connected + prvu polohu
    PLAN,             // naplanujem VSETKY segmenty este na zemi
    SET_MODE,         // GUIDED
    ARM,
    TAKEOFF,
    ROTATE,           // na mieste sa otocim do smeru letu dalsieho segmentu
    NAVIGATE,         // letim cez medzibody cesty (pass-through)
    ARRIVE,           // posledny bod segmentu = waypoint misie (hard/soft)
    TASK_YAW,
    LAND,
    GROUND_WAIT,      // landtakeoff: pauza na zemi
    NEXT_WAYPOINT,
    DONE,
    ABORT
  };

  enum class LogLevel { Info, Warn, Error };
  using Logger = std::function<void(LogLevel, const std::string &)>;

  MissionExecutor(
    Vehicle & vehicle, lrs_planning::Planner & planner, Mission mission,
    ExecutorParams params, Logger log);

  void tick(double now);

  State state() const {return state_;}
  // Hotovo = DONE, alebo ABORT, v ktorom uz nie je co prikazat (LAND potvrdeny,
  // dron na zemi, alebo riadenie prevzal pilot). Potom sa node moze ukoncit.
  bool finished() const
  {
    return state_ == State::DONE || (state_ == State::ABORT && !command_on_abort_);
  }
  std::size_t waypointIndex() const {return wp_idx_;}
  const std::vector<lrs_planning::Path> & segments() const {return segments_;}
  static const char * name(State s);

  // abort (ROS sluzba alebo chyba misie): prepne do LAND
  void abort(const std::string & why);

private:
  enum class AfterLand { Done, GroundWait };

  void enter(State s, const std::string & why);
  double inState(double now) const {return now - state_t0_;}

  // jednotlive stavy
  void onWaitConnection(const VehicleState & st);
  void onPlan(const VehicleState & st, double now);
  void onSetMode(const VehicleState & st, double now);
  void onArm(const VehicleState & st, double now);
  void onTakeoff(const VehicleState & st, double now);
  void onRotate(const VehicleState & st, double now);
  void onNavigate(const VehicleState & st, double now);
  void onArrive(const VehicleState & st, double now);
  void onTaskYaw(const VehicleState & st, double now);
  void onLand(const VehicleState & st, double now);
  void onGroundWait(double now);
  void onNextWaypoint(const VehicleState & st);
  void onAbort(const VehicleState & st, double now);

  // pomocne
  void beginSegment(const VehicleState & st);
  void waypointReached(const VehicleState & st, double now);
  void startLanding(AfterLand after);
  void startTakeoffSequence(double z_map, bool resume_next);
  bool requestDue(double now) const;   // treba (znova) poslat poziadavku?
  void markRequestSent(double now);
  std::optional<double> legYaw(std::size_t to_idx) const;  // smer useku path[to-1]->path[to]
  bool airborneState() const;
  bool controlState() const;     // dron je (alebo bude) armnuty a program ho riadi
  // pilot prevzal / spojenie stratene: prestat riadit, NIC neprikazovat
  void release(const std::string & why);
  void log(LogLevel l, const std::string & msg);
  void send(const Vec3 & p, double yaw);   // pozicny setpoint + zapamatanie
  void logSummary();

  struct Arrival { std::size_t wp; double err; double seg_time; };

  Vehicle & vehicle_;
  lrs_planning::Planner & planner_;
  Mission mission_;
  ExecutorParams p_;
  Logger logger_;

  State state_{State::WAIT_CONNECTION};
  double now_{0.0};
  double state_t0_{0.0};
  double mission_t0_{-1.0};

  std::vector<lrs_planning::Path> segments_;  // segments_[i] konci vo waypointe i

  // Planovanie bezi v samostatnom vlakne (std::async), aby neblokovalo riadiaci
  // timer. Vysledok sa vyzdvihne v stave PLAN, ked je future pripravena.
  struct PlanOutcome
  {
    std::vector<lrs_planning::Path> segments;
    std::vector<std::string> lines;   // logy po segmentoch
    std::string error;                // prazdne = OK
  };
  std::future<PlanOutcome> plan_future_;
  double last_plan_log_{0.0};
  std::size_t wp_idx_{0};
  std::size_t path_idx_{0};     // index cieloveho bodu v segments_[wp_idx_]

  Vec3 hold_pos_{};             // kde drzat poziciu (ROTATE, TASK_YAW)
  double yaw_sp_{0.0};          // aktualny yaw setpoint (unwrapnuty)
  bool stopped_{true};          // dron stoji -> pred segmentom sa otoci na mieste
  std::optional<double> cond_t0_;  // od kedy plati podmienka (settle, yaw hold)

  double takeoff_z_{0.0};
  bool resume_next_after_takeoff_{false};
  AfterLand after_land_{AfterLand::Done};

  double req_sent_t_{-1.0};
  int req_attempts_{0};

  double seg_t0_{0.0};
  Vec3 last_sp_{};
  bool sent_this_tick_{false};
  bool command_on_abort_{true};
  double last_disconnect_warn_{-1.0};
  std::vector<Arrival> arrivals_;
};

}
