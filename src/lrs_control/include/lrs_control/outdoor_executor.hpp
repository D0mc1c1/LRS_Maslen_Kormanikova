#pragma once
// A1.3: stavovy automat outdoor misie. Bez ROS -> testovatelny offline (outdoor_sim).
//
// Ramec: MAVROS local ENU (x = vychod, y = sever, z = hore), metre od EKF originu.
// Vsetky ciele misie su relativne k miestu vzletu (launch), nic nie je
// absolutna suradnica. Geodeticky pociatok (home alebo prvy fix) sluzi na
// prevod lat/lon <-> ENU (POI zadane ako lat/lon, vypisy polohy).
#include <functional>
#include <optional>
#include <string>

#include "lrs_control/geo.hpp"
#include "lrs_control/vehicle.hpp"

namespace lrs_control
{

struct OutdoorParams
{
  double altitude{5.0};          // [m] letova vyska nad miestom vzletu

  // GPS pred vzletom
  int min_satellites{6};         // kontroluje sa len ak MAVROS posiela pocet satelitov
  double max_h_acc{2.0};         // [m] max. odhad horizontalnej chyby
  double gps_settle_time{3.0};   // [s] GPS musi byt OK nepretrzite tolko
  double home_wait{10.0};        // [s] ak home nepride, pociatok = prvy platny fix

  // vzlet a stabilne drzanie
  double takeoff_tol{0.2};       // [m] vzlet hotovy pri vyske >= altitude - takeoff_tol
  double hold_tol{0.3};          // [m] max. odchylka od bodu pri drzani
  double hold_v{0.2};            // [m/s] max. rychlost pri drzani
  double hold_time{5.0};         // [s] tolko musi drzat stabilne

  // poziadavky a timeouty [s]
  double request_retry{3.0};
  double mode_timeout{20.0};
  double arm_timeout{60.0};
  double takeoff_timeout{40.0};
  double hold_timeout{60.0};
  double land_timeout{90.0};
};

class OutdoorExecutor
{
public:
  enum class State
  {
    WAIT_CONNECTION,  // MAVROS connected + prva poloha
    WAIT_GPS,         // GPS fix pouzitelny nepretrzite gps_settle_time
    WAIT_ORIGIN,      // home (alebo prvy fix) + zapamatanie miesta vzletu
    SET_MODE,         // GUIDED
    ARM,
    TAKEOFF,          // cmd/takeoff na `altitude` nad launch, overene vyskou
    HOLD,             // drzi bod nad launch, kym nie je stabilny hold_time
    LAND,             // pristatie na mieste, cakanie na disarm
    DONE,
    ABORT
  };

  enum class LogLevel { Info, Warn, Error };
  using Logger = std::function<void(LogLevel, const std::string &)>;

  OutdoorExecutor(Vehicle & vehicle, OutdoorParams params, Logger log);

  void tick(double now);

  // Externe prerusenie (sluzba ~/abort): dron pristane (LAND).
  void abort(const std::string & why);

  State state() const {return state_;}
  bool finished() const {return state_ == State::DONE || state_ == State::ABORT;}
  static const char * name(State s);

  // pre vypisy a nasledujuce casti
  bool hasOrigin() const {return ltp_.valid();}
  const LocalTangentPlane & origin() const {return ltp_;}
  const Vec3 & launch() const {return launch_;}

private:
  void enter(State s, const std::string & why);
  double inState(double now) const {return now - state_t0_;}
  bool airborneState() const;
  bool requestDue(double now) const;
  void markRequestSent(double now);
  void send(const Vec3 & p, double yaw);
  double heightAboveLaunch(const VehicleState & st) const;
  // land = true: po preruseni poziadaj o LAND; false: len prestan riadit (pilot prevzal)
  void doAbort(const std::string & why, bool land);
  void log(LogLevel l, const std::string & msg);

  void onWaitConnection(const VehicleState & st);
  void onWaitGps(const VehicleState & st, double now);
  void onWaitOrigin(const VehicleState & st, double now);
  void onSetMode(const VehicleState & st, double now);
  void onArm(const VehicleState & st, double now);
  void onTakeoff(const VehicleState & st, double now);
  void onHold(const VehicleState & st, double now);
  void onLand(const VehicleState & st, double now);
  void onAbort(const VehicleState & st, double now);

  Vehicle & vehicle_;
  OutdoorParams p_;
  Logger logger_;

  State state_{State::WAIT_CONNECTION};
  double now_{0.0};
  double state_t0_{0.0};
  double mission_t0_{-1.0};
  std::optional<double> cond_t0_;   // od kedy plati podmienka (GPS ok, stabilne drzanie)
  double last_wait_log_{-1.0};

  LocalTangentPlane ltp_;           // geodeticky pociatok
  std::string origin_src_;
  Vec3 launch_{};                   // miesto vzletu v local ENU
  Vec3 hold_target_{};
  double yaw_sp_{0.0};
  Vec3 last_sp_{};
  bool sent_this_tick_{false};

  double req_sent_t_{-1.0};
  int req_attempts_{0};
  bool land_on_abort_{true};
};

}  // namespace lrs_control
