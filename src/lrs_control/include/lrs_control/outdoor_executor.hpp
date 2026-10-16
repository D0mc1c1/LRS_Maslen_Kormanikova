#pragma once
// A1.3: stavovy automat outdoor misie. Bez ROS -> testovatelny offline (outdoor_sim).
//
// Ramec: MAVROS local ENU (x = vychod, y = sever, z = hore), metre od EKF originu.
// Vsetky ciele misie su relativne k miestu vzletu (launch), nic nie je
// absolutna suradnica. Geodeticky pociatok (home alebo prvy fix) sluzi na
// prevod lat/lon <-> ENU (POI zadane ako lat/lon, vypisy polohy).
#include <cmath>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "lrs_control/geo.hpp"
#include "lrs_control/trajectory.hpp"
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

  // POI (bod zaujmu na zemi, stred stvorca). Bud lat/lon, alebo posun od miesta vzletu.
  bool poi_use_latlon{false};
  double poi_lat{0.0}, poi_lon{0.0};      // [deg] ak poi_use_latlon
  double poi_east{0.0}, poi_north{5.0};   // [m] inak: posun od miesta vzletu (ENU)

  // stvorec okolo POI: W1 SZ, W2 SV, W3 JV, W4 JZ, spat W1 (v smere hodinovych ruciciek)
  double square_side{5.0};       // [m]
  double speed{1.0};             // [m/s] max. rychlost na usekoch (stvorec, presuny)
  double accel{0.5};             // [m/s^2] zrychlenie / spomalenie na usekoch
  double corner_tol{0.3};        // [m] roh dosiahnuty, ked je dron blizsie
  double leg_timeout_extra{20.0};// [s] usek moze trvat o tolko dlhsie ako podla profilu
  bool poi_yaw_from_setpoint{false}; // smer na POI z: false = skutocnej polohy, true = setpointu
  double poi_min_dist{0.5};      // [m] blizsie k POI sa smer nepocita (nedefinovany)
  double align_tol_deg{5.0};     // [deg] pred stvorcom natocit na POI s touto presnostou
  double align_time{1.0};        // [s] ... nepretrzite tolko

  // feedforward do setpointu
  bool velocity_ff{true};        // rychlost trajektorie
  bool yaw_rate_ff{true};        // rychlost otacania (ArduCopter 4.1+)

  // osmicka okolo POI (casova trajektoria, timer v node = tick)
  double fig8_amplitude{3.0};    // [m] A, zadanie: 3 m
  double fig8_omega{0.25};       // [rad/s] max. rychlost v strede A*omega*sqrt(2) = 1.06 m/s
  double fig8_laps{1.0};         // pocet kol (aspon 1)
  double fig8_ramp{3.0};         // [s] plynuly rozbeh / dobeh fazy, 0 = th = omega*t
  double fig8_rotation_deg{0.0}; // [deg] otocenie dlhej osi od vychodu (ENU, CCW)
  double fig8_max_speed{1.5};    // [m/s] vyssia A*omega*sqrt(2) -> misia sa nespusti
  double max_track_err{1.5};     // [m] odchylka od setpointu na osmicke -> ABORT + LAND

  // bezpecnost: max. vodorovna vzdialenost misie od miesta vzletu
  double geofence_radius{15.0};  // [m]

  // abort a strata spojenia (cast 5)
  std::string abort_mode{"LAND"};// mod po ~/abort alebo chybe misie: LAND / RTL / BRAKE / LOITER
  double state_timeout{3.0};     // [s] /mavros/state neprislo -> spojenie stratene
  double pose_timeout{1.0};      // [s] poloha neprisla -> spojenie stratene

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
    POI_ALIGN,        // na mieste natocit nos na POI
    SQUARE,           // presun na W1, potom W1-W2-W3-W4-W1, nos stale na POI
    FIG8_ENTRY,       // presun z W1 do stredu osmicky (nad POI), natacanie na tangentu
    FIG8_ALIGN,       // v strede dorovnat yaw na tangentu v t = 0
    FIG8,             // osmicka: kazdy tick novy setpoint z rovnic, yaw = tangenta
    RETURN,           // navrat nad miesto vzletu
    LAND,             // pristatie na mieste, cakanie na disarm
    DONE,
    ABORT
  };

  enum class LogLevel { Info, Warn, Error };
  using Logger = std::function<void(LogLevel, const std::string &)>;

  OutdoorExecutor(Vehicle & vehicle, OutdoorParams params, Logger log);

  void tick(double now);

  // Externe prerusenie (sluzba ~/abort): prepne do abort_mode (LAND / RTL / ...).
  void abort(const std::string & why);

  State state() const {return state_;}
  // Hotovo = DONE, alebo ABORT, v ktorom uz nie je co prikazat (mod potvrdeny,
  // dron na zemi, alebo riadenie prevzal pilot). Potom sa node moze ukoncit.
  bool finished() const
  {
    return state_ == State::DONE || (state_ == State::ABORT && !command_on_abort_);
  }
  static const char * name(State s);

  // pre vypisy a nasledujuce casti
  bool hasOrigin() const {return ltp_.valid();}
  const LocalTangentPlane & origin() const {return ltp_;}
  const Vec3 & launch() const {return launch_;}
  const Vec3 & poi() const {return poi_;}

  // prevod geodeticke <-> MAVROS local (po WAIT_ORIGIN)
  Vec3 llaToLocal(const LatLonAlt & g) const;
  LatLonAlt localToLla(const Vec3 & p) const;

  // chyba natocenia na POI pocas stvorca (W1 -> ... -> W1), bez presunu na W1
  struct PoiStats
  {
    double max_deg{0.0};
    double sum_sq{0.0};
    long n{0};
    double rmsDeg() const {return n > 0 ? std::sqrt(sum_sq / n) : 0.0;}
  };
  const PoiStats & poiStats() const {return poi_stats_;}

  // osmicka: odchylka skutocnej polohy od setpointu, natocenie vs. tangenta, tvar
  struct Fig8Stats
  {
    double track_max{0.0}, track_sum_sq{0.0};
    double yaw_max_deg{0.0}, yaw_sum_sq{0.0};
    double max_yaw_step_deg{0.0};   // najvacsi skok yaw setpointu medzi tickmi
    double x_min{0.0}, x_max{0.0}, y_min{0.0}, y_max{0.0};   // skutocna poloha od stredu
    long n{0};
    bool done{false};
    double trackRms() const {return n > 0 ? std::sqrt(track_sum_sq / n) : 0.0;}
    double yawRmsDeg() const {return n > 0 ? std::sqrt(yaw_sum_sq / n) : 0.0;}
  };
  const Fig8Stats & fig8Stats() const {return fig8_stats_;}
  const Figure8 & figure8() const {return fig8_;}
  // posledna odchylka od osmicky [m], < 0 ak sa prave nepocita
  double trackErr() const {return track_err_;}
  // posledna chyba natocenia na POI [deg], < 0 ak sa prave nepocita
  double poiErrDeg() const {return poi_err_deg_;}
  int cornersReached() const {return corners_reached_;}

private:
  void enter(State s, const std::string & why);
  double inState(double now) const {return now - state_t0_;}
  bool airborneState() const;
  bool requestDue(double now) const;
  void markRequestSent(double now);
  void send(const Vec3 & p, double yaw);
  // setpoint trajektorie: poloha + rychlost + yaw + yaw_rate (feedforward podla parametrov)
  void sendTraj(const Vec3 & p, const Vec3 & v, double yaw, double yaw_rate);
  // smer na POI z bodu p, ak je p prilis blizko POI, vrati `fallback`
  double poiYaw(const Vec3 & p, double fallback) const;
  // rychlost zmeny smeru na POI pri pohybe bodu p rychlostou v [rad/s]
  double poiYawRate(const Vec3 & p, const Vec3 & v) const;
  bool setupMissionGeometry(const VehicleState & st);
  void startLeg(std::size_t i, const Vec3 & from);
  void startFig8Entry(const Vec3 & from);
  void startReturn(const Vec3 & from);
  // drzi `pos` a otaca na `target`; true = natoceny nepretrzite align_time
  bool alignStep(const VehicleState & st, double now, const Vec3 & pos, double target,
    const std::string & what);
  double heightAboveLaunch(const VehicleState & st) const;
  // command = true: po preruseni prepni do abort_mode;
  // false: len prestan riadit, NIC neprikazuj (pilot prevzal / spojenie stratene)
  void doAbort(const std::string & why, bool command);
  // stavy, v ktorych dron je (alebo bude) armnuty a program ho riadi
  bool controlState() const;
  void log(LogLevel l, const std::string & msg);

  void onWaitConnection(const VehicleState & st);
  void onWaitGps(const VehicleState & st, double now);
  void onWaitOrigin(const VehicleState & st, double now);
  void onSetMode(const VehicleState & st, double now);
  void onArm(const VehicleState & st, double now);
  void onTakeoff(const VehicleState & st, double now);
  void onHold(const VehicleState & st, double now);
  void onPoiAlign(const VehicleState & st, double now);
  void onSquare(const VehicleState & st, double now);
  void onFig8Entry(const VehicleState & st, double now);
  void onFig8Align(const VehicleState & st, double now);
  void onFig8(const VehicleState & st, double now);
  void onReturn(const VehicleState & st, double now);
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

  Vec3 geo_offset_{};               // local = ltp_.toEnu(lla) + geo_offset_
  Vec3 poi_{};                      // POI v local ENU (vo vyske miesta vzletu = na zemi)
  std::vector<Vec3> square_;        // W1, W2, W3, W4, W1 (vo vyske letu)
  std::size_t leg_i_{0};            // index ciela v square_ (0 = presun na W1)
  StraightLeg leg_;
  double leg_t0_{0.0};
  double leg_err_max_{0.0};
  PoiStats poi_stats_;
  double poi_err_deg_{-1.0};
  int corners_reached_{0};

  Figure8 fig8_;
  double fig8_t0_{0.0};
  double entry_yaw0_{0.0}, entry_yaw1_{0.0};
  int fig8_quarter_{0};
  Fig8Stats fig8_stats_;
  double track_err_{-1.0};

  double req_sent_t_{-1.0};
  int req_attempts_{0};
  bool command_on_abort_{true};
};

}  // namespace lrs_control
