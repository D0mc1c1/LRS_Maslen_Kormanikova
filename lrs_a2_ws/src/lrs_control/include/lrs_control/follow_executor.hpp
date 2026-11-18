#pragma once
#include <deque>
#include <functional>
#include <string>
#include <vector>

#include "lrs_control/search_pattern.hpp"
#include "lrs_control/trajectory.hpp"
#include "lrs_control/vehicle.hpp"

namespace lrs_control
{

struct TargetObs
{
  double t{0.0};
  Vec3 p{};
};

struct FollowParams
{
  int sysid{1};
  double search_alt{10.0};       // [m] nad miestom vzletu
  double follow_alt{6.0};
  double lost_timeout{1.5};
  int confirm_count{3};
  double confirm_window{1.0};
  std::string abort_mode{"LOITER"};

  // vzlet (rovnako ako A1)
  double takeoff_tol{0.3};       // [m] vzlet hotovy pri vyske >= search_alt - takeoff_tol
  double state_timeout{3.0};     // [s] mavros/state neprislo -> spojenie stratene
  double pose_timeout{1.0};      // [s] poloha neprisla -> spojenie stratene
  double request_retry{3.0};     // [s] po tomto case sa poziadavka posle znova
  double mode_timeout{20.0};
  double arm_timeout{60.0};
  double takeoff_timeout{40.0};

  // hladanie
  double search_speed{1.5};        // [m/s] rychlost na ramenach
  double search_accel{0.8};        // [m/s^2]
  double search_overlap{0.3};      // prekrytie pasov 0..0.9
  double search_max_radius{25.0};  // [m] dosah vzoru od stredu
  double cam_hfov_deg{60.0};
  double cam_aspect{0.75};
  // TODO: dalsie parametre
};

class FollowExecutor
{
public:
  enum class State { WAIT_CONNECTION, SET_MODE, ARM, TAKEOFF, SEARCH, TRACK, ABORT };

  using Logger = std::function<void(const std::string &)>;

  FollowExecutor(Vehicle & vehicle, FollowParams params, Logger log);

  void tick(double now, const std::vector<TargetObs> & obs);
  void abort(const std::string & why);

  State state() const {return state_;}
  static const char * name(State s);

  const std::vector<Vec3> & searchPath() const {return pattern_;}
  int searchPathVersion() const {return pattern_version_;}

private:
  void onWaitConnection(const VehicleState & st);
  void onSetMode(const VehicleState & st, double now);
  void onArm(const VehicleState & st, double now);
  void onTakeoff(const VehicleState & st, double now);
  void onSearch(const VehicleState & st, double now, const std::vector<TargetObs> & obs);
  void onTrack(const VehicleState & st, double now, const std::vector<TargetObs> & obs);
  void onAbort(const VehicleState & st, double now);

  void startSearch(const VehicleState & st, const std::string & why);
  void startLeg(std::size_t i, const Vec3 & from);

  void enter(State s, const std::string & why);
  double inState() const {return now_ - state_t0_;}
  bool requestDue() const;
  void markRequestSent();
  bool controlState() const;    // program riadi (od SET_MODE)
  bool airborneState() const;   // program posiela setpointy vo vzduchu

  Vehicle & vehicle_;
  FollowParams p_;
  Logger log_;
  State state_{State::WAIT_CONNECTION};

  double now_{0.0};
  double state_t0_{0.0};
  double req_sent_t_{-1.0};     // -1 = v tomto stave este nic neposlane
  Vec3 launch_{};               // miesto vzletu (lokalne ENU drona)
  double launch_yaw_{0.0};
  Vec3 hover_{};                // nad miestom vzletu vo vyske search_alt
  double wait_log_t_{-10.0};

  std::vector<Vec3> pattern_;   // body vyhladavacieho vzoru
  int pattern_version_{0};
  std::size_t leg_i_{0};        // index bodu, ku ktoremu dron leti
  StraightLeg leg_;
  double leg_t0_{0.0};

  std::deque<double> confirm_t_;  // casy detekcii v okne confirm_window
  TargetObs last_obs_{};
};

}  // namespace lrs_control
