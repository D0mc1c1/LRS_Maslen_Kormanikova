#pragma once
// SimVehicle: hruby kinematicky model drona pre offline testy (mission_sim, outdoor_sim).
// Sprava sa ako ArduPilot v podstatnych veciach:
//  - arm len v GUIDED, takeoff len ked je armnuty
//  - pozicny setpoint je latched, rychlost a zrychlenie su obmedzene
//  - LAND: klesa a po dosadnuti sa SAM disarmne (presne ako ArduPilot)
//  - volitelne GPS / home / kompas (enableGps) pre A1.3
#include <algorithm>
#include <cmath>
#include <functional>
#include <string>

#include "lrs_control/geo.hpp"
#include "lrs_control/geometry.hpp"
#include "lrs_control/vehicle.hpp"

namespace lrs_control
{

class SimVehicle : public Vehicle
{
public:
  explicit SimVehicle(const Vec3 & spawn) {s_.pos = spawn; s_.mode = "STABILIZE";}

  // GPS model (A1.3): lat/lon sa pocita z polohy cez LocalTangentPlane okolo `origin`.
  // gps_after = kedy pride prvy fix [s], home_after = kedy pride home (< 0 = nikdy)
  void enableGps(const LatLonAlt & origin, double gps_after, double home_after)
  {
    gps_on_ = true;
    ltp_ = LocalTangentPlane(origin);
    gps_after_ = gps_after;
    home_after_ = home_after;
  }

  // zasah pilota z RC (prepnutie modu mimo programu)
  void forceMode(const std::string & m) {s_.mode = m;}

  VehicleState state() const override {return s_;}

  void requestMode(const std::string & m) override {pend("mode", [this, m] {s_.mode = m; return true;});}
  void requestArm(bool a) override
  {
    pend("arm", [this, a] {
        if (s_.mode != "GUIDED") {return false;}
        s_.armed = a;
        armed_t_ = t_;
        return true;
      });
  }
  void requestTakeoff(double z) override
  {
    pend("takeoff", [this, z] {
        if (!s_.armed || s_.mode != "GUIDED" || airborne_) {return false;}
        target_ = {s_.pos.x, s_.pos.y, z};
        v_ff_ = {};
        yaw_rate_ff_ = 0.0;
        airborne_ = true;
        return true;
      });
  }
  void requestLand() override {pend("land", [this] {s_.mode = "LAND"; return true;});}
  RequestStatus requestStatus() const override {return status_;}

  void sendPositionTarget(const Vec3 & p, double yaw) override
  {
    TrajectorySetpoint sp;
    sp.p = p;
    sp.yaw = yaw;
    sendTrajectorySetpoint(sp);
  }
  // rychlost a yaw_rate = feedforward, P regulator dorovnava len odchylku
  void sendTrajectorySetpoint(const TrajectorySetpoint & sp) override
  {
    if (s_.mode == "GUIDED" && s_.armed && airborne_) {
      target_ = sp.p;
      v_ff_ = sp.v.value_or(Vec3{});
      yaw_target_ = sp.yaw;
      yaw_rate_ff_ = sp.yaw_rate.value_or(0.0);
    }
    ++setpoints_;
  }

  void step(double dt)
  {
    t_ += dt;
    s_.connected = t_ > 0.5;
    s_.has_pose = t_ > 0.5;
    if (pending_ && t_ >= pending_t_) {   // odpoved sluzby po 0.2 s
      status_ = pending_fn_() ? RequestStatus::Ok : RequestStatus::Failed;
      pending_ = false;
    }
    // ArduPilot: armnuty na zemi bez vzletu -> po 10 s disarm
    if (s_.armed && !airborne_ && t_ - armed_t_ > 10.0) {s_.armed = false;}

    Vec3 v_des{0, 0, 0};
    if (s_.mode == "LAND" && s_.armed) {
      v_des = {0, 0, -0.5};
      if (s_.pos.z <= 0.01) {
        s_.pos.z = 0;
        land_t_ += dt;
        if (land_t_ > 1.0) {s_.armed = false; airborne_ = false; land_t_ = 0;}
      }
    } else if (s_.mode == "GUIDED" && airborne_) {
      // P regulator polohy s obmedzenim rychlosti (vmax 1.5 m/s)
      const double k = 1.2, vmax = 1.5;
      v_des = {v_ff_.x + k * (target_.x - s_.pos.x), v_ff_.y + k * (target_.y - s_.pos.y),
        v_ff_.z + k * (target_.z - s_.pos.z)};
      const double n = norm(v_des);
      if (n > vmax) {v_des = {v_des.x * vmax / n, v_des.y * vmax / n, v_des.z * vmax / n};}
      // yaw: P regulator (ako ATC_ANG_YAW_P ~4) + feedforward, max 60 deg/s.
      // Bez feedforward zaostava za otacajucim sa cielom o (rychlost otacania / 4).
      const double e = angleDiff(yaw_target_, s_.yaw);
      const double rate = std::clamp(yaw_rate_ff_ + 4.0 * e, -deg2rad(60), deg2rad(60));
      s_.yaw = wrapPi(s_.yaw + rate * dt);
    }
    // obmedzenie zrychlenia 2 m/s^2
    const double amax = 2.0 * dt;
    auto lim = [&](double v, double d) {return v + std::clamp(d - v, -amax, amax);};
    s_.vel = {lim(s_.vel.x, v_des.x), lim(s_.vel.y, v_des.y), lim(s_.vel.z, v_des.z)};
    s_.pos = {s_.pos.x + s_.vel.x * dt, s_.pos.y + s_.vel.y * dt,
      std::max(0.0, s_.pos.z + s_.vel.z * dt)};

    // GPS, home, kompas (ako MAVROS v SITL)
    if (gps_on_) {
      s_.has_gps = t_ > 0.5;
      s_.gps_status = t_ > gps_after_ ? 0 : -1;
      s_.h_acc = t_ > gps_after_ ? 0.3 : 5.0;
      s_.gps = ltp_.toLla(s_.pos);
      s_.has_rel_alt = true;
      s_.rel_alt = s_.pos.z;
      s_.has_compass = true;
      s_.compass_hdg_deg = enuYawToCompassDeg(rad2deg(s_.yaw));
      if (home_after_ >= 0.0 && t_ > home_after_ && !s_.has_home) {
        s_.has_home = true;
        s_.home = ltp_.origin();
      }
    }
  }

  long setpoints() const {return setpoints_;}
  double t() const {return t_;}

private:
  template<class F>
  void pend(const char *, F fn)
  {
    pending_ = true;
    pending_t_ = t_ + 0.2;
    pending_fn_ = fn;
    status_ = RequestStatus::Pending;
  }

  VehicleState s_;
  double t_{0}, armed_t_{0}, land_t_{0}, pending_t_{0};
  bool airborne_{false}, pending_{false};
  std::function<bool()> pending_fn_;
  RequestStatus status_{RequestStatus::None};
  Vec3 target_{}, v_ff_{};
  double yaw_target_{0}, yaw_rate_ff_{0};
  long setpoints_{0};
  bool gps_on_{false};
  LocalTangentPlane ltp_;
  double gps_after_{0}, home_after_{-1};
};

}  // namespace lrs_control
