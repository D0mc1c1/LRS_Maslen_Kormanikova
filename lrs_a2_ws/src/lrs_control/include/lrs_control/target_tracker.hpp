#pragma once
// Alfa-beta filter: odhad polohy a rychlosti roveru z detekcii (len x, y; z = posledna detekcia).
//   predikcia   x_p = x + v dt
//   rezidual    r   = z - x_p
//   oprava      x   = x_p + alpha r,   v = v + (beta / dt) r
// alpha: ako velmi verit novej detekcii, beta: ako rychlo sa prisposobi rychlost.
#include <algorithm>
#include <cmath>

#include "lrs_control/geometry.hpp"

namespace lrs_control
{

struct TrackerParams
{
  double alpha{0.5};
  double beta{0.1};
  double gate{3.0};        // [m] detekcia dalej od predikcie sa zahodi
  double max_speed{3.0};   // [m/s] strop odhadu rychlosti
  int reset_after{5};      // po tolkych zahodenych za sebou sa filter resetuje
};

class AlphaBetaTracker
{
public:
  explicit AlphaBetaTracker(TrackerParams p = {})
  : p_(p) {}

  void reset(const Vec3 & z, double t)
  {
    x_ = z;
    v_ = {};
    t_ = t;
    init_ = true;
    rejects_ = 0;
  }

  // true = detekcia pouzita
  bool update(const Vec3 & z, double t)
  {
    if (!init_) {
      reset(z, t);
      return true;
    }
    const double dt = t - t_;
    if (dt <= 0.0) {
      return false;   // stara alebo duplicitna detekcia
    }
    const Vec3 xp = predict(t);
    const double rx = z.x - xp.x, ry = z.y - xp.y;
    if (std::hypot(rx, ry) > p_.gate) {
      if (++rejects_ >= p_.reset_after) {
        reset(z, t);   // rover je naozaj inde
        return true;
      }
      return false;
    }
    rejects_ = 0;
    const double k = p_.beta / std::max(dt, 0.05);
    x_ = {xp.x + p_.alpha * rx, xp.y + p_.alpha * ry, z.z};
    v_ = {v_.x + k * rx, v_.y + k * ry, 0.0};
    const double s = std::hypot(v_.x, v_.y);
    if (s > p_.max_speed) {
      v_.x *= p_.max_speed / s;
      v_.y *= p_.max_speed / s;
    }
    t_ = t;
    return true;
  }

  Vec3 predict(double t) const
  {
    const double dt = t - t_;
    return {x_.x + v_.x * dt, x_.y + v_.y * dt, x_.z};
  }

  Vec3 velocity() const {return v_;}
  double lastUpdate() const {return t_;}
  bool initialized() const {return init_;}

private:
  TrackerParams p_;
  Vec3 x_{};
  Vec3 v_{};
  double t_{0.0};
  bool init_{false};
  int rejects_{0};
};

}  // namespace lrs_control
