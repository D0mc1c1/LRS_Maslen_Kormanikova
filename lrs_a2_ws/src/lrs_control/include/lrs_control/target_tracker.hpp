#pragma once
// Alfa-beta filter: odhad polohy a rychlosti roveru z detekcii (len x, y; z = posledna detekcia).
//   predikcia   x_p = x + v dt
//   rezidual    r   = z - x_p
//   oprava      x   = x_p + alpha r,   v = v + (beta / dt) r
// alpha: ako velmi verit novej detekcii, beta: ako rychlo sa prisposobi rychlost.
//
// alpha a beta su zadane pre detekcie kazdych dt_ref = 0.1 s (10 Hz). Pri castejsich
// detekciach sa zosilnenia zmensia tak, aby filter za sekundu urobil rovnako velku opravu:
//   alpha_dt = 1 - (1 - alpha)^(dt / dt_ref),   oprava rychlosti = beta * dt / dt_ref^2 * r
// Inak by detektor s 28 Hz opravoval 2.8x castejsie a odhad rychlosti by bol zasumeny.
#include <algorithm>
#include <cmath>

#include "lrs_control/geometry.hpp"

namespace lrs_control
{

struct TrackerParams
{
  double alpha{0.3};
  double beta{0.02};
  double gate{3.0};        // [m] detekcia dalej od predikcie sa zahodi
  double max_speed{1.5};   // [m/s] strop odhadu rychlosti roveru
  double dt_ref{0.1};      // [s] pre tento interval detekcii su zadane alpha a beta
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
    // po dlhsej medzere nie vacsie zosilnenie ako pri 10 Hz
    const double f = std::min(dt, p_.dt_ref) / p_.dt_ref;
    const double a = 1.0 - std::pow(1.0 - p_.alpha, f);
    const double k = p_.beta * f / p_.dt_ref;
    x_ = {xp.x + a * rx, xp.y + a * ry, z.z};
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
