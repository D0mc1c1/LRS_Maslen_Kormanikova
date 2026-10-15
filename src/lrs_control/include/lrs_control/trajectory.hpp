#pragma once
// A1.3: casove trajektorie pre outdoor misiu. Bez ROS, header-only.
//
// Trapezoid  : rychlostny profil 0 -> v_max -> 0 s obmedzenym zrychlenim
// StraightLeg: priamy usek a -> b s trapezoidovym profilom; vracia polohu
//              aj rychlost v case t (rychlost ide do setpointu ako feedforward)
#include <algorithm>
#include <cmath>

#include "lrs_control/geometry.hpp"

namespace lrs_control
{

// Prejdena drahu s(t) na useku dlzky len: zrychlenie a, konstantna v_max, spomalenie a.
// Ak je usek kratky, v_max sa nedosiahne (trojuholnikovy profil).
struct Trapezoid
{
  double len{0.0}, v_max{1.0}, a_max{0.5};

  Trapezoid() = default;
  Trapezoid(double l, double v, double a)
  : len(std::max(0.0, l)), v_max(v), a_max(a)
  {
    t_acc_ = v_max / a_max;
    if (a_max * t_acc_ * t_acc_ > len) {        // trojuholnik
      t_acc_ = std::sqrt(len / a_max);
      v_peak_ = a_max * t_acc_;
      t_cruise_ = 0.0;
    } else {
      v_peak_ = v_max;
      t_cruise_ = (len - a_max * t_acc_ * t_acc_) / v_max;
    }
  }

  double duration() const {return 2.0 * t_acc_ + t_cruise_;}

  double s(double t) const
  {
    if (t <= 0.0) {return 0.0;}
    if (t >= duration()) {return len;}
    if (t < t_acc_) {return 0.5 * a_max * t * t;}
    const double d_acc = 0.5 * a_max * t_acc_ * t_acc_;
    if (t < t_acc_ + t_cruise_) {return d_acc + v_peak_ * (t - t_acc_);}
    const double td = duration() - t;            // cas do konca
    return len - 0.5 * a_max * td * td;
  }

  double v(double t) const
  {
    if (t <= 0.0 || t >= duration()) {return 0.0;}
    if (t < t_acc_) {return a_max * t;}
    if (t < t_acc_ + t_cruise_) {return v_peak_;}
    return a_max * (duration() - t);
  }

private:
  double t_acc_{0.0}, t_cruise_{0.0}, v_peak_{0.0};
};

struct StraightLeg
{
  Vec3 a{}, b{};
  Trapezoid prof;

  StraightLeg() = default;
  StraightLeg(const Vec3 & from, const Vec3 & to, double v_max, double a_max)
  : a(from), b(to), prof(dist(from, to), v_max, a_max) {}

  double duration() const {return prof.duration();}
  double length() const {return prof.len;}

  Vec3 dir() const
  {
    const double l = prof.len;
    return l > 1e-9 ? Vec3{(b.x - a.x) / l, (b.y - a.y) / l, (b.z - a.z) / l} : Vec3{};
  }
  Vec3 pos(double t) const
  {
    const double s = prof.s(t);
    const Vec3 d = dir();
    return {a.x + d.x * s, a.y + d.y * s, a.z + d.z * s};
  }
  Vec3 vel(double t) const
  {
    const double v = prof.v(t);
    const Vec3 d = dir();
    return {d.x * v, d.y * v, d.z * v};
  }
};

// Horizontalny smer (ENU yaw) z bodu p na bod q. Vyska sa ignoruje:
// POI je na zemi, dron vo vyske 5 m, otacame len yaw (nie gimbal).
inline double bearingXY(const Vec3 & p, const Vec3 & q)
{
  return std::atan2(q.y - p.y, q.x - p.x);
}

}  // namespace lrs_control
