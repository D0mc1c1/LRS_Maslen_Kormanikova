#pragma once
// A1.3: casove trajektorie pre outdoor misiu. Bez ROS, header-only.
//
// Trapezoid  : rychlostny profil 0 -> v_max -> 0 s obmedzenym zrychlenim
// StraightLeg: priamy usek a -> b s trapezoidovym profilom; vracia polohu
//              aj rychlost v case t (rychlost ide do setpointu ako feedforward)
// Figure8    : osmicka x = A sin t, y = A sin t cos t s tangencialnym yaw
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

// Osmicka (Geronova lemniskata) okolo stredu c, v rovine vo vyske c.z:
//
//   x(th) = A sin(th)
//   y(th) = A sin(th) cos(th)          (potom otocene o `rot` okolo c, ENU)
//
// Faza th(t) je casovy profil: rozbeh za `ramp` s na th' = omega, konstantne omega,
// dobeh na th = 2*pi*laps. Pri ramp = 0 je to presne th = omega * t.
// Rychlost   |v| = A * th' * sqrt(cos^2 th + cos^2 2th),  max A * omega * sqrt(2) v strede.
// Tangenta   yaw = atan2(cos 2th, cos th) + rot  (derivacie podla th, takze je
//            definovana aj pri th' = 0; cos th a cos 2th nie su nikdy naraz 0).
struct Figure8
{
  Vec3 c{};
  double A{3.0};
  double rot{0.0};
  Trapezoid phase;

  Figure8() = default;
  Figure8(const Vec3 & center, double amplitude, double omega, double laps, double ramp,
    double rotation)
  : c(center), A(amplitude), rot(rotation),
    phase(2.0 * kPi * laps, omega, ramp > 0.0 ? omega / ramp : 1e9) {}

  double duration() const {return phase.duration();}
  double omega() const {return phase.v_max;}
  double theta(double t) const {return phase.s(t);}
  double maxSpeed() const {return A * omega() * std::sqrt(2.0);}

  Vec3 posAt(double th) const
  {
    return rotAdd(A * std::sin(th), A * std::sin(th) * std::cos(th), c.z);
  }
  // derivacia podla th (smer dotycnice, bez th')
  Vec3 dPosAt(double th) const
  {
    const double dx = A * std::cos(th), dy = A * std::cos(2.0 * th);
    const double cr = std::cos(rot), sr = std::sin(rot);
    return {cr * dx - sr * dy, sr * dx + cr * dy, 0.0};
  }
  double tangentYawAt(double th) const
  {
    return wrapPi(std::atan2(std::cos(2.0 * th), std::cos(th)) + rot);
  }
  // d(yaw)/d(th) = (f g' - g f') / (f^2 + g^2),  f = cos th, g = cos 2th
  double dYawAt(double th) const
  {
    const double f = std::cos(th), g = std::cos(2.0 * th);
    const double df = -std::sin(th), dg = -2.0 * std::sin(2.0 * th);
    return (f * dg - g * df) / (f * f + g * g);
  }

  Vec3 pos(double t) const {return posAt(theta(t));}
  Vec3 vel(double t) const
  {
    const Vec3 d = dPosAt(theta(t));
    const double w = phase.v(t);
    return {d.x * w, d.y * w, 0.0};
  }
  double yaw(double t) const {return tangentYawAt(theta(t));}
  double yawRate(double t) const {return dYawAt(theta(t)) * phase.v(t);}

private:
  Vec3 rotAdd(double x, double y, double z) const
  {
    const double cr = std::cos(rot), sr = std::sin(rot);
    return {c.x + cr * x - sr * y, c.y + sr * x + cr * y, z};
  }
};

// Horizontalny smer (ENU yaw) z bodu p na bod q. Vyska sa ignoruje:
// POI je na zemi, dron vo vyske 5 m, otacame len yaw (nie gimbal).
inline double bearingXY(const Vec3 & p, const Vec3 & q)
{
  return std::atan2(q.y - p.y, q.x - p.x);
}

}  // namespace lrs_control
