#pragma once
// Casove trajektorie pre outdoor misiu bez zavislosti na ROS.
// Vsetky vypocty su dostupne priamo v hlavickovom subore.
//
// Trapezoid  : rychlostny profil so zrychlenim, konstantnou rychlostou a spomalenim.
// StraightLeg: priamy usek medzi bodmi s vypoctom polohy a rychlosti.
// Figure8    : trajektoria v tvare osmicky s natocenim podla dotycnice.

#include <algorithm>
#include <cmath>

#include "lrs_control/geometry.hpp"

namespace lrs_control
{

// Vypocet prejdenej drahy a rychlosti podla trapezoveho profilu.
// Pri kratkom useku sa maximalna rychlost nedosiahne a vznikne trojuholnikovy profil.
struct Trapezoid
{
  double len{0.0}, v_max{1.0}, a_max{0.5};

  Trapezoid() = default;
  Trapezoid(double l, double v, double a)
  : len(std::max(0.0, l)), v_max(v), a_max(a)
  {
    t_acc_ = v_max / a_max;
    if (a_max * t_acc_ * t_acc_ > len) {        // Trojuholnikovy profil
      t_acc_ = std::sqrt(len / a_max);
      v_peak_ = a_max * t_acc_;
      t_cruise_ = 0.0;
    } else {
      v_peak_ = v_max;
      t_cruise_ = (len - a_max * t_acc_ * t_acc_) / v_max;
    }
  }

  // Vrati celkovy cas prejazdu useku.
  double duration() const {return 2.0 * t_acc_ + t_cruise_;}

  // Vrati prejdenu drahu v case t.
  double s(double t) const
  {
    if (t <= 0.0) {return 0.0;}
    if (t >= duration()) {return len;}
    if (t < t_acc_) {return 0.5 * a_max * t * t;}
    const double d_acc = 0.5 * a_max * t_acc_ * t_acc_;
    if (t < t_acc_ + t_cruise_) {return d_acc + v_peak_ * (t - t_acc_);}
    const double td = duration() - t;            // Zostavajuci cas do konca
    return len - 0.5 * a_max * td * td;
  }

  // Vrati okamzitu rychlost v case t.
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

// Priamy usek medzi bodmi a a b s trapezovym rychlostnym profilom.
// Umoznuje vypocitat polohu a rychlost v lubovolnom case.
struct StraightLeg
{
  Vec3 a{}, b{};
  Trapezoid prof;

  StraightLeg() = default;
  StraightLeg(const Vec3 & from, const Vec3 & to, double v_max, double a_max)
  : a(from), b(to), prof(dist(from, to), v_max, a_max) {}

  // Vrati cas potrebny na prejdenie useku.
  double duration() const {return prof.duration();}

  // Vrati dlzku useku v metroch.
  double length() const {return prof.len;}

  // Vypocita jednotkovy vektor smeru pohybu.
  Vec3 dir() const
  {
    const double l = prof.len;
    return l > 1e-9 ? Vec3{(b.x - a.x) / l, (b.y - a.y) / l, (b.z - a.z) / l} : Vec3{};
  }

  // Vrati polohu dronu v case t.
  Vec3 pos(double t) const
  {
    const double s = prof.s(t);
    const Vec3 d = dir();
    return {a.x + d.x * s, a.y + d.y * s, a.z + d.z * s};
  }

  // Vrati vektor rychlosti v case t.
  Vec3 vel(double t) const
  {
    const double v = prof.v(t);
    const Vec3 d = dir();
    return {d.x * v, d.y * v, d.z * v};
  }
};

// Trajektoria v tvare osmicky (Geronova lemniskata) okolo stredu c.
// Pohyb prebieha v horizontalnej rovine vo vyske c.z.
// Parameter A urcuje velkost osmicky a rot jej natocenie.
//
// x(th) = A * sin(th)
// y(th) = A * sin(th) * cos(th)
//
// Faza th sa meni podla casoveho profilu s rozbehom a dobehom.
// Parameter omega urcuje maximalnu uhlovu rychlost a laps pocet obehov.
// Pri ramp = 0 sa faza meni konstantnou rychlostou.
//
// Velkost rychlosti zavisi od fazy a uhlovej rychlosti.
// Smer yaw sleduje dotycnicu trajektorie, aj ked dron prave stoji.
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

  // Vrati celkovy cas trvania trajektorie.
  double duration() const {return phase.duration();}

  // Vrati maximalnu uhlovu rychlost.
  double omega() const {return phase.v_max;}

  // Vrati fazu trajektorie v case t.
  double theta(double t) const {return phase.s(t);}

  // Vrati maximalnu rychlost pohybu po trajektorii.
  double maxSpeed() const {return A * omega() * std::sqrt(2.0);}

  // Vypocita polohu bodu na osmicke pre zadanu fazu.
  Vec3 posAt(double th) const
  {
    return rotAdd(A * std::sin(th), A * std::sin(th) * std::cos(th), c.z);
  }

  // Vypocita derivaciu polohy podla fazy, teda smer dotycnice.
  Vec3 dPosAt(double th) const
  {
    const double dx = A * std::cos(th), dy = A * std::cos(2.0 * th);
    const double cr = std::cos(rot), sr = std::sin(rot);
    return {cr * dx - sr * dy, sr * dx + cr * dy, 0.0};
  }

  // Vypocita yaw podla smeru dotycnice trajektorie.
  double tangentYawAt(double th) const
  {
    return wrapPi(std::atan2(std::cos(2.0 * th), std::cos(th)) + rot);
  }

  // Vypocita zmenu yaw podla fazy trajektorie.
  double dYawAt(double th) const
  {
    const double f = std::cos(th), g = std::cos(2.0 * th);
    const double df = -std::sin(th), dg = -2.0 * std::sin(2.0 * th);
    return (f * dg - g * df) / (f * f + g * g);
  }

  // Vrati polohu dronu v case t.
  Vec3 pos(double t) const {return posAt(theta(t));}

  // Vrati vektor rychlosti dronu v case t.
  Vec3 vel(double t) const
  {
    const Vec3 d = dPosAt(theta(t));
    const double w = phase.v(t);
    return {d.x * w, d.y * w, 0.0};
  }

  // Vrati smer natocenia dronu v case t.
  double yaw(double t) const {return tangentYawAt(theta(t));}

  // Vrati uhlovu rychlost otacania v case t.
  double yawRate(double t) const {return dYawAt(theta(t)) * phase.v(t);}

private:
  // Otoci bod okolo stredu c a prida jeho suradnice.
  Vec3 rotAdd(double x, double y, double z) const
  {
    const double cr = std::cos(rot), sr = std::sin(rot);
    return {c.x + cr * x - sr * y, c.y + sr * x + cr * y, z};
  }
};

// Vypocita horizontalny smer ENU z bodu p do bodu q.
// Vyska sa ignoruje, pretoze sa meni iba yaw, nie naklon gimbalu.
inline double bearingXY(const Vec3 & p, const Vec3 & q)
{
  return std::atan2(q.y - p.y, q.x - p.x);
}

}