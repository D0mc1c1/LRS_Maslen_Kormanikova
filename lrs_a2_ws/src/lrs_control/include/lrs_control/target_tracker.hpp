#pragma once
#include "lrs_control/geometry.hpp"

namespace lrs_control
{

struct TrackerParams
{
  double alpha{0.5};
  double beta{0.1};
  double gate{3.0};
};

class AlphaBetaTracker
{
public:
  explicit AlphaBetaTracker(TrackerParams p = {})
  : p_(p) {}

  // TODO
  void reset(const Vec3 & z, double t) {(void)z; (void)t;}

  // TODO
  bool update(const Vec3 & z, double t) {(void)z; (void)t; return false;}

  // TODO
  Vec3 predict(double t) const {(void)t; return {};}

  Vec3 velocity() const {return {};}
  double lastUpdate() const {return 0.0;}

private:
  TrackerParams p_;
};

}  // namespace lrs_control
