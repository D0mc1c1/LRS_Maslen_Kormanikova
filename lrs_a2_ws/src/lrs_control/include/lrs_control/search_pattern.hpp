#pragma once
#include <vector>

#include "lrs_control/geometry.hpp"

namespace lrs_control
{

struct SearchPatternParams
{
  double altitude{10.0};
  double overlap{0.3};
  double max_radius{25.0};
  double cam_hfov_deg{60.0};
  double cam_aspect{0.75};
};

// TODO: rozsirujuci sa stvorec
inline std::vector<Vec3> expandingSquare(const Vec3 & center, const SearchPatternParams & p)
{
  (void)p;
  return {center};
}

}  // namespace lrs_control
