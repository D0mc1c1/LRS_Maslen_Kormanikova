#pragma once
// Rozsirujuci sa stvorec: ramena d, d, 2d, 2d, 3d, 3d, ... v smeroch +x, +y, -x, -y.
// Susedne pasy su od seba d, d = mensi rozmer zaberu kamery * (1 - prekrytie).
#include <algorithm>
#include <cmath>
#include <vector>

#include "lrs_control/geometry.hpp"

namespace lrs_control
{

struct SearchPatternParams
{
  double altitude{10.0};      // [m] vyska nad zemou (urcuje zaber kamery)
  double overlap{0.3};        // prekrytie susednych pasov 0..0.9
  double max_radius{25.0};    // [m] ziadny roh dalej od stredu (max(|dx|, |dy|))
  double cam_hfov_deg{60.0};  // horizontalne FOV kamery
  double cam_aspect{0.75};    // vyska / sirka obrazu (480 / 640)
};

// zaber kamery na zemi [m]: sirka = 2 h tan(hfov / 2), vyska = sirka * aspect
inline double footprintWidth(const SearchPatternParams & p)
{
  return 2.0 * p.altitude * std::tan(deg2rad(p.cam_hfov_deg) / 2.0);
}

inline double searchSpacing(const SearchPatternParams & p)
{
  const double w = footprintWidth(p);
  const double h = w * p.cam_aspect;
  return std::min(w, h) * (1.0 - std::clamp(p.overlap, 0.0, 0.9));
}

// prvy bod je stred, potom rohy spiraly vo vyske center.z
inline std::vector<Vec3> expandingSquare(const Vec3 & center, const SearchPatternParams & p)
{
  std::vector<Vec3> pts{center};
  const double d = searchSpacing(p);
  if (d <= 0.0) {
    return pts;
  }
  const double dirs[4][2] = {{1.0, 0.0}, {0.0, 1.0}, {-1.0, 0.0}, {0.0, -1.0}};
  double x = center.x, y = center.y;
  for (int i = 0; ; ++i) {
    const double len = (i / 2 + 1) * d;
    const double nx = x + dirs[i % 4][0] * len;
    const double ny = y + dirs[i % 4][1] * len;
    if (std::max(std::abs(nx - center.x), std::abs(ny - center.y)) > p.max_radius + 1e-6) {
      break;
    }
    pts.push_back({nx, ny, center.z});
    x = nx;
    y = ny;
  }
  return pts;
}

}  // namespace lrs_control
