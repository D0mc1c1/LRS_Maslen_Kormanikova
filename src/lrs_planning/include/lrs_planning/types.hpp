#pragma once
#include <vector>

namespace lrs_planning
{

struct Vec3 { double x{0}, y{0}, z{0}; };
struct Index3 { int x{0}, y{0}, z{0}; };
using Path = std::vector<Vec3>;

}  // namespace lrs_planning
