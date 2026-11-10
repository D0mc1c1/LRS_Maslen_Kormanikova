#pragma once
#include <vector>

namespace lrs_planning
{

struct Vec3 { double x{0}, y{0}, z{0}; };   // x, y, z v metroch
struct Index3 { int x{0}, y{0}, z{0}; };    // indexy bunky
using Path = std::vector<Vec3>;             // zoznam 3D bodov

}
