#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


#include "lrs_planning/voxel_map.hpp"
#include "lrs_planning/inflation.hpp"
#include "lrs_planning/planning_algorithm.hpp"

static bool isLineFree(
  const lrs_planning::Vec3& p1,
  const lrs_planning::Vec3& p2,
  const lrs_planning::VoxelMap& map,
  double safety_radius,
  double voxel_size)
{
  // Počet krokov pre interpoláciu
  double dist = std::sqrt(
    (p2.x-p1.x)*(p2.x-p1.x) +
    (p2.y-p1.y)*(p2.y-p1.y) +
    (p2.z-p1.z)*(p2.z-p1.z)
  );
  int steps = static_cast<int>(dist / (voxel_size * 0.5)) + 1;

  
  for (int i = 0; i <= steps; ++i) {
    double t = (steps > 0) ? static_cast<double>(i) / steps : 0.0;
    lrs_planning::Vec3 p{
      p1.x + t * (p2.x - p1.x),
      p1.y + t * (p2.y - p1.y),
      p1.z + t * (p2.z - p1.z)
    };

    if (map.isOccupied(p.x, p.y, p.z, safety_radius)) {
      return false;
    }
  }
  return true;
}


static lrs_planning::Path lineOfSightShortcut(
  const lrs_planning::Path& in_path,
  const lrs_planning::VoxelMap& map,
  double safety_radius,
  double voxel_size)
{
  if (in_path.size() <= 2) {
    return in_path;
  }

  lrs_planning::Path out_path;
  out_path.push_back(in_path[0]);

  size_t i = 0;
  while (i < in_path.size() - 1) {
    size_t best = i + 1;

    for (size_t j = in_path.size() - 1; j > i; --j) {
      if (isLineFree(in_path[i], in_path[j], map, safety_radius, voxel_size)) {
        best = j;
        break;
      }
    }

    if (best != i) {
      out_path.push_back(in_path[best]);
      i = best;
    } else {
      break;
    }
  }

  return out_path;
}


// Kontrola po zjednoduseni: KAZDY usek (nielen body) musi byt volny.
// A* overuje len stredy voxelov; usecka medzi dvoma susednymi stredmi moze
// prejst k prekazke blizsie o max. polovicu uhlopriecky voxelu (voxel*sqrt(3)/2,
// 8.7 cm pri 10 cm voxeli). Kontrola preto pouziva r - tato tolerancia.
static double checkRadius(double r, double voxel) {return r - voxel * std::sqrt(3.0) / 2.0;}

static bool pathIsFree(
  const lrs_planning::Path & path, const lrs_planning::VoxelMap & map,
  double safety_radius, double voxel_size)
{
  for (size_t k = 0; k + 1 < path.size(); ++k) {
    if (!isLineFree(path[k], path[k + 1], map, safety_radius, voxel_size)) {
      return false;
    }
  }
  return true;
}

// Naplanuje jeden segment, zjednodusi ho a vypise vysledok. Vrati true ak OK.
static bool planSegment(
  const lrs_planning::VoxelMap & map, const lrs_planning::Vec3 & start,
  const lrs_planning::Vec3 & goal, double r, double voxel, bool print_points)
{
  lrs_planning::Path path;
  double ms = 0.0;
  const bool found = lrs_planning::astar(map, start, goal, r, voxel, path, ms);
  std::printf("(%.2f, %.2f, %.2f) -> (%.2f, %.2f, %.2f): ",
    start.x, start.y, start.z, goal.x, goal.y, goal.z);
  if (!found) {
    std::printf("NO PATH, %.0f ms\n", ms);
    return false;
  }
  const auto simp = lineOfSightShortcut(path, map, r, voxel);
  const bool safe = pathIsFree(simp, map, checkRadius(r, voxel), voxel);
  std::printf("OK %.0f ms, bodov %zu -> %zu, kontrola usekov (r=%.3f) %s\n",
    ms, path.size(), simp.size(), checkRadius(r, voxel), safe ? "OK" : "ZLYHALA!");
  if (print_points) {
    for (size_t k = 0; k < simp.size(); ++k) {
      std::printf("   %zu: (%.2f, %.2f, %.2f)\n", k, simp[k].x, simp[k].y, simp[k].z);
    }
  }
  return safe;
}

static void usage()
{
  std::cerr <<
    "usage:\n"
    "  plan_cli <map.pcd> <sx> <sy> <sz> <gx> <gy> <gz> [radius=0.5] [voxel=0.1]\n"
    "  plan_cli <map.pcd> --mission <mission.csv> [radius=0.5] [voxel=0.1]\n"
    "  plan_cli <map.pcd> --query <x> <y> <z> [radius=0.5] [voxel=0.1]\n";
}

int main(int argc, char ** argv)
{
  if (argc < 3) {
    usage();
    return 1;
  }
  const std::string mode = argv[2];
  int extra = 0;  // index prveho volitelneho argumentu (radius)
  if (mode == "--mission") {
    extra = 4;
  } else if (mode == "--query") {
    extra = 6;
  } else {
    extra = 8;
  }
  if (argc < extra) {
    usage();
    return 1;
  }
  const double safety_radius = argc > extra ? std::stod(argv[extra]) : 0.5;
  const double voxel = argc > extra + 1 ? std::stod(argv[extra + 1]) : 0.1;

  lrs_planning::VoxelMap map;
  if (!map.load(argv[1], voxel)) {
    std::cerr << "Nepodarilo sa nacitat " << argv[1] << "\n";
    return 1;
  }
  // regaly: obal koliznych boxov z models/fei_lrs_racks/model.sdf
  map.addSolidBox({1.07, 6.29, 0.0}, {7.57, 7.34, 4.10});    // regal A
  map.addSolidBox({1.07, 10.84, 0.0}, {7.57, 11.89, 4.10});  // regal B

  const auto mn = map.minBound();
  const auto mx = map.maxBound();
  std::cout << "=== MAP LOADED ===\n"
            << "points: " << map.pointCount() << "\n"
            << "bbox min: " << mn.x << " " << mn.y << " " << mn.z << "\n"
            << "bbox max: " << mx.x << " " << mx.y << " " << mx.z << "\n"
            << "voxel count: " << map.gridX() * map.gridY() * map.gridZ() << "\n"
            << "grid size: " << map.gridX() << " x " << map.gridY() << " x " << map.gridZ() << "\n"
            << "safety radius: " << safety_radius << " m, voxel: " << voxel << " m\n\n";

  if (mode == "--query") {
    const double x = std::stod(argv[3]), y = std::stod(argv[4]), z = std::stod(argv[5]);
    std::cout << "(" << x << ", " << y << ", " << z << "): bez inflacie "
              << (map.isOccupied(x, y, z, 0.0) ? "OBSADENE" : "VOLNE")
              << ", s infláciou " << safety_radius << " m "
              << (map.isOccupied(x, y, z, safety_radius) ? "OBSADENE" : "VOLNE") << "\n";
    return 0;
  }

  if (mode == "--mission") {
    // CSV x,y,z,precision,task; prvy segment zacina nad spawnom (13, 7)
    std::ifstream f(argv[3]);
    if (!f) {
      std::cerr << "Nepodarilo sa otvorit " << argv[3] << "\n";
      return 1;
    }
    std::vector<lrs_planning::Vec3> wps;
    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#' || line[0] == 'x') continue;
      std::stringstream ss(line);
      double x, y, z;
      char c;
      if (ss >> x >> c >> y >> c >> z) wps.push_back({x, y, z});
    }
    if (wps.empty()) {
      std::cerr << "Misia je prazdna\n";
      return 1;
    }
    lrs_planning::Vec3 prev{13.0, 7.0, wps.front().z};
    int ok = 0;
    for (size_t k = 0; k < wps.size(); ++k) {
      std::printf("\n--- segment %zu -> WP%zu ---\n", k, k + 1);
      if (planSegment(map, prev, wps[k], safety_radius, voxel, true)) ++ok;
      prev = wps[k];
    }
    std::printf("\n=== %d / %zu segmentov OK ===\n", ok, wps.size());
    return ok == static_cast<int>(wps.size()) ? 0 : 2;
  }

  const lrs_planning::Vec3 start{std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4])};
  const lrs_planning::Vec3 goal{std::stod(argv[5]), std::stod(argv[6]), std::stod(argv[7])};
  return planSegment(map, start, goal, safety_radius, voxel, true) ? 0 : 2;
}
