#include "lrs_control/viz.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace lrs_control
{

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{

Marker base(const std::string & ns, int id, int type)
{
  Marker m;
  m.header.frame_id = "map";
  m.ns = ns;
  m.id = id;
  m.type = type;
  m.action = Marker::ADD;
  m.pose.orientation.w = 1.0;
  return m;
}

void color(Marker & m, float r, float g, float b, float a)
{
  m.color.r = r;
  m.color.g = g;
  m.color.b = b;
  m.color.a = a;
}

geometry_msgs::msg::Point pt(double x, double y, double z)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  p.z = z;
  return p;
}

bool insideBox(const lrs_planning::AStarPlanner::Box & b, double x, double y, double z)
{
  return x >= b.lo.x && x <= b.hi.x && y >= b.lo.y && y <= b.hi.y && z >= b.lo.z && z <= b.hi.z;
}

}  // namespace

MarkerArray mapMarkers(
  const lrs_planning::VoxelMap & map,
  const std::vector<lrs_planning::AStarPlanner::Box> & boxes,
  double inflation_radius, double roof_z, double flight_ceiling_z)
{
  MarkerArray arr;

  // --- obsadene voxely, farba podla vysky (modra dole -> zlta hore) ---------
  // steny do roof_z -> "voxely", vsetko nad roof_z -> "strecha" (priesvitna)
  Marker vox = base("voxely", 0, Marker::CUBE_LIST);
  const double v = map.voxelSize();
  vox.scale.x = vox.scale.y = vox.scale.z = v;
  color(vox, 1.0f, 1.0f, 1.0f, 1.0f);
  Marker roof = vox;
  roof.ns = "strecha";
  const auto mn = map.minBound();
  for (std::size_t iz = 0; iz < map.gridZ(); ++iz) {
    const double z = mn.z + (iz + 0.5) * v;
    if (z < 0.15) continue;  // podlaha
    const bool is_roof = z > roof_z;
    const double t = std::clamp(z / roof_z, 0.0, 1.0);
    std_msgs::msg::ColorRGBA c;
    c.r = static_cast<float>(t);
    c.g = static_cast<float>(0.4 + 0.5 * t);
    c.b = static_cast<float>(1.0 - t);
    c.a = is_roof ? 0.15f : 0.9f;
    for (std::size_t iy = 0; iy < map.gridY(); ++iy) {
      const double y = mn.y + (iy + 0.5) * v;
      for (std::size_t ix = 0; ix < map.gridX(); ++ix) {
        const double x = mn.x + (ix + 0.5) * v;
        // len body z mracna (bez boxov regalov a stropu letu); vnutro regalov
        // sa kresli zvlast ako cerveny box
        if (!map.isOccupiedCloud(x, y, z)) continue;
        bool in_box = false;
        for (const auto & b : boxes) {
          if (insideBox(b, x, y, z)) {in_box = true; break;}
        }
        if (in_box) continue;
        Marker & target = is_roof ? roof : vox;
        target.points.push_back(pt(x, y, z));
        target.colors.push_back(c);
      }
    }
  }
  arr.markers.push_back(vox);
  arr.markers.push_back(roof);

  // --- strop letu planovaca: priesvitna modra rovina ------------------------
  if (std::isfinite(flight_ceiling_z)) {
    const auto mx = map.maxBound();
    Marker ceil = base("strop_letu", 0, Marker::CUBE);
    ceil.pose.position = pt((mn.x + mx.x) / 2, (mn.y + mx.y) / 2, flight_ceiling_z);
    ceil.scale.x = mx.x - mn.x;
    ceil.scale.y = mx.y - mn.y;
    ceil.scale.z = 0.02;
    color(ceil, 0.3f, 0.5f, 1.0f, 0.12f);
    arr.markers.push_back(ceil);
  }

  // --- regaly: plny box (cerveny) a inflovany box (priesvitny) ----------------
  int id = 0;
  for (const auto & b : boxes) {
    Marker solid = base("regaly", id++, Marker::CUBE);
    solid.pose.position = pt((b.lo.x + b.hi.x) / 2, (b.lo.y + b.hi.y) / 2, (b.lo.z + b.hi.z) / 2);
    solid.scale.x = b.hi.x - b.lo.x;
    solid.scale.y = b.hi.y - b.lo.y;
    solid.scale.z = b.hi.z - b.lo.z;
    color(solid, 0.9f, 0.2f, 0.2f, 0.45f);
    arr.markers.push_back(solid);

    Marker infl = solid;
    infl.ns = "regaly_inflacia";
    infl.scale.x += 2 * inflation_radius;
    infl.scale.y += 2 * inflation_radius;
    infl.scale.z += 2 * inflation_radius;
    color(infl, 0.9f, 0.4f, 0.2f, 0.12f);
    arr.markers.push_back(infl);
  }
  return arr;
}

MarkerArray planMarkers(const Mission & mission, const std::vector<lrs_planning::Path> & segments)
{
  MarkerArray arr;
  Marker clear;
  clear.action = Marker::DELETEALL;
  clear.header.frame_id = "map";
  arr.markers.push_back(clear);

  // --- waypointy misie: hard = oranzova, soft = tyrkysova ------------------------
  for (std::size_t i = 0; i < mission.size(); ++i) {
    const auto & wp = mission[i];
    Marker s = base("waypointy", static_cast<int>(i), Marker::SPHERE);
    s.pose.position = pt(wp.p.x, wp.p.y, wp.p.z);
    s.scale.x = s.scale.y = s.scale.z = 0.3;
    if (wp.precision == Precision::Hard) {
      color(s, 1.0f, 0.5f, 0.0f, 0.9f);
    } else {
      color(s, 0.0f, 0.8f, 0.9f, 0.9f);
    }
    arr.markers.push_back(s);

    Marker t = base("popisy", static_cast<int>(i), Marker::TEXT_VIEW_FACING);
    t.pose.position = pt(wp.p.x, wp.p.y, wp.p.z + 0.4);
    t.scale.z = 0.25;
    color(t, 1.0f, 1.0f, 1.0f, 1.0f);
    t.text = "WP" + std::to_string(i + 1) + " " + describe(wp);
    arr.markers.push_back(t);
  }

  // --- segmenty z A*: ciara + body po zjednoduseni -------------------------------
  for (std::size_t i = 0; i < segments.size(); ++i) {
    Marker line = base("cesta", static_cast<int>(i), Marker::LINE_STRIP);
    line.scale.x = 0.05;
    color(line, 0.2f, 1.0f, 0.2f, 1.0f);
    Marker dots = base("body_cesty", static_cast<int>(i), Marker::SPHERE_LIST);
    dots.scale.x = dots.scale.y = dots.scale.z = 0.12;
    color(dots, 1.0f, 1.0f, 1.0f, 1.0f);
    for (const auto & p : segments[i]) {
      line.points.push_back(pt(p.x, p.y, p.z));
      dots.points.push_back(pt(p.x, p.y, p.z));
    }
    arr.markers.push_back(line);
    arr.markers.push_back(dots);
  }
  return arr;
}

std::vector<lrs_planning::AStarPlanner::Box> parseBoxes(const std::vector<double> & flat)
{
  if (flat.size() % 6 != 0) {
    throw std::runtime_error("rack_boxes musi mat nasobok 6 cisel");
  }
  std::vector<lrs_planning::AStarPlanner::Box> boxes;
  for (std::size_t i = 0; i < flat.size(); i += 6) {
    boxes.push_back({{flat[i], flat[i + 1], flat[i + 2]}, {flat[i + 3], flat[i + 4], flat[i + 5]}});
  }
  return boxes;
}

}  // namespace lrs_control
