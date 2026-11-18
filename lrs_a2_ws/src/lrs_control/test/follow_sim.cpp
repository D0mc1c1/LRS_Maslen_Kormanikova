// Offline test FollowExecutor bez ROS a bez simulatora.
// usage: follow_sim [takeoff|search|confirm|rc|linkloss]
// TODO: sledovanie s filtrom, strata a opatovne najdenie
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <vector>

#include "lrs_control/follow_executor.hpp"
#include "sim_vehicle.hpp"

using lrs_control::FollowExecutor;
using lrs_control::Vec3;
using State = FollowExecutor::State;

// rohy vzoru: ramena d, d, 2d, 2d, ..., vsetko do max_radius
static bool checkPattern(const lrs_control::FollowParams & p)
{
  lrs_control::SearchPatternParams sp;
  sp.altitude = p.search_alt;
  sp.overlap = p.search_overlap;
  sp.max_radius = p.search_max_radius;
  const Vec3 c{-10.0, -10.0, 10.0};
  const auto pts = lrs_control::expandingSquare(c, sp);
  const double d = lrs_control::searchSpacing(sp);
  std::printf("vzor: %zu bodov, rozostup %.2f m (zaber %.2f x %.2f m)\n", pts.size(), d,
    lrs_control::footprintWidth(sp), lrs_control::footprintWidth(sp) * sp.cam_aspect);
  bool ok = pts.size() >= 5 && std::abs(pts[0].x - c.x) < 1e-9 && std::abs(pts[0].y - c.y) < 1e-9;
  for (std::size_t i = 1; i < pts.size(); ++i) {
    const double len = lrs_control::distXY(pts[i - 1], pts[i]);
    const double want = ((i - 1) / 2 + 1) * d;
    const double r = std::max(std::abs(pts[i].x - c.x), std::abs(pts[i].y - c.y));
    std::printf("  roh %2zu  x %6.2f  y %6.2f  rameno %5.2f m\n", i, pts[i].x, pts[i].y, len);
    ok = ok && std::abs(len - want) < 1e-6 && r <= sp.max_radius + 1e-6 &&
      std::abs(pts[i].z - c.z) < 1e-9;
  }
  return ok;
}

int main(int argc, char ** argv)
{
  const std::string sc = argc > 1 ? argv[1] : "takeoff";
  if (sc != "takeoff" && sc != "search" && sc != "confirm" && sc != "rc" && sc != "linkloss") {
    std::printf("usage: follow_sim [takeoff|search|confirm|rc|linkloss]\n");
    return 1;
  }

  lrs_control::SimVehicle veh({-10.0, -10.0, 0.0});
  lrs_control::FollowParams p;
  bool loop_done = false;
  FollowExecutor ex(veh, p, [&](const std::string & m) {
      if (m.find("vzor prejdeny") != std::string::npos) {loop_done = true;}
      if (sc != "search" || m.find("roh") == std::string::npos) {
        std::printf("[t=%6.2f] %s\n", veh.t(), m.c_str());
      }
    });

  bool ok = true;
  if (sc == "search") {
    ok = checkPattern(p);
  }

  const double dt = 0.05;
  // rover ako vo fake_target: kruznica r 3 m, 0.3 m/s, stred start + (10, 8)
  const Vec3 rc{0.0, -2.0, 0.0};
  auto rover = [&rc](double t) {
      const double a = 0.3 / 3.0 * t;
      return Vec3{rc.x + 3.0 * std::cos(a), rc.y + 3.0 * std::sin(a), 0.0};
    };
  lrs_control::SearchPatternParams cam;
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.1);
  double track_t = -1.0, err_sum = 0.0;
  int err_n = 0;
  bool spurious_ok = true;

  double search_t = -1.0;
  Vec3 at_search{};
  bool injected = false;
  double max_err = 0.0, max_r = 0.0;
  std::vector<double> corner_min;
  while (veh.t() < 400.0) {
    veh.step(dt);
    std::vector<lrs_control::TargetObs> obs;
    const auto st0 = veh.state();
    const int k = static_cast<int>(std::lround(veh.t() / dt));
    if (sc == "confirm" && search_t > 0.0 && k % 2 == 0) {   // kamera 10 Hz
      const Vec3 r = rover(veh.t());
      const Vec3 rel{r.x - st0.pos.x, r.y - st0.pos.y, r.z - st0.pos.z};
      if (lrs_control::inFootprint(rel, st0.yaw, st0.pos.z, cam)) {
        obs.push_back({veh.t(), {r.x + noise(rng), r.y + noise(rng), 0.0}});
      }
      // jedna falosna detekcia 2 s po zaciatku hladania nesmie spustit sledovanie
      if (std::abs(veh.t() - (search_t + 2.0)) < 0.5 * dt) {
        obs.push_back({veh.t(), {-30.0, 20.0, 0.0}});
      }
    }
    ex.tick(veh.t(), obs);
    const auto st = veh.state();
    if (sc == "confirm" && search_t > 0.0 && veh.t() < search_t + 3.0 &&
      ex.state() == State::TRACK)
    {
      spurious_ok = false;
    }
    if (sc == "confirm" && ex.state() == State::TRACK) {
      if (track_t < 0.0) {track_t = veh.t();}
      if (veh.t() > track_t + 20.0) {   // po ustaleni
        err_sum += lrs_control::distXY(st.pos, rover(veh.t()));
        ++err_n;
      }
      if (veh.t() > track_t + 60.0) {break;}
    }
    if (ex.state() == State::SEARCH && search_t < 0.0) {
      search_t = veh.t();
      at_search = st.pos;
      corner_min.assign(ex.searchPath().size(), 1e9);
      if (sc == "takeoff") {break;}
    }
    if (ex.state() == State::SEARCH && sc == "search" && veh.setpoints() > 0) {
      const Vec3 sp = veh.lastSetpoint().p;
      max_err = std::max(max_err, lrs_control::dist(sp, st.pos));
      max_r = std::max(max_r, std::max(std::abs(st.pos.x - at_search.x),
        std::abs(st.pos.y - at_search.y)));
      for (std::size_t i = 0; i < corner_min.size(); ++i) {
        corner_min[i] = std::min(corner_min[i], lrs_control::dist(st.pos, ex.searchPath()[i]));
      }
      if (loop_done) {break;}
    }
    // 5 s po vzlete zasah pilota alebo strata spojenia
    if (search_t > 0.0 && veh.t() > search_t + 5.0 && !injected) {
      injected = true;
      if (sc == "rc") {veh.forceMode("LOITER");}
      if (sc == "linkloss") {veh.dropLink();}
    }
    if (ex.state() == State::ABORT) {break;}
  }

  const auto st = veh.state();
  std::printf("koniec: t %.1f s, stav %s, poloha %.2f %.2f %.2f\n", veh.t(),
    FollowExecutor::name(ex.state()), st.pos.x, st.pos.y, st.pos.z);

  if (sc == "takeoff") {
    ok = ex.state() == State::SEARCH && std::abs(st.pos.z - p.search_alt) < 0.3 &&
      std::abs(st.pos.x + 10.0) < 0.2 && std::abs(st.pos.y + 10.0) < 0.2;
  } else if (sc == "search") {
    const double worst = corner_min.empty() ? 1e9 :
      *std::max_element(corner_min.begin(), corner_min.end());
    std::printf("vzor preleteny za %.1f s, najvacsia odchylka od setpointu %.2f m, "
      "dosah %.1f m, najhorsi roh %.2f m\n", veh.t() - search_t, max_err, max_r, worst);
    ok = ok && loop_done && ex.state() == State::SEARCH && max_err < 1.0 && worst < 0.5 &&
      std::abs(st.pos.z - p.search_alt) < 0.3;
  } else if (sc == "confirm") {
    const double err = err_n > 0 ? err_sum / err_n : 1e9;
    std::printf("najdeny za %.1f s hladania, falosna detekcia %s, priemerna odchylka nad roverom "
      "%.2f m\n", track_t - search_t, spurious_ok ? "ignorovana" : "SPUSTILA SLEDOVANIE", err);
    ok = track_t > 0.0 && spurious_ok && ex.state() == State::TRACK && err < 1.0 &&
      std::abs(st.pos.z - p.follow_alt) < 0.3;
  } else {
    ok = injected && ex.state() == State::ABORT;
  }
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
