// Offline test FollowExecutor bez ROS a bez simulatora.
// usage: follow_sim [takeoff|search|confirm|track [naive]|lost|abort [LOITER|BRAKE|RTL]|
//                    gazebo [+|-]|rc|linkloss]
// gazebo: detektor ako v Gazebe (seed 42): 28 Hz, sum 4 cm a chyba z naklonu drona, ktory
// MAVROS hlasi neskor ako ho vidi kamera (z rosbagu ~15 ms, test 50 ms = prisnejsie);
// rover ako randomize_rovers.py. "-" = opacne znamienko chyby (stabilnejsi pripad).
// Na ladenie: premenne ALPHA, BETA, LEAD, MAXV prepisu parametre sledovania, TAU oneskorenie.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <numeric>
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
  if (sc != "takeoff" && sc != "search" && sc != "confirm" && sc != "track" && sc != "lost" &&
    sc != "abort" && sc != "gazebo" && sc != "rc" && sc != "linkloss")
  {
    std::printf("usage: follow_sim [takeoff|search|confirm|track [naive]|lost|"
      "abort [LOITER|BRAKE|RTL]|gazebo [+|-]|rc|linkloss]\n");
    return 1;
  }

  lrs_control::SimVehicle veh({-10.0, -10.0, 0.0});
  lrs_control::FollowParams p;
  const bool naive = argc > 2 && std::string(argv[2]) == "naive";
  if (naive) {   // bez filtra: mieri na poslednu detekciu
    p.track_alpha = 1.0;
    p.track_beta = 0.0;
    p.track_lead = 0.0;
  }
  if (const char * e = std::getenv("ALPHA")) {p.track_alpha = std::atof(e);}
  if (const char * e = std::getenv("BETA")) {p.track_beta = std::atof(e);}
  if (const char * e = std::getenv("LEAD")) {p.track_lead = std::atof(e);}
  if (const char * e = std::getenv("MAXV")) {p.track_max_target_speed = std::atof(e);}
  if (sc == "abort" && argc > 2) {
    p.abort_mode = argv[2];
  }
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

  // track: po 20 s sledovania rover zmeni pohyb na priamky 0.5 m/s s nahodnymi zakrutami
  // kazdych 8 s; detekcie chodia s oneskorenim 0.1 s (cas = cas snimky)
  Vec3 rpos = rover(0.0);
  double heading = 0.0, next_turn = 0.0;
  bool walk = false;
  std::uniform_real_distribution<double> turn(-1.6, 1.6);
  std::deque<lrs_control::TargetObs> pending;
  double se = 0.0, emax = 0.0;
  int ne = 0, frames = 0, seen = 0;
  const bool moving = sc == "track" || sc == "lost";

  // lost: rover sa 2x na 6 s skryje (30 a 70 s po prvom najdeni)
  auto hidden = [&](double t) {
      return sc == "lost" && track_t > 0.0 &&
             ((t >= track_t + 30.0 && t < track_t + 36.0) || (t >= track_t + 70.0 && t < track_t + 76.0));
    };
  // gazebo: rover ako randomize_rovers.py (0.15-0.4 m/s, +-0.2 rad/s, nova hodnota kazdych
  // 3-6 s, rampa 0.2 m/s^2, do 4 m od stredu), detektor 28 Hz so sumom a chybou z naklonu
  const double tilt_sign = (argc > 2 && std::string(argv[2]) == "-") ? -1.0 : 1.0;
  const double tilt_delay = std::getenv("TAU") ? std::atof(std::getenv("TAU")) : 0.05;
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  double g_speed = 0.3, g_rate = 0.0, g_cmd_speed = 0.3, g_cmd_rate = 0.0, g_next = 0.0;
  double g_yaw = 0.0, g_next_det = 0.0;
  std::normal_distribution<double> g_noise(0.0, 0.04);
  std::deque<std::pair<double, Vec3>> acc_hist;   // (cas, zrychlenie drona)
  Vec3 prev_vel{}, tilt{};   // tilt = naklon [rad] v smere x, y (ENU)
  long ff_n = 0, ff_big = 0;
  double acc_se = 0.0, gv_max = 0.0;
  int acc_n = 0;
  std::vector<double> gvx;

  State prev = State::WAIT_CONNECTION;
  double abort_t = -1.0, mode_ok_t = -1.0;
  Vec3 abort_pos{};
  long sp_after = -1;
  double lost_at = -1.0;
  std::vector<double> lost_delay, refind_time;   // od skrytia po stratu, od straty po najdenie

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
    if (moving) {
      if (!walk && track_t > 0.0 && veh.t() > track_t + 20.0) {
        walk = true;
        heading = std::atan2(rpos.y - rc.y, rpos.x - rc.x) + 1.57;
        next_turn = veh.t() + 8.0;
      }
      if (walk) {
        if (veh.t() >= next_turn) {heading += turn(rng); next_turn += 8.0;}
        rpos.x += 0.5 * std::cos(heading) * dt;
        rpos.y += 0.5 * std::sin(heading) * dt;
      } else {
        rpos = rover(veh.t());
      }
      if (search_t > 0.0 && k % 2 == 0) {
        const Vec3 rel{rpos.x - st0.pos.x, rpos.y - st0.pos.y, -st0.pos.z};
        const bool vis = lrs_control::inFootprint(rel, st0.yaw, st0.pos.z, cam);
        if (track_t > 0.0) {++frames; seen += vis ? 1 : 0;}
        if (vis && !hidden(veh.t())) {
          pending.push_back({veh.t(), {rpos.x + noise(rng), rpos.y + noise(rng), 0.0}});
        }
      }
      while (!pending.empty() && pending.front().t + 0.1 <= veh.t() + 1e-9) {
        obs.push_back(pending.front());
        pending.pop_front();
      }
    }
    if ((sc == "confirm" || sc == "abort") && search_t > 0.0 && k % 2 == 0) {   // kamera 10 Hz
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
    if (sc == "gazebo") {
      // rover
      if (veh.t() >= g_next) {
        g_cmd_speed = 0.15 + 0.25 * u01(rng);
        g_cmd_rate = -0.2 + 0.4 * u01(rng);
        g_next = veh.t() + 3.0 + 3.0 * u01(rng);
      }
      const Vec3 off{rpos.x - rc.x, rpos.y - rc.y, 0.0};
      if (std::hypot(off.x, off.y) > 4.0) {   // spat k stredu
        const double want = std::atan2(-off.y, -off.x);
        g_cmd_rate = std::clamp(2.0 * std::remainder(want - g_yaw, 2.0 * M_PI), -0.2, 0.2);
      }
      g_speed += std::clamp(g_cmd_speed - g_speed, -0.2 * dt, 0.2 * dt);
      g_rate += std::clamp(g_cmd_rate - g_rate, -0.2 * dt, 0.2 * dt);
      g_yaw += g_rate * dt;
      rpos.x += g_speed * std::cos(g_yaw) * dt;
      rpos.y += g_speed * std::sin(g_yaw) * dt;
      // zrychlenie drona (naklon ~ a / g) a jeho historia
      const Vec3 acc{(st0.vel.x - prev_vel.x) / dt, (st0.vel.y - prev_vel.y) / dt, 0.0};
      prev_vel = st0.vel;
      // naklon ~ zrychlenie / g, s dynamikou natocenia (casova konstanta 0.2 s)
      const double lp = dt / (0.2 + dt);
      tilt.x += lp * (acc.x / 9.81 - tilt.x);
      tilt.y += lp * (acc.y / 9.81 - tilt.y);
      acc_hist.push_back({veh.t(), tilt});
      while (acc_hist.size() > 2 && acc_hist.front().first < veh.t() - 0.5) {acc_hist.pop_front();}
      Vec3 tilt_old = acc_hist.front().second;
      for (const auto & h : acc_hist) {
        if (h.first <= veh.t() - tilt_delay + 1e-9) {tilt_old = h.second;}
      }
      // detektor 28 Hz: chyba = vyska * (naklon teraz - naklon, ktory hlasi MAVROS s oneskorenim)
      const double kt = st0.pos.z * tilt_sign;
      while (search_t > 0.0 && g_next_det <= veh.t()) {
        const Vec3 rel{rpos.x - st0.pos.x, rpos.y - st0.pos.y, -st0.pos.z};
        if (lrs_control::inFootprint(rel, st0.yaw, st0.pos.z, cam)) {
          obs.push_back({g_next_det, {rpos.x + g_noise(rng) + kt * (tilt.x - tilt_old.x),
              rpos.y + g_noise(rng) + kt * (tilt.y - tilt_old.y), 0.0}});
        }
        g_next_det += 1.0 / 28.0;
      }
      if (g_next_det < veh.t() - 1.0) {g_next_det = veh.t();}
    }
    ex.tick(veh.t(), obs);
    const auto st = veh.state();
    if (sc == "gazebo" && ex.state() == State::TRACK) {
      if (track_t < 0.0) {track_t = veh.t();}
      if (veh.t() > track_t + 10.0) {
        const double e = lrs_control::distXY(st.pos, rpos);
        se += e * e;
        emax = std::max(emax, e);
        ++ne;
        const Vec3 ff = veh.lastSetpoint().v.value_or(Vec3{});
        ++ff_n;
        ff_big += std::hypot(ff.x, ff.y) > 0.6 ? 1 : 0;
        const double a = std::hypot(tilt.x, tilt.y);   // [rad]
        acc_se += a * a;
        ++acc_n;
        gv_max = std::max(gv_max, std::hypot(st.vel.x, st.vel.y));
        gvx.push_back(st.vel.x);
      }
      if (veh.t() > track_t + 260.0) {break;}
    }
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
    if (sc == "track" && ex.state() == State::TRACK) {
      if (track_t < 0.0) {track_t = veh.t();}
      if (veh.t() > track_t + 5.0) {
        const double e = lrs_control::distXY(st.pos, rpos);
        se += e * e;
        emax = std::max(emax, e);
        ++ne;
      }
      if (veh.t() > track_t + 120.0) {break;}
    }
    // abort: 10 s po najdeni ako cez sluzbu ~/abort, potom 20 s pozorovat
    if (sc == "abort") {
      if (ex.state() == State::TRACK && track_t < 0.0) {track_t = veh.t();}
      if (track_t > 0.0 && abort_t < 0.0 && veh.t() > track_t + 10.0) {
        abort_t = veh.t();
        abort_pos = st.pos;
        ex.abort("abort cez sluzbu ~/abort (test)");
      }
      if (abort_t > 0.0 && mode_ok_t < 0.0 && st.mode == p.abort_mode) {mode_ok_t = veh.t();}
      if (abort_t > 0.0 && veh.t() > abort_t + 1.0 && sp_after < 0) {sp_after = veh.setpoints();}
      if (abort_t > 0.0 && veh.t() > abort_t + 20.0) {break;}
    }
    if (sc == "lost") {
      if (ex.state() == State::TRACK && track_t < 0.0) {track_t = veh.t();}
      if (prev == State::TRACK && ex.state() == State::SEARCH) {
        lost_at = veh.t();
        const double hide0 = veh.t() < track_t + 50.0 ? track_t + 30.0 : track_t + 70.0;
        lost_delay.push_back(veh.t() - hide0);
      }
      if (prev == State::SEARCH && ex.state() == State::TRACK && lost_at > 0.0) {
        refind_time.push_back(veh.t() - lost_at);
      }
      if (track_t > 0.0 && veh.t() > track_t + 110.0) {break;}
    }
    prev = ex.state();
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
    if (ex.state() == State::ABORT && sc != "abort") {break;}
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
  } else if (sc == "track") {
    const double rms = ne > 0 ? std::sqrt(se / ne) : 1e9;
    const double vis = frames > 0 ? 100.0 * seen / frames : 0.0;
    std::printf("%s: sledovanie 115 s, odchylka nad roverom RMS %.2f m, max %.2f m, "
      "rover v zabere %.1f %% snimok\n", naive ? "BEZ filtra" : "alfa-beta", rms, emax, vis);
    ok = track_t > 0.0 && ex.state() == State::TRACK && vis > 95.0 &&
      (naive || rms < 0.5) && std::abs(st.pos.z - p.follow_alt) < 0.3;
  } else if (sc == "abort") {
    const bool no_sp = sp_after >= 0 && veh.setpoints() == sp_after;
    const double moved = lrs_control::distXY(st.pos, abort_pos);
    std::printf("abort -> %s: mod potvrdeny po %.2f s, setpointy po aborte %s, LAND poziadavky %d, "
      "posun za 20 s %.2f m\n", p.abort_mode.c_str(), mode_ok_t - abort_t,
      no_sp ? "ziadne" : "STALE CHODIA", veh.landRequests(), moved);
    ok = ex.state() == State::ABORT && mode_ok_t > 0.0 && mode_ok_t - abort_t < 2.0 && no_sp &&
      veh.landRequests() == 0 && (p.abort_mode == "RTL" || moved < 0.5);
  } else if (sc == "gazebo") {
    const double rms = ne > 0 ? std::sqrt(se / ne) : 1e9;
    const double tilt_rms = std::sqrt(acc_se / std::max(acc_n, 1)) * 180.0 / M_PI;
    // dominantna frekvencia kmitania vx (DFT 0.2-2 Hz)
    double best_a = 0.0, best_f = 0.0;
    const double mean = gvx.empty() ? 0.0 :
      std::accumulate(gvx.begin(), gvx.end(), 0.0) / gvx.size();
    for (double f = 0.2; f <= 2.0; f += 0.02) {
      double re = 0.0, im = 0.0;
      for (std::size_t i = 0; i < gvx.size(); ++i) {
        re += (gvx[i] - mean) * std::cos(2 * M_PI * f * i * dt);
        im += (gvx[i] - mean) * std::sin(2 * M_PI * f * i * dt);
      }
      const double a = 2.0 * std::hypot(re, im) / std::max<std::size_t>(gvx.size(), 1);
      if (a > best_a) {best_a = a; best_f = f;}
    }
    std::printf("gazebo (naklon %s): odchylka nad roverom RMS %.2f m, max %.2f m\n"
      "  feedforward > 0.6 m/s: %.1f %% tickov, max rychlost drona %.2f m/s\n"
      "  naklon RMS %.1f deg, kmitanie vx %.2f Hz s amplitudou %.2f m/s\n",
      tilt_sign > 0 ? "+" : "-", rms, emax, 100.0 * ff_big / std::max(ff_n, 1L), gv_max,
      tilt_rms, best_f, best_a);
    ok = track_t > 0.0 && ex.state() == State::TRACK && rms < 0.3 && ff_big < 0.02 * ff_n &&
      tilt_rms < 3.0;
  } else if (sc == "lost") {
    double worst = 0.0;
    for (std::size_t i = 0; i < lost_delay.size(); ++i) {
      const double rf = i < refind_time.size() ? refind_time[i] : -1.0;
      std::printf("strata %zu: rozpoznana %.1f s po skryti, znova najdeny %.1f s po strate\n",
        i + 1, lost_delay[i], rf);
      worst = std::max(worst, rf < 0.0 ? 1e9 : rf);
    }
    ok = lost_delay.size() == 2 && refind_time.size() == 2 && worst < 15.0 &&
      ex.state() == State::TRACK && std::abs(st.pos.z - p.follow_alt) < 0.3;
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
