// outdoor_sim [scenar]
//
// Offline test OutdoorExecutor BEZ ROS a bez Gazeba (simulovany dron: sim_vehicle.hpp).
// Scenare:
//   normal    GPS po 2 s, home po 4 s -> vzlet, stvorec s POI, osmicka, navrat, pristatie
//   noff      ako normal, ale bez feedforward (len poloha + yaw) -> vacsie chyby
//   noramp    osmicka bez rozbehu (th = omega * t od nuly) -> skok rychlosti na starte
//   fast      osmicka omega 0.35 rad/s (max 1.48 m/s, limit 1.5)
//   toofast   osmicka omega 0.40 rad/s (max 1.70 m/s > 1.5) -> NEVZLIETA
//   setpoint  smer na POI zo setpointu namiesto skutocnej polohy
//   latlon    POI zadane ako lat/lon (rovnake miesto ako posun E 0 N 5)
//   nohome    home nepride -> pociatok z GPS fixu po home_wait (DONE)
//   nogps     GPS sa nikdy nechyti -> NEARMUJE, po 30 s koniec testu
//   geofence  POI 50 m daleko -> NEVZLIETA
//   rc        pilot prepne na stvorci na LOITER -> ABORT, nic neprikazuje, ziadne setpointy
//   abort     sluzba ~/abort v polovici osmicky -> ABORT + LAND na mieste
//   abortrtl  ako abort, ale abort_mode RTL -> navrat nad home a pristatie (ArduPilot)
//   linkloss  MAVROS spadne na osmicke -> ABORT, nic sa neposiela
//
// SIM_CSV=subor.csv -> zaznam letu (20 Hz) v rovnakom tvare, aky cita plot_outdoor.py --csv
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

#include "lrs_control/geometry.hpp"
#include "lrs_control/outdoor_executor.hpp"
#include "sim_vehicle.hpp"

using namespace lrs_control;
using State = OutdoorExecutor::State;

int main(int argc, char ** argv)
{
  const std::string sc = argc > 1 ? argv[1] : "normal";
  const std::set<std::string> known{"normal", "noff", "noramp", "fast", "toofast", "setpoint",
    "latlon", "nohome", "nogps", "geofence", "rc", "abort", "abortrtl", "linkloss"};
  if (!known.count(sc)) {
    std::cerr << "usage: outdoor_sim [normal|noff|noramp|fast|toofast|setpoint|latlon|nohome|"
      "nogps|geofence|rc|abort|abortrtl|linkloss]\n";
    return 1;
  }

  // EKF origin v SITL nie je presne na mieste spawnu -> launch mierne mimo (0, 0)
  SimVehicle veh({0.4, -0.3, 0.0});
  const LatLonAlt home{48.15144444, 17.07375000, 150.0};   // fei_lrs_outdoor.world
  veh.enableGps(home, sc == "nogps" ? 1e9 : 2.0, sc == "nohome" ? -1.0 : 4.0);

  OutdoorParams p;
  if (sc == "noff") {p.velocity_ff = false; p.yaw_rate_ff = false;}
  if (sc == "setpoint") {p.poi_yaw_from_setpoint = true;}
  if (sc == "latlon") {
    // E 0 N 5 od miesta vzletu (0.4, -0.3) = (0.4, 4.7) od home
    const LatLonAlt g = LocalTangentPlane(home).toLla({0.4, 4.7, 0.0});
    p.poi_use_latlon = true;
    p.poi_lat = g.lat_deg;
    p.poi_lon = g.lon_deg;
  }
  if (sc == "geofence") {p.poi_north = 50.0;}
  if (sc == "noramp") {p.fig8_ramp = 0.0;}
  if (sc == "fast") {p.fig8_omega = 0.35;}
  if (sc == "toofast") {p.fig8_omega = 0.40;}
  if (sc == "abortrtl") {p.abort_mode = "RTL";}

  OutdoorExecutor ex(veh, p, [&veh](OutdoorExecutor::LogLevel l, const std::string & m) {
      const char * tag = l == OutdoorExecutor::LogLevel::Error ? "ERROR" :
      l == OutdoorExecutor::LogLevel::Warn ? "WARN " : "INFO ";
      std::printf("%s %s\n", tag, m.c_str());
    });

  std::ofstream csv;
  if (const char * f = std::getenv("SIM_CSV")) {
    csv.open(f);
    csv << "t,phase,x,y,z,yaw,sp_valid,sp_x,sp_y,sp_z,sp_yaw,poi_x,poi_y,rel_alt\n";
  }
  long sp_count_csv = 0;

  const double dt = 0.05;
  const double t_end = sc == "nogps" ? 30.0 : 300.0;
  double hold_t0 = -1.0, max_z = 0.0, max_hold_err = 0.0, max_r = 0.0;
  bool injected = false;
  int land_req_at_inject = 0;
  long sp_at_inject = 0;   // pocet setpointov pri vstupe do ABORT

  while (veh.t() < t_end && !ex.finished()) {
    veh.step(dt);
    ex.tick(veh.t());
    const auto st = veh.state();
    max_z = std::max(max_z, st.pos.z);
    if (csv.is_open() && ex.hasOrigin()) {
      const auto & sp = veh.lastSetpoint();
      const bool sp_valid = veh.setpoints() != sp_count_csv;   // v tomto ticku poslany setpoint
      sp_count_csv = veh.setpoints();
      char b[320];
      std::snprintf(b, sizeof(b), "%.3f,%s,%.4f,%.4f,%.4f,%.5f,%d,%.4f,%.4f,%.4f,%.5f,%.4f,%.4f,%.4f\n",
        veh.t(), ex.phase().c_str(), st.pos.x, st.pos.y, st.pos.z, st.yaw, sp_valid ? 1 : 0,
        sp.p.x, sp.p.y, sp.p.z, sp.yaw, ex.poi().x, ex.poi().y, st.rel_alt);
      csv << b;
    }
    max_r = std::max(max_r, distXY(st.pos, ex.launch()));

    if (ex.state() == State::HOLD) {
      if (hold_t0 < 0) {hold_t0 = veh.t();}
      const Vec3 tgt{ex.launch().x, ex.launch().y, ex.launch().z + p.altitude};
      if (veh.t() - hold_t0 > 2.0) {max_hold_err = std::max(max_hold_err, dist(st.pos, tgt));}
    }
    // pilot uprostred stvorca (po W2), abort v polovici osmicky
    if (!injected && sc == "rc" && ex.state() == State::SQUARE && ex.cornersReached() >= 2) {
      injected = true;
      std::printf("SIM   pilot prepol z RC na LOITER\n");
      veh.forceMode("LOITER");
    }
    if (!injected && (sc == "abort" || sc == "abortrtl" || sc == "linkloss") &&
      ex.state() == State::FIG8 && ex.fig8Stats().n > 280)
    {
      injected = true;
      if (sc == "linkloss") {
        std::printf("SIM   MAVROS spadol\n");
        veh.dropLink();
      } else {
        std::printf("SIM   volanie ~/abort\n");
        ex.abort("abort cez sluzbu ~/abort");
      }
    }
    // setpointy a LAND poziadavky od vstupu do ABORT
    // (po strate spojenia ide ABORT az po pose_timeout)
    if (ex.state() == State::ABORT && sp_at_inject == 0) {
      sp_at_inject = veh.setpoints();
      land_req_at_inject = veh.landRequests();
    }
  }
  // ABORT: este 60 s simulovat (pristatie po abort, resp. ze po RC nic neposiela)
  for (int i = 0; i < 1200 && ex.state() == State::ABORT; ++i) {
    veh.step(dt);
    ex.tick(veh.t());
  }

  const auto st = veh.state();
  const auto & ps = ex.poiStats();
  std::printf("\n=== %s: stav %s, t = %.1f s, max vyska %.2f m, max od vzletu %.2f m\n",
    sc.c_str(), OutdoorExecutor::name(ex.state()), veh.t(), max_z, max_r);
  const auto & fs = ex.fig8Stats();
  std::printf("    drzanie max chyba %.3f m | rohy %d/5 | yaw na POI max %.1f deg RMS %.1f deg\n",
    max_hold_err, ex.cornersReached(), ps.max_deg, ps.rmsDeg());
  std::printf("    osmicka %s: odchylka max %.2f m RMS %.2f m | yaw max %.1f deg RMS %.1f deg | "
    "E %.2f..%.2f N %.2f..%.2f\n", fs.done ? "hotova" : "NEDOKONCENA", fs.track_max,
    fs.trackRms(), fs.yaw_max_deg, fs.yawRmsDeg(), fs.x_min, fs.x_max, fs.y_min, fs.y_max);
  std::printf("    koniec: mod %s armed %d, poloha (%.2f, %.2f, %.2f), LAND poziadaviek %d\n",
    st.mode.c_str(), st.armed, st.pos.x, st.pos.y, st.pos.z, veh.landRequests());
  const long sp_after = veh.setpoints() - sp_at_inject;
  if (injected) {
    std::printf("    po ABORT: setpointov %ld, LAND poziadaviek %d, finished %d\n",
      sp_after, veh.landRequests() - land_req_at_inject, ex.finished());
  }

  bool ok = false;
  // tvar osmicky: +-A vo vychode, +-A/2 v severe (zrkadlova by mala rovnaky obal,
  // smer obletu kontroluje yaw vs. tangenta)
  const double A = p.fig8_amplitude;
  const bool shape = std::abs(fs.x_max - A) < 0.2 && std::abs(fs.x_min + A) < 0.2 &&
    std::abs(fs.y_max - A / 2) < 0.2 && std::abs(fs.y_min + A / 2) < 0.2;
  const bool full = ex.state() == State::DONE && std::abs(max_z - p.altitude) < 0.3 &&
    max_hold_err < p.hold_tol && !st.armed && ex.cornersReached() == 5 && fs.done && shape &&
    distXY(st.pos, ex.launch()) < 0.3;
  if (sc == "normal" || sc == "latlon" || sc == "nohome" || sc == "fast") {
    ok = full && ps.max_deg < 3.0 && fs.track_max < 0.3 && fs.yaw_max_deg < 5.0;
  } else if (sc == "noff" || sc == "setpoint" || sc == "noramp") {
    ok = full;   // len prejst; chyby porovnat s normal
  } else if (sc == "nogps") {
    ok = ex.state() == State::WAIT_GPS && !st.armed && max_z == 0.0;
  } else if (sc == "geofence" || sc == "toofast") {
    ok = ex.state() == State::ABORT && !st.armed && max_z == 0.0;
  } else if (sc == "rc" || sc == "linkloss") {
    // nic neprikazovat: ziadna poziadavka o LAND, ziadny setpoint po ABORT,
    // node sa moze ukoncit
    ok = ex.state() == State::ABORT && veh.landRequests() == land_req_at_inject &&
      sp_after == 0 && ex.finished() && (sc == "linkloss" || st.mode == "LOITER");
  } else if (sc == "abort") {
    ok = ex.state() == State::ABORT && ex.finished() && st.mode == "LAND" && !st.armed;
  } else if (sc == "abortrtl") {
    ok = ex.state() == State::ABORT && ex.finished() && !st.armed &&
      std::hypot(st.pos.x, st.pos.y) < 0.2;   // pristal na home
  }
  std::printf("    VYSLEDOK: %s\n", ok ? "OK" : "CHYBA");
  return ok ? 0 : 2;
}
