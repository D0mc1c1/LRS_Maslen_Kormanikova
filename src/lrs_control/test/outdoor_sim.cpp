// outdoor_sim [scenar]
//
// Offline test OutdoorExecutor BEZ ROS a bez Gazeba (simulovany dron: sim_vehicle.hpp).
// Scenare:
//   normal   GPS po 2 s, home po 4 s -> vzlet, drzanie, pristatie (DONE)
//   nohome   home nepride -> pociatok z prveho fixu po home_wait (DONE)
//   nogps    GPS sa nikdy nechyti -> NEARMUJE, po 30 s koniec testu
//   rc       pilot prepne pocas drzania na LOITER -> ABORT bez prikazu LAND
//   abort    sluzba ~/abort pocas drzania -> ABORT + LAND na mieste
#include <cstdio>
#include <iostream>
#include <string>

#include "lrs_control/geometry.hpp"
#include "lrs_control/outdoor_executor.hpp"
#include "sim_vehicle.hpp"

using namespace lrs_control;

// pocita poziadavky o LAND (mod aj cmd/land), aby sa dalo overit "nebojovat s pilotom"
class CountingSim : public SimVehicle
{
public:
  using SimVehicle::SimVehicle;
  void requestMode(const std::string & m) override
  {
    if (m == "LAND") {++land_requests;}
    SimVehicle::requestMode(m);
  }
  void requestLand() override {++land_requests; SimVehicle::requestLand();}
  int land_requests{0};
};

int main(int argc, char ** argv)
{
  const std::string sc = argc > 1 ? argv[1] : "normal";
  if (sc != "normal" && sc != "nohome" && sc != "nogps" && sc != "rc" && sc != "abort") {
    std::cerr << "usage: outdoor_sim [normal|nohome|nogps|rc|abort]\n";
    return 1;
  }

  // EKF origin v SITL nie je presne na mieste spawnu -> launch mierne mimo (0, 0)
  CountingSim veh({0.4, -0.3, 0.0});
  const LatLonAlt home{48.15144444, 17.07375000, 150.0};   // fei_lrs_outdoor.world
  veh.enableGps(home, sc == "nogps" ? 1e9 : 2.0, sc == "nohome" ? -1.0 : 4.0);

  OutdoorParams p;
  OutdoorExecutor ex(veh, p, [&veh](OutdoorExecutor::LogLevel l, const std::string & m) {
      const char * tag = l == OutdoorExecutor::LogLevel::Error ? "ERROR" :
      l == OutdoorExecutor::LogLevel::Warn ? "WARN " : "INFO ";
      std::printf("%s %s\n", tag, m.c_str());
    });

  const double dt = 0.05;
  const double t_end = sc == "nogps" ? 30.0 : 300.0;
  double hold_t0 = -1.0, max_z = 0.0, max_hold_err = 0.0;
  bool injected = false;
  int land_req_at_inject = 0;

  while (veh.t() < t_end && !ex.finished()) {
    veh.step(dt);
    ex.tick(veh.t());
    const auto st = veh.state();
    max_z = std::max(max_z, st.pos.z);

    if (ex.state() == OutdoorExecutor::State::HOLD) {
      if (hold_t0 < 0) {hold_t0 = veh.t();}
      const Vec3 tgt{ex.launch().x, ex.launch().y, ex.launch().z + p.altitude};
      if (veh.t() - hold_t0 > 2.0) {max_hold_err = std::max(max_hold_err, dist(st.pos, tgt));}
      if (!injected && veh.t() - hold_t0 > 3.0) {
        injected = true;
        land_req_at_inject = veh.land_requests;
        if (sc == "rc") {
          std::printf("SIM   pilot prepol z RC na LOITER\n");
          veh.forceMode("LOITER");
        } else if (sc == "abort") {
          std::printf("SIM   volanie ~/abort\n");
          ex.abort("abort cez sluzbu ~/abort");
        }
      }
    }
  }
  // ABORT: este 60 s simulovat (pristatie po abort, resp. ze po RC nic neposiela)
  for (int i = 0; i < 1200 && ex.state() == OutdoorExecutor::State::ABORT; ++i) {
    veh.step(dt);
    ex.tick(veh.t());
  }

  const auto st = veh.state();
  std::printf("\n=== %s: stav %s, t = %.1f s, max vyska %.2f m, max chyba drzania %.3f m\n",
    sc.c_str(), OutdoorExecutor::name(ex.state()), veh.t(), max_z, max_hold_err);
  std::printf("    koniec: mod %s armed %d, poloha (%.2f, %.2f, %.2f), LAND poziadaviek %d\n",
    st.mode.c_str(), st.armed, st.pos.x, st.pos.y, st.pos.z, veh.land_requests);

  bool ok = false;
  if (sc == "normal" || sc == "nohome") {
    ok = ex.state() == OutdoorExecutor::State::DONE && std::abs(max_z - p.altitude) < 0.3 &&
      max_hold_err < p.hold_tol && !st.armed;
  } else if (sc == "nogps") {
    ok = ex.state() == OutdoorExecutor::State::WAIT_GPS && !st.armed && max_z == 0.0;
  } else if (sc == "rc") {
    // po prevzati pilotom ziadna poziadavka o LAND a mod ostal LOITER
    ok = ex.state() == OutdoorExecutor::State::ABORT && veh.land_requests == land_req_at_inject &&
      st.mode == "LOITER";
  } else if (sc == "abort") {
    ok = ex.state() == OutdoorExecutor::State::ABORT && st.mode == "LAND" && !st.armed;
  }
  std::printf("    VYSLEDOK: %s\n", ok ? "OK" : "CHYBA");
  return ok ? 0 : 2;
}
