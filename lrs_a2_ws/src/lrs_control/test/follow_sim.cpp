// Offline test FollowExecutor bez ROS a bez simulatora.
// usage: follow_sim [takeoff|rc|linkloss]
// TODO: hladanie, sledovanie, strata a opatovne najdenie
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "lrs_control/follow_executor.hpp"
#include "sim_vehicle.hpp"

using lrs_control::FollowExecutor;
using State = FollowExecutor::State;

int main(int argc, char ** argv)
{
  const std::string sc = argc > 1 ? argv[1] : "takeoff";
  if (sc != "takeoff" && sc != "rc" && sc != "linkloss") {
    std::printf("usage: follow_sim [takeoff|rc|linkloss]\n");
    return 1;
  }

  lrs_control::SimVehicle veh({-10.0, -10.0, 0.0});
  lrs_control::FollowParams p;
  FollowExecutor ex(veh, p, [&veh](const std::string & m) {
      std::printf("[t=%6.2f] %s\n", veh.t(), m.c_str());
    });

  const double dt = 0.05;
  const std::vector<lrs_control::TargetObs> no_obs;
  double search_t = -1.0;
  bool injected = false;
  while (veh.t() < 60.0) {
    veh.step(dt);
    ex.tick(veh.t(), no_obs);
    if (ex.state() == State::SEARCH && search_t < 0.0) {search_t = veh.t();}
    // 5 s po vzlete zasah pilota alebo strata spojenia
    if (search_t > 0.0 && veh.t() > search_t + 5.0 && !injected) {
      injected = true;
      if (sc == "rc") {veh.forceMode("LOITER");}
      if (sc == "linkloss") {veh.dropLink();}
    }
    if (ex.state() == State::ABORT) {break;}
  }

  const auto st = veh.state();
  std::printf("koniec: stav %s, poloha %.2f %.2f %.2f\n", FollowExecutor::name(ex.state()),
    st.pos.x, st.pos.y, st.pos.z);

  bool ok = false;
  if (sc == "takeoff") {
    ok = ex.state() == State::SEARCH && std::abs(st.pos.z - p.search_alt) < 0.3 &&
      std::abs(st.pos.x + 10.0) < 0.2 && std::abs(st.pos.y + 10.0) < 0.2;
  } else {
    ok = injected && ex.state() == State::ABORT;
  }
  std::printf("%s\n", ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
