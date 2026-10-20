// Program raz za sekundu vypisuje udaje o GPS polohe, vyske,
// pocte satelitov, presnosti GPS, suradnicovom ramci ENU a kurze drona.
//
// GPS suradnice sa zobrazuju v stupnoch a metroch:

// ENU predstavuje lokalny suradnicovy system:
// E = vychod, N = sever, U = hore.
//
// Program porovnava polohu vypocitanu z GPS s lokalnou polohou MAVROS.
// Zaroven porovnava smer z kompasu so smerom yaw z local_position.
//
// Tento uzol je iba diagnosticky.

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "lrs_control/geo.hpp"
#include "lrs_control/geometry.hpp"
#include "lrs_control/mavros_vehicle.hpp"

using namespace std::chrono_literals;
using lrs_control::LatLonAlt;
using lrs_control::LocalTangentPlane;

// Hlavna trieda ROS 2 uzla na diagnostiku GPS, polohy a orientacie drona.
class GpsInfoNode : public rclcpp::Node
{
public:
  // Konstruktor vytvori uzol s nazvom gps_info.
  GpsInfoNode()
  : Node("gps_info")
  {
    // Minimalny pocet satelitov a maximalna povolena horizontalna chyba GPS.
    // Hodnoty sa daju zmenit cez parametre ROS 2.
    min_sats_ = declare_parameter<int>("min_satellites", 6);
    max_h_acc_ = declare_parameter<double>("max_h_acc", 2.0);

    // Vytvorenie objektu na komunikaciu s MAVROS.
    // FrameTransform{} znamena, ze sa nepouziva dodatocna transformacia
    // medzi suradnicovymi ramcami.
    vehicle_ = std::make_unique<lrs_control::MavrosVehicle>(*this, lrs_control::FrameTransform{});

    // Casovac spusta funkciu print() kazdu sekundu.
    timer_ = create_wall_timer(1s, [this]() {print();});
  }

private:
  // Funkcia nacita aktualny stav drona a vypise diagnosticke informacie.
  void print()
  {
    // Ziskanie poslednych udajov o GPS, polohe, orientacii a stave MAVROS.
    const auto st = vehicle_->state();

    // Ak GPS udaje este nie su dostupne, vypise sa upozornenie
    // a funkcia sa ukonci bez dalsieho spracovania.
    if (!st.has_gps) {
      RCLCPP_INFO(get_logger(), "cakam na /mavros/global_position/global ...");
      return;
    }

    // Kontrola, ci GPS splna pozadovane podmienky.
    // Funkcia gpsReady() kontroluje napriklad pocet satelitov a presnost.
    std::string why;
    const bool ready = lrs_control::gpsReady(st, min_sats_, max_h_acc_, why);

    // Pomocne pole na formatovanie textovych vystupov.
    char b[256];

    // Vystupny retazec, do ktoreho sa postupne ukladaju diagnosticke udaje.
    std::string out = "\n";

    // Vypis stavu GPS, poctu satelitov, presnosti a vysledku kontroly.
    std::snprintf(b, sizeof(b),
      "GPS   : status %d, satelity %s, presnost %s -> %s\n", st.gps_status,
      st.satellites >= 0 ? std::to_string(st.satellites).c_str() : "?",
      st.h_acc >= 0 ? (std::to_string(st.h_acc).substr(0, 4) + " m").c_str() : "?",
      ready ? "POUZITELNE NA VZLET" : ("NEPOUZITELNE: " + why).c_str());
    out += b;

    // Vypis zemepisnej sirky, dlzky, GPS vysky a relativnej vysky.
    // rel_alt predstavuje relativnu vysku, ak ju MAVROS poskytuje.
    std::snprintf(b, sizeof(b), "poloha: lat %.7f  lon %.7f  alt %.2f m  | rel_alt %s\n",
      st.gps.lat_deg, st.gps.lon_deg, st.gps.alt_m,
      st.has_rel_alt ? (std::to_string(st.rel_alt).substr(0, 5) + " m").c_str() : "?");
    out += b;

    // Urci sa pociatok lokalneho suradnicoveho systemu ENU.
    // Ak MAVROS poskytuje home, pouzije sa tato poloha.
    // Ak home este nie je dostupny, pouzije sa prva platna GPS poloha.
    if (st.has_home) {
      origin_ = st.home;
      origin_src_ = "home z /mavros/home_position/home";
      has_origin_ = true;
    } else if (!has_origin_ && ready) {
      origin_ = st.gps;
      origin_src_ = "prva platna GPS poloha (home zatial nie je)";
      has_origin_ = true;
    }

    // Ak uz mame pociatok, mozeme prevadzat GPS suradnice na lokalne ENU.
    if (has_origin_) {
      // Vytvorenie lokalnej tangentovej roviny s pociatkom origin_.
      const LocalTangentPlane ltp(origin_);

      // Vypis zvoleneho pociatku a informacie o jeho zdroji.
      std::snprintf(b, sizeof(b), "pociatok: lat %.7f  lon %.7f  alt %.2f m  <- %s\n",
        origin_.lat_deg, origin_.lon_deg, origin_.alt_m, origin_src_.c_str());
      out += b;

      // Prevod GPS suradnic na lokalne suradnice ENU.
      // enu.x = vychodna vzdialenost, enu.y = severna vzdialenost.
      // enu.z je vyskovy rozdiel vypocitany z geodetickych udajov.
      //
      // Pre vysku sa prednostne pouzije rel_alt, pretoze GPS vyska
      // a vyska home nemusia pouzivat rovnaky vyskovy system.
      const auto enu = ltp.toEnu(st.gps);
      const double up = st.has_rel_alt ? st.rel_alt : enu.z;

      // Vypis polohy vypocitanej z GPS v metroch od pociatku.
      std::snprintf(b, sizeof(b),
        "ENU z GPS        : E %7.2f  N %7.2f  U %7.2f m (U = rel_alt; rozdiel GPS altitude %.2f m)\n",
        enu.x, enu.y, up, enu.z);
      out += b;

      // Porovnanie s lokalnou polohou, ktoru poskytuje MAVROS.
      // Ide o polohu z lokalneho odhadu EKF.
      if (st.has_pose) {
        // Vypocet horizontalneho rozdielu medzi GPS a local position.
        // Pouziva sa vzdialenost v rovine EN, bez vysky.
        const double dh = std::hypot(enu.x - st.pos.x, enu.y - st.pos.y);

        // Vypis lokalnej polohy a rozdielu oproti GPS.
        std::snprintf(b, sizeof(b),
          "MAVROS local     : E %7.2f  N %7.2f  U %7.2f m  | rozdiel horizontalne %.2f m\n",
          st.pos.x, st.pos.y, st.pos.z, dh);
        out += b;

        // Opacny prevod: lokalna poloha ENU sa prevedie na GPS suradnice.
        // Takto sa da skontrolovat aj prevod z lokalneho systemu na LLA.
        const LatLonAlt back = ltp.toLla({st.pos.x, st.pos.y, st.pos.z});

        // Prevod spat do ENU sluzi na kontrolu oboch smerov transformacie.
        const auto again = ltp.toEnu(back);

        // Vypis zemepisnych suradnic a spatne vypocitanej polohy ENU.
        std::snprintf(b, sizeof(b),
          "local -> LLA     : lat %.7f  lon %.7f (spat do ENU: E %.2f N %.2f)\n",
          back.lat_deg, back.lon_deg, again.x, again.y);
        out += b;
      }
    } else {
      // Ak zatial nemame home ani platnu GPS polohu,
      // lokalny suradnicovy system este nie je mozne inicializovat.
      out += "pociatok: zatial nie je (cakam na home alebo platny GPS fix)\n";
    }

    // Ak je dostupny kompas, porovna sa jeho kurz s ENU yaw.
    if (st.has_compass) {
      // Prevod kompasoveho kurzu na yaw v suradnicovom systeme ENU.
      const double yaw_from_compass = lrs_control::compassToEnuYawDeg(st.compass_hdg_deg);

      // Prevod yaw z local_position z radianov na stupne.
      const double yaw_pose = lrs_control::rad2deg(st.yaw);

      // Rozdiel orientacii sa vypocita pomocou uhlovej diferencie,
      // aby sa spravne osetril prechod cez hranicu -180/180 stupnov.
      const double diff = lrs_control::rad2deg(
        lrs_control::angleDiff(lrs_control::deg2rad(yaw_from_compass), st.yaw));

      // Vypis kompasoveho kurzu, yaw z MAVROS a ich uhloveho rozdielu.
      std::snprintf(b, sizeof(b),
        "kurz  : kompas %6.1f deg (0 = sever, CW) -> ENU %6.1f deg | yaw z local_position %6.1f deg ENU"
        " | rozdiel %.1f deg\n", st.compass_hdg_deg, yaw_from_compass, yaw_pose, diff);
      out += b;
    }

    // Zobrazenie vsetkych nazbieranych diagnostickych udajov v ROS logu.
    RCLCPP_INFO(get_logger(), "%s", out.c_str());
  }

  // Parametre na posudenie kvality GPS.
  int min_sats_{6};
  double max_h_acc_{2.0};

  // Pociatok lokalneho suradnicoveho systemu a informacia o jeho zdroji.
  LatLonAlt origin_{};
  std::string origin_src_;
  bool has_origin_{false};

  // Objekt na komunikaciu s MAVROS a casovac diagnostiky.
  std::unique_ptr<lrs_control::MavrosVehicle> vehicle_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// Vstupny bod programu.
int main(int argc, char ** argv)
{
  // Inicializacia ROS 2.
  rclcpp::init(argc, argv);

  // Vytvorenie diagnostickeho uzla.
  auto node = std::make_shared<GpsInfoNode>();

  // Executor zabezpecuje spracovanie casovaca a dalsich callbackov uzla.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  // Ukoncenie ROS 2 po zastaveni vykonavania uzla.
  rclcpp::shutdown();
  return 0;
}
