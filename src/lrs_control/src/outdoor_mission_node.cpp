// A1.3 outdoor misia.
//
// Spustenie ROS 2 uzla s konfiguracnym suborom:
//   ros2 run lrs_control outdoor_mission --ros-args
//     --params-file src/lrs_control/config/outdoor.yaml
//
// Postup misie:
// GPS -> home -> GUIDED -> arm -> vzlet -> drzanie pozicie
// -> natocenie na POI -> stvorec -> osmicka -> navrat -> pristatie.
//
// POI (Point of Interest) je zaujmovy bod, na ktory sa dron pocas letu
// nataca prednou stranou.
// Osmicka sa riadi pomocou casovej trajektorie a yaw sleduje jej tangentu.
//
// Ramec MAVROS local ENU:
// E = vychod, N = sever, U = vyska.
// FrameTransform{} znamena, ze sa nepouziva ziadna dodatocna transformacia.
//
// Zaznam a vizualizacia v RViz:
// /lrs/outdoor/state   - aktualny stav misie
// /lrs/outdoor/poi     - poloha zaujmoveho bodu
// /lrs/outdoor/plan    - naplanovana trasa
// /lrs/outdoor/markers - znacky bodov v RViz
// /lrs/outdoor/flown   - skutocne preletena trasa
//
// Planovana trajektoria sa porovnava so skutocnou polohou drona.
// Na vykreslenie a analyzu zaznamu sluzi scripts/plot_outdoor.py.
//
// Vlakna:
// control_group  - riadenie misie a sluzba abort
// sensor_group   - prijimanie udajov zo senzorov MAVROS
// service_group  - odpovede sluzieb MAVROS

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "lrs_control/geo.hpp"
#include "lrs_control/geometry.hpp"
#include "lrs_control/mavros_vehicle.hpp"
#include "lrs_control/outdoor_executor.hpp"

using lrs_control::OutdoorExecutor;

// Hlavna trieda ROS 2 uzla, ktora nastavuje parametre,
// vytvara objekty na riadenie drona a zabezpecuje komunikaciu.
class OutdoorMissionNode : public rclcpp::Node
{
public:
  OutdoorMissionNode()
  : Node("outdoor_mission")
  {
    // Frekvencia posielania riadiacich setpointov a vypisu stavu.
    const double rate = declare_parameter<double>("setpoint_rate", 20.0);
    status_period_ = declare_parameter<double>("status_period", 2.0);

    // Nacitanie parametrov misie z konfiguracie alebo pouzitie predvolenych hodnot.
    lrs_control::OutdoorParams p;
    p.altitude = declare_parameter("altitude", p.altitude);
    p.min_satellites = declare_parameter("min_satellites", p.min_satellites);
    p.max_h_acc = declare_parameter("max_h_acc", p.max_h_acc);
    p.gps_settle_time = declare_parameter("gps_settle_time", p.gps_settle_time);
    p.home_wait = declare_parameter("home_wait", p.home_wait);
    p.takeoff_tol = declare_parameter("takeoff_tol", p.takeoff_tol);
    p.hold_tol = declare_parameter("hold_tol", p.hold_tol);
    p.hold_v = declare_parameter("hold_v", p.hold_v);
    p.hold_time = declare_parameter("hold_time", p.hold_time);

    // POI: ak su zadane suradnice GPS, pouzije sa lat/lon.
    // Inak sa POI urci posunom od miesta vzletu.
    p.poi_lat = declare_parameter("poi_lat", 0.0);
    p.poi_lon = declare_parameter("poi_lon", 0.0);
    p.poi_use_latlon = p.poi_lat != 0.0 && p.poi_lon != 0.0;
    p.poi_east = declare_parameter("poi_east", p.poi_east);
    p.poi_north = declare_parameter("poi_north", p.poi_north);

    // Parametre stvorca a pohybu drona.
    p.square_side = declare_parameter("square_side", p.square_side);
    p.speed = declare_parameter("speed", p.speed);
    p.accel = declare_parameter("accel", p.accel);
    p.corner_tol = declare_parameter("corner_tol", p.corner_tol);
    p.leg_timeout_extra = declare_parameter("leg_timeout_extra", p.leg_timeout_extra);

    // Vyber sposobu urcenia smeru drona na POI.
    const auto yaw_src = declare_parameter<std::string>("poi_yaw_source", "position");
    p.poi_min_dist = declare_parameter("poi_min_dist", p.poi_min_dist);
    p.align_tol_deg = declare_parameter("align_tol_deg", p.align_tol_deg);
    p.align_time = declare_parameter("align_time", p.align_time);

    // Feedforward parametre: dopredne odovzdanie rychlosti a uhlovej rychlosti.
    p.velocity_ff = declare_parameter("velocity_ff", p.velocity_ff);
    p.yaw_rate_ff = declare_parameter("yaw_rate_ff", p.yaw_rate_ff);

    // Bezpecnostne obmedzenia a parametre trajektorie osmicky.
    p.geofence_radius = declare_parameter("geofence_radius", p.geofence_radius);
    p.fig8_amplitude = declare_parameter("fig8_amplitude", p.fig8_amplitude);
    p.fig8_omega = declare_parameter("fig8_omega", p.fig8_omega);
    p.fig8_laps = declare_parameter("fig8_laps", p.fig8_laps);
    p.fig8_ramp = declare_parameter("fig8_ramp", p.fig8_ramp);
    p.fig8_rotation_deg = declare_parameter("fig8_rotation_deg", p.fig8_rotation_deg);
    p.fig8_max_speed = declare_parameter("fig8_max_speed", p.fig8_max_speed);
    p.max_track_err = declare_parameter("max_track_err", p.max_track_err);

    // Parametre bezpecnostneho spravania a casovych limitov.
    p.abort_mode = declare_parameter("abort_mode", p.abort_mode);
    p.state_timeout = declare_parameter("state_timeout", p.state_timeout);
    p.pose_timeout = declare_parameter("pose_timeout", p.pose_timeout);
    p.request_retry = declare_parameter("request_retry", p.request_retry);
    p.mode_timeout = declare_parameter("mode_timeout", p.mode_timeout);
    p.arm_timeout = declare_parameter("arm_timeout", p.arm_timeout);
    p.takeoff_timeout = declare_parameter("takeoff_timeout", p.takeoff_timeout);
    p.hold_timeout = declare_parameter("hold_timeout", p.hold_timeout);
    p.land_timeout = declare_parameter("land_timeout", p.land_timeout);

    // Kontrola, ci je pozadovana vyska v povolenom rozsahu.
    if (p.altitude < 1.0 || p.altitude > 30.0) {
      throw std::runtime_error("altitude mimo rozsahu 1..30 m");
    }

    // Kontrola platnosti sposobu urcenia yaw.
    if (yaw_src != "position" && yaw_src != "setpoint") {
      throw std::runtime_error("poi_yaw_source musi byt 'position' alebo 'setpoint'");
    }
    p.poi_yaw_from_setpoint = yaw_src == "setpoint";

    // Kontrola rychlosti a zrychlenia.
    if (p.speed <= 0.0 || p.speed > 3.0 || p.accel <= 0.0) {
      throw std::runtime_error("speed musi byt v (0, 3] m/s a accel > 0");
    }

    // Kontrola parametrov osmicky.
    if (p.fig8_omega <= 0.0 || p.fig8_laps < 1.0 || p.fig8_ramp < 0.0 || p.fig8_amplitude <= 0.0) {
      throw std::runtime_error("fig8: omega > 0, laps >= 1, ramp >= 0, amplitude > 0");
    }

    // Kontrola povoleneho modu pri preruseni misie.
    if (p.abort_mode != "LAND" && p.abort_mode != "RTL" && p.abort_mode != "BRAKE" &&
      p.abort_mode != "LOITER")
    {
      throw std::runtime_error("abort_mode musi byt LAND, RTL, BRAKE alebo LOITER");
    }

    // Stav MAVROS sa aktualizuje priblizne raz za sekundu,
    // preto musi byt timeout dostatocne dlhy.
    if (p.state_timeout < 1.5) {
      throw std::runtime_error("state_timeout < 1.5 s: /mavros/state chodi len ~1 Hz");
    }

    // Vytvorenie objektu na komunikaciu s MAVROS.
    // FrameTransform{} ponechava suradnicovy ramec bez dodatocnej transformacie.
    vehicle_ = std::make_unique<lrs_control::MavrosVehicle>(*this, lrs_control::FrameTransform{});

    // Vytvorenie vykonavaca misie.
    // Logger prevadza spravy vykonavaca na ROS logy podla ich zavaznosti.
    executor_ = std::make_unique<OutdoorExecutor>(
      *vehicle_, p,
      [this](OutdoorExecutor::LogLevel l, const std::string & m) {
        switch (l) {
          case OutdoorExecutor::LogLevel::Info: RCLCPP_INFO(get_logger(), "%s", m.c_str()); break;
          case OutdoorExecutor::LogLevel::Warn: RCLCPP_WARN(get_logger(), "%s", m.c_str()); break;
          case OutdoorExecutor::LogLevel::Error: RCLCPP_ERROR(get_logger(), "%s", m.c_str()); break;
        }
      });

    // QoS nastavenie uchova poslednu spravu, aby ju mohli dostat
    // aj odberatelia, ktori sa pripoja neskor.
    const auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    // Publisher aktualneho stavu misie.
    state_pub_ = create_publisher<std_msgs::msg::String>("/lrs/outdoor/state",
        rclcpp::QoS(rclcpp::KeepLast(20)).reliable().transient_local());

    // Publisher polohy POI, planovanej trasy a znaciek pre RViz.
    poi_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/lrs/outdoor/poi", latched);
    plan_pub_ = create_publisher<nav_msgs::msg::Path>("/lrs/outdoor/plan", latched);
    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/lrs/outdoor/markers",
        latched);

    // Publisher skutocnej trasy, ktoru dron preletel.
    flown_pub_ = create_publisher<nav_msgs::msg::Path>("/lrs/outdoor/flown", 10);
    flown_.header.frame_id = "map";

    // Riadiaca skupina zabezpecuje, ze jej callbacky nebudu bezat sucasne.
    control_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Timer pravidelne spusta riadenie misie.
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {controlTick();}, control_group_);

    // Sluzba umoznuje prerusit misiu zvonka cez ~/abort.
    abort_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/abort",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        executor_->abort("abort cez sluzbu ~/abort");
        res->success = true;
        res->message = "ABORT -> " + abort_mode_;
      },
#if RCLCPP_VERSION_MAJOR >= 28
      rclcpp::ServicesQoS(),
#else
      rmw_qos_profile_services_default,
#endif
      control_group_);

    // Vypis zakladnych informacii o bezpecnostnych parametroch a misii.
    abort_mode_ = p.abort_mode;
    RCLCPP_INFO(get_logger(), "abort: ~/abort -> %s, spojenie stratene po %.1f s (state) / "
      "%.1f s (poloha)", p.abort_mode.c_str(), p.state_timeout, p.pose_timeout);
    RCLCPP_INFO(get_logger(), "outdoor misia: vyska %.1f m, stvorec %.1f m, %.1f m/s, POI %s, "
      "yaw na POI zo %s, feedforward v=%d yaw_rate=%d, %.0f Hz",
      p.altitude, p.square_side, p.speed, p.poi_use_latlon ? "lat/lon" : "posun",
      yaw_src.c_str(), p.velocity_ff, p.yaw_rate_ff, rate);
  }

private:
  // Hlavny riadiaci krok, ktory sa spusta pravidelne pomocou timeru.
  void controlTick()
  {
    // Aktualny cas ROS sa odovzda vykonavacovi misie.
    const double t = now().seconds();
    executor_->tick(t);

    // Aktualizacia zaznamu a pravidelneho vypisu stavu.
    publishRecord();
    if (t - last_status_ >= status_period_) {
      last_status_ = t;
      printStatus();
    }

    // Po dokonceni misie sa ROS uzol korektne vypne.
    if (executor_->finished() && !finished_) {
      finished_ = true;
      RCLCPP_INFO(get_logger(), "misia skoncila v stave %s, koncim",
        OutdoorExecutor::name(executor_->state()));
      rclcpp::shutdown();
    }
  }

  // Zverejnenie aktualnej fazy, planovanej trasy a skutocnej prejdenej trasy.
  void publishRecord()
  {
    // Stav sa publikuje iba pri zmene fazy misie.
    const std::string ph = executor_->phase();
    if (ph != last_phase_) {
      last_phase_ = ph;
      std_msgs::msg::String m;
      m.data = ph;
      state_pub_->publish(m);
    }

    // Plan sa zverejni raz, ked uz je znamy pociatok a geometria misie.
    if (!plan_published_ && executor_->hasOrigin() && !executor_->square().empty()) {
      plan_published_ = true;
      publishPlan();
    }

    // Kazdy stvrty riadiaci tick sa pouzije na zaznamenanie polohy.
    // Pri frekvencii 20 Hz to zodpoveda priblizne 5 Hz.
    if (++viz_tick_ % 4 != 0) {
      return;
    }

    const auto st = vehicle_->state();

    // Zaznamenava sa iba poloha drona, ktory je armnuty a ma platnu polohu.
    // Limit poctu bodov brani nekontrolovanemu rastu zaznamu.
    const bool flying = st.armed && st.has_pose;
    if (!flying || flown_.poses.size() > 20000) {
      return;
    }

    // Prevod aktualnej polohy a yaw na spravu PoseStamped pre RViz.
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = now();
    ps.pose.position.x = st.pos.x;
    ps.pose.position.y = st.pos.y;
    ps.pose.position.z = st.pos.z;
    ps.pose.orientation.z = std::sin(st.yaw / 2.0);
    ps.pose.orientation.w = std::cos(st.yaw / 2.0);

    // Bod sa prida do zoznamu prejdenej trasy a trasa sa publikuje.
    flown_.header.stamp = ps.header.stamp;
    flown_.poses.push_back(ps);
    flown_pub_->publish(flown_);
  }

  // Zverejnenie POI, planovanej trasy a grafickych znaciek v RViz.
  void publishPlan()
  {
    const auto stamp = now();
    const auto & poi = executor_->poi();

    // Publikovanie polohy zaujmoveho bodu POI.
    geometry_msgs::msg::PointStamped pm;
    pm.header.frame_id = "map";
    pm.header.stamp = stamp;
    pm.point.x = poi.x;
    pm.point.y = poi.y;
    pm.point.z = poi.z;
    poi_pub_->publish(pm);

    // Prevod naplanovanych bodov na spravu Path.
    nav_msgs::msg::Path path;
    path.header = pm.header;
    for (const auto & p : executor_->plannedPath()) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header = pm.header;
      ps.pose.position.x = p.x;
      ps.pose.position.y = p.y;
      ps.pose.position.z = p.z;
      ps.pose.orientation.w = 1.0;
      path.poses.push_back(ps);
    }
    plan_pub_->publish(path);

    // Pomocna lambda vytvara znacky s rovnakymi zakladnymi vlastnostami.
    // Jednotlive znacky sa nasledne odlisuju typom, farbou, polohou a velkostou.
    visualization_msgs::msg::MarkerArray ma;
    auto mk = [&](int id, int type, double r, double g, double b) {
        visualization_msgs::msg::Marker m;
        m.header = pm.header;
        m.ns = "outdoor";
        m.id = id;
        m.type = type;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.color.r = r;
        m.color.g = g;
        m.color.b = b;
        m.color.a = 1.0;
        return m;
      };

    // Cervená gulova znacka oznacuje POI.
    auto poi_m = mk(0, visualization_msgs::msg::Marker::SPHERE, 1.0, 0.2, 0.2);
    poi_m.pose.position = pm.point;
    poi_m.scale.x = poi_m.scale.y = poi_m.scale.z = 0.4;
    ma.markers.push_back(poi_m);

    // Modry valec oznacuje miesto vzletu.
    const auto & l = executor_->launch();
    auto launch_m = mk(1, visualization_msgs::msg::Marker::CYLINDER, 0.2, 0.6, 1.0);
    launch_m.pose.position.x = l.x;
    launch_m.pose.position.y = l.y;
    launch_m.pose.position.z = l.z + 0.05;
    launch_m.scale.x = launch_m.scale.y = 0.6;
    launch_m.scale.z = 0.1;
    ma.markers.push_back(launch_m);

    // Textove znacky W1 az W4 oznacuju jednotlive rohy stvorca.
    const auto & sq = executor_->square();
    for (std::size_t i = 0; i + 1 < sq.size(); ++i) {   // W1..W4 (posledny = W1)
      auto t = mk(10 + static_cast<int>(i), visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
          1.0, 1.0, 1.0);
      t.pose.position.x = sq[i].x;
      t.pose.position.y = sq[i].y;
      t.pose.position.z = sq[i].z + 0.4;
      t.scale.z = 0.4;
      t.text = "W" + std::to_string(i + 1);
      ma.markers.push_back(t);
    }

    // Textova znacka POI sa zobrazi nad zaujmovym bodom.
    auto poi_t = mk(20, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 1.0, 0.4, 0.4);
    poi_t.pose.position = pm.point;
    poi_t.pose.position.z += 0.6;
    poi_t.scale.z = 0.4;
    poi_t.text = "POI";
    ma.markers.push_back(poi_t);

    // Odoslanie vsetkych znaciek do RViz.
    markers_pub_->publish(ma);
  }

  // Pravidelny vypis diagnostiky: stav, poloha, vyska, GPS a orientacia.
  void printStatus()
  {
    const auto st = vehicle_->state();
    const auto & l = executor_->launch();
    char b[320];

    // Vypocet a formatovanie diagnostickych udajov.
    std::snprintf(b, sizeof(b),
      "[%s] %s armed=%d | od vzletu E %.2f N %.2f U %.2f | rel_alt %s | "
      "lat %.7f lon %.7f | kompas %.0f deg, ENU yaw %.0f deg%s",
      OutdoorExecutor::name(executor_->state()), st.mode.c_str(), st.armed ? 1 : 0,
      st.pos.x - l.x, st.pos.y - l.y, st.pos.z - l.z,
      st.has_rel_alt ? (std::to_string(st.rel_alt).substr(0, 5) + " m").c_str() : "?",
      st.gps.lat_deg, st.gps.lon_deg, st.compass_hdg_deg, lrs_control::rad2deg(st.yaw),
      executor_->poiErrDeg() >= 0.0 ?
      (" | chyba na POI " + std::to_string(executor_->poiErrDeg()).substr(0, 4) + " deg").c_str() :
      executor_->trackErr() >= 0.0 ?
      (" | odchylka od osmicky " + std::to_string(executor_->trackErr()).substr(0, 4) + " m").c_str() :
      "");

    // Vypis zostavenej diagnostiky do ROS logu.
    RCLCPP_INFO(get_logger(), "%s", b);
  }

  // Objekt na komunikaciu s MAVROS a vykonavac misie.
  std::unique_ptr<lrs_control::MavrosVehicle> vehicle_;
  std::unique_ptr<OutdoorExecutor> executor_;

  // Riadiaca skupina, timer a sluzba na prerusenie misie.
  rclcpp::CallbackGroup::SharedPtr control_group_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr abort_srv_;

  // Parametre a publishery pre zaznam a vizualizaciu.
  std::string abort_mode_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr poi_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr flown_pub_;

  // Uchovavanie prejdenej trasy a pomocnych stavovych premennych.
  nav_msgs::msg::Path flown_;
  std::string last_phase_;
  bool plan_published_{false};
  unsigned viz_tick_{0};
  double status_period_{2.0};
  double last_status_{0.0};
  bool finished_{false};
};

// Vstupny bod programu.
// Inicializuje ROS 2, vytvori uzol, spusti spracovanie callbackov
// a po skonceni misie alebo chybe ukonci ROS.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<OutdoorMissionNode> node;
  try {
    // Vytvorenie uzla; pri chybe konfiguracie sa zachyti vynimka.
    node = std::make_shared<OutdoorMissionNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("outdoor_mission"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // MultithreadedExecutor umoznuje spracovavat callbacky vo viacerych vlaknach.
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  // Ukoncenie ROS po skonceni vykonavania uzla.
  rclcpp::shutdown();
  return 0;
}
