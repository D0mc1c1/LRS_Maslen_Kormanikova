// ArUco detektor vlastneho roveru (§2, §3).
//
// Odbera spodnu kameru a polohu drona z MAVROS, najde znacky DICT_4X4_50 a publikuje
// target/pose = stred VLASTNEJ znacky (id == marker_id) v lokalnom ENU drona (frame "map").
// Cudziu znacku len zaloguje ("ignorujem cudziu znacku id N"), co je dokaz pre §2.
// Ked je znacka takmer pod dronom, zaloguje "NAD ROVEROM" na overenie §3.
//
// Temy relativne k namespace uzla (drone_N):
//   in:  down_camera/image_raw, down_camera/camera_info, mavros/local_position/pose
//   out: target/pose (PoseStamped, len pri platnej detekcii vlastnej znacky)
//        ~/debug (Image, len ak publish_debug: zelena = vlastna, cervena = cudzia znacka)
//
// Spracovanie obrazu bezi vo vlastnej callback group (image_group), poloha a camera_info
// v sensor_group, takze pomala snimka nezdrzi prijem polohy.
#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core/version.hpp>
#include <opencv2/imgproc.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#define LRS_ARUCO_NEW_API 1
#else
#include <opencv2/aruco.hpp>
#define LRS_ARUCO_NEW_API 0
#endif

#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "lrs_vision/marker_pose.hpp"

namespace
{

int dictionaryFromName(const std::string & name)
{
  static const std::map<std::string, int> dicts = {
    {"DICT_4X4_50", cv::aruco::DICT_4X4_50},
    {"DICT_4X4_100", cv::aruco::DICT_4X4_100},
    {"DICT_5X5_50", cv::aruco::DICT_5X5_50},
    {"DICT_6X6_50", cv::aruco::DICT_6X6_50},
  };
  const auto it = dicts.find(name);
  if (it == dicts.end()) {
    throw std::runtime_error("nepodporovany dictionary '" + name + "'");
  }
  return it->second;
}

cv::Vec3d vec3Param(const std::vector<double> & v, const char * name)
{
  if (v.size() != 3) {
    throw std::runtime_error(std::string(name) + " musi mat 3 cisla");
  }
  return {v[0], v[1], v[2]};
}

}  // namespace

class ArucoDetectorNode : public rclcpp::Node
{
public:
  ArucoDetectorNode()
  : Node("aruco_detector")
  {
    marker_id_ = declare_parameter<int>("marker_id", 1);
    marker_size_ = declare_parameter<double>("marker_size", 0.638);
    const auto dict_name = declare_parameter<std::string>("dictionary", "DICT_4X4_50");
    const auto cam_xyz = declare_parameter<std::vector<double>>("cam_xyz", {0.0, 0.0, -0.03});
    const auto cam_rpy = declare_parameter<std::vector<double>>("cam_rpy_deg", {0.0, 90.0, 0.0});
    pose_max_age_ = declare_parameter<double>("pose_max_age", 0.5);
    over_xy_ = declare_parameter<double>("over_xy", 0.3);
    stats_period_ = declare_parameter<double>("stats_period", 10.0);
    publish_debug_ = declare_parameter<bool>("publish_debug", false);

    if (marker_size_ <= 0.0 || marker_size_ > 2.0) {
      throw std::runtime_error("marker_size mimo rozsahu 0..2 m");
    }
    extr_ = lrs_vision::extrinsicsFromPose(
      vec3Param(cam_xyz, "cam_xyz"), vec3Param(cam_rpy, "cam_rpy_deg"));
    dictionary_ = cv::aruco::getPredefinedDictionary(dictionaryFromName(dict_name));
#if LRS_ARUCO_NEW_API
    detector_ = cv::aruco::ArucoDetector(dictionary_);
#endif

    image_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sensor_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions image_opts, sensor_opts;
    image_opts.callback_group = image_group_;
    sensor_opts.callback_group = sensor_group_;

    // KeepLast(1): ked spracovanie nestiha, starsie snimky sa zahodia, nehromadia
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "down_camera/image_raw", rclcpp::SensorDataQoS().keep_last(1),
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) {onImage(m);}, image_opts);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "down_camera/camera_info", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {onCameraInfo(*m);}, sensor_opts);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "mavros/local_position/pose", rclcpp::SensorDataQoS(),
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr m) {
        const rclcpp::Time t = now();
        std::lock_guard<std::mutex> lk(mutex_);
        // vodorovna rychlost drona z dvoch po sebe iducich poloh (pre odhad chyby z oneskorenia)
        const double dt = pose_rx_.nanoseconds() ? (t - pose_rx_).seconds() : 0.0;
        if (dt > 0.01 && dt < 1.0) {
          const double v = std::hypot(m->pose.position.x - pose_.pose.position.x,
              m->pose.position.y - pose_.pose.position.y) / dt;
          speed_ = 0.8 * speed_ + 0.2 * v;
        }
        pose_ = *m;
        pose_rx_ = t;
      }, sensor_opts);

    target_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("target/pose", 10);
    if (publish_debug_) {
      debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug", rclcpp::SensorDataQoS());
    }

    const auto & R = extr_.R_body_optical;
    RCLCPP_INFO(get_logger(),
      "aruco_detector: namespace '%s', marker_id %d, %s, marker_size %.3f m, OpenCV %s (%s API)",
      get_namespace(), marker_id_, dict_name.c_str(), marker_size_, CV_VERSION,
      LRS_ARUCO_NEW_API ? "ArucoDetector" : "contrib aruco");
    RCLCPP_INFO(get_logger(),
      "kamera v tele (%.3f, %.3f, %.3f) m, R_body_optical [%.0f %.0f %.0f; %.0f %.0f %.0f; %.0f %.0f %.0f]",
      extr_.t_body_cam[0], extr_.t_body_cam[1], extr_.t_body_cam[2],
      R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));
  }

private:
  void onCameraInfo(const sensor_msgs::msg::CameraInfo & m)
  {
    std::array<double, 9> k{};
    std::copy(m.k.begin(), m.k.end(), k.begin());
    const auto cam = lrs_vision::intrinsicsFromCameraInfo(k, m.d);
    std::lock_guard<std::mutex> lk(mutex_);
    if (!cam_.valid && cam.valid) {
      RCLCPP_INFO(get_logger(), "camera_info: %ux%u, fx %.2f, fy %.2f, cx %.1f, cy %.1f",
        m.width, m.height, k[0], k[4], k[2], k[5]);
    }
    cam_ = cam;
  }

  void onImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {
    // cas prijatia snimky v hodinach uzla (nie simulacny cas zo snimky): follow_mission
    // z neho pocita potvrdenie a lost_timeout
    const rclcpp::Time stamp = now();
    const auto t_start = std::chrono::steady_clock::now();
    ++frames_;

    lrs_vision::CameraIntrinsics cam;
    geometry_msgs::msg::PoseStamped pose;
    rclcpp::Time pose_rx;
    double speed = 0.0;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      cam = cam_;
      pose = pose_;
      pose_rx = pose_rx_;
      speed = speed_;
    }
    if (!cam.valid) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cakam na down_camera/camera_info");
      return;
    }
    const double pose_age = (stamp - pose_rx).seconds();
    if (pose_rx.nanoseconds() == 0 || pose_age > pose_max_age_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "poloha drona z MAVROS chyba alebo je stara (%.2f s) -> snimku preskakujem", pose_age);
      return;
    }

    cv::Mat gray;
    try {
      gray = cv_bridge::toCvShare(msg, "mono8")->image;
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
      return;
    }

    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
#if LRS_ARUCO_NEW_API
    detector_.detectMarkers(gray, corners, ids);  // len v image_group, nikdy naraz
#else
    cv::aruco::detectMarkers(gray, dictionary_, corners, ids);
#endif

    const auto & q = pose.pose.orientation;
    const auto & dp = pose.pose.position;
    const cv::Matx33d R_enu_body = lrs_vision::rotationFromQuaternion(q.x, q.y, q.z, q.w);
    const cv::Vec3d drone_pos(dp.x, dp.y, dp.z);
    int own_index = -1;
    std::string own_text;

    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] != marker_id_) {
        ++foreign_;
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "ignorujem cudziu znacku id %d (moja je %d)", ids[i], marker_id_);
        continue;
      }
      const auto p_opt = lrs_vision::markerInOptical(corners[i], cam, marker_size_);
      if (!p_opt) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "solvePnP zlyhal pre znacku %d",
          ids[i]);
        continue;
      }
      const cv::Vec3d p_body = lrs_vision::opticalToBody(*p_opt, extr_);
      const cv::Vec3d p_enu = lrs_vision::bodyToEnu(p_body, drone_pos, R_enu_body);
      ++own_;
      own_index = static_cast<int>(i);
      char buf[96];
      std::snprintf(buf, sizeof(buf), "id %d  %.1f m  ENU %.1f %.1f", ids[i], (*p_opt)[2],
        p_enu[0], p_enu[1]);
      own_text = buf;

      geometry_msgs::msg::PoseStamped out;
      out.header.stamp = stamp;
      out.header.frame_id = "map";
      out.pose.position.x = p_enu[0];
      out.pose.position.y = p_enu[1];
      out.pose.position.z = p_enu[2];
      out.pose.orientation.w = 1.0;
      target_pub_->publish(out);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "vidim svoju znacku %d: ENU (%.2f, %.2f, %.2f), v tele (%.2f, %.2f), vzdialenost %.2f m",
        ids[i], p_enu[0], p_enu[1], p_enu[2], p_body[0], p_body[1], (*p_opt)[2]);

      // §3: takmer priamo pod dronom -> vzdialenost kamera-znacka ma byt vyska drona - 0.32 m
      if (std::abs(p_body[0]) < over_xy_ && std::abs(p_body[1]) < over_xy_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "NAD ROVEROM: vyska drona %.2f m, kamera-znacka %.2f m (ocakavane %.2f), "
          "vyska znacky %.2f m (ocakavane ~0.32), odhad ENU (%.2f, %.2f)",
          dp.z, (*p_opt)[2], dp.z - 0.03 - 0.32, p_enu[2], p_enu[0], p_enu[1]);
      }
      // bez break: aj cudzie znacky za vlastnou sa zapocitaju do statistiky
    }

    if (debug_pub_) {
      publishDebug(msg, gray, corners, ids, own_index, own_text);
    }

    const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t_start).count();
    proc_ms_sum_ += ms;
    proc_ms_max_ = std::max(proc_ms_max_, ms);
    // za cas spracovania sa dron posunie o v*t: o tolko je detekcia "stara" (zadanie, tip 3)
    lag_err_sum_ += speed * ms / 1000.0;
    speed_sum_ += speed;
    const double since = (stamp - last_stats_).seconds();
    if (last_stats_.nanoseconds() == 0) {
      last_stats_ = stamp;
    } else if (since >= stats_period_) {
      const double n = frames_ ? static_cast<double>(frames_) : 1.0;
      RCLCPP_INFO(get_logger(),
        "za %.0f s: snimky %d (%.1f Hz), vlastna znacka %d, cudzia %d, spracovanie %.1f ms "
        "(max %.1f), rychlost drona %.2f m/s -> posun z oneskorenia %.3f m",
        since, frames_, frames_ / since, own_, foreign_, proc_ms_sum_ / n, proc_ms_max_,
        speed_sum_ / n, lag_err_sum_ / n);
      frames_ = own_ = foreign_ = 0;
      proc_ms_sum_ = proc_ms_max_ = lag_err_sum_ = speed_sum_ = 0.0;
      last_stats_ = stamp;
    }
  }

  // ladiaci obraz: vsetky znacky orámovane, vlastna zelena, cudzie cervene, kriz = stred obrazu
  void publishDebug(
    const sensor_msgs::msg::Image::ConstSharedPtr & msg, const cv::Mat & gray,
    const std::vector<std::vector<cv::Point2f>> & corners, const std::vector<int> & ids,
    int own_index, const std::string & own_text)
  {
    cv::Mat img;
    cv::cvtColor(gray, img, cv::COLOR_GRAY2BGR);
    const cv::Point c(img.cols / 2, img.rows / 2);
    cv::line(img, c - cv::Point(12, 0), c + cv::Point(12, 0), cv::Scalar(255, 200, 0), 1);
    cv::line(img, c - cv::Point(0, 12), c + cv::Point(0, 12), cv::Scalar(255, 200, 0), 1);
    for (size_t i = 0; i < ids.size(); ++i) {
      const bool own = static_cast<int>(i) == own_index;
      const cv::Scalar col = own ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 255);
      std::vector<cv::Point> poly;
      for (const auto & p : corners[i]) {
        poly.emplace_back(cvRound(p.x), cvRound(p.y));
      }
      cv::polylines(img, poly, true, col, 2);
      cv::putText(img, own ? own_text : "cudzia id " + std::to_string(ids[i]),
        poly[0] + cv::Point(0, -8), cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1);
    }
    cv::putText(img, std::string(get_namespace()) + "  moja znacka " + std::to_string(marker_id_),
      cv::Point(8, 20), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1);
    auto out = cv_bridge::CvImage(msg->header, "bgr8", img).toImageMsg();
    debug_pub_->publish(*out);
  }

  int marker_id_{1};
  double marker_size_{0.638};
  double pose_max_age_{0.5};
  double over_xy_{0.3};
  double stats_period_{10.0};
  bool publish_debug_{false};
  lrs_vision::CameraExtrinsics extr_;
#if LRS_ARUCO_NEW_API
  cv::aruco::Dictionary dictionary_;
  cv::aruco::ArucoDetector detector_;
#else
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
#endif

  std::mutex mutex_;
  lrs_vision::CameraIntrinsics cam_;
  geometry_msgs::msg::PoseStamped pose_;
  rclcpp::Time pose_rx_{0, 0, RCL_ROS_TIME};
  double speed_{0.0};  // [m/s] vodorovna rychlost drona (vyhladena)

  // statistiky (len v image_group)
  int frames_{0}, own_{0}, foreign_{0};
  double proc_ms_sum_{0.0}, proc_ms_max_{0.0}, lag_err_sum_{0.0}, speed_sum_{0.0};
  rclcpp::Time last_stats_{0, 0, RCL_ROS_TIME};

  rclcpp::CallbackGroup::SharedPtr image_group_, sensor_group_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDetectorNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
