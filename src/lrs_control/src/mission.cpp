#include "lrs_control/mission.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace lrs_control
{

namespace
{

std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) {
    return "";
  }
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return s;
}

[[noreturn]] void fail(const std::string & path, int line, const std::string & msg)
{
  throw std::runtime_error(path + ":" + std::to_string(line) + ": " + msg);
}

double toDouble(const std::string & s, const std::string & path, int line, const char * what)
{
  try {
    std::size_t used = 0;
    const double v = std::stod(s, &used);
    if (used != s.size()) {
      throw std::invalid_argument(s);
    }
    return v;
  } catch (const std::exception &) {
    fail(path, line, std::string("neplatne cislo v stlpci ") + what + ": '" + s + "'");
  }
}

}  // namespace

Mission loadMission(const std::string & path)
{
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("nepodarilo sa otvorit misiu: " + path);
  }

  Mission mission;
  std::string raw;
  int line = 0;
  while (std::getline(f, raw)) {
    ++line;
    const std::string s = trim(raw);
    if (s.empty() || s[0] == '#') {
      continue;
    }

    std::vector<std::string> cols;
    std::stringstream ss(s);
    std::string c;
    while (std::getline(ss, c, ',')) {
      cols.push_back(trim(c));
    }
    if (!cols.empty() && lower(cols[0]) == "x") {
      continue;  // hlavicka
    }
    if (cols.size() != 5) {
      fail(path, line, "ocakavam 5 stlpcov x,y,z,precision,task, nasiel som " +
        std::to_string(cols.size()));
    }

    Waypoint wp;
    wp.line = line;
    wp.p = {toDouble(cols[0], path, line, "x"),
      toDouble(cols[1], path, line, "y"),
      toDouble(cols[2], path, line, "z")};

    const std::string prec = lower(cols[3]);
    if (prec == "hard") {
      wp.precision = Precision::Hard;
    } else if (prec == "soft") {
      wp.precision = Precision::Soft;
    } else {
      fail(path, line, "neznama precision '" + cols[3] + "' (hard/soft)");
    }

    std::string task = lower(cols[4]);
    task.erase(std::remove_if(task.begin(), task.end(),
      [](unsigned char ch) {return std::isspace(ch);}), task.end());
    if (task.empty() || task == "-") {
      wp.task = TaskType::None;
    } else if (task == "takeoff") {
      wp.task = TaskType::Takeoff;
    } else if (task == "land") {
      wp.task = TaskType::Land;
    } else if (task == "landtakeoff") {
      wp.task = TaskType::LandTakeoff;
    } else if (task.rfind("yaw", 0) == 0 && task.size() > 3) {
      wp.task = TaskType::Yaw;
      wp.yaw_deg = toDouble(task.substr(3), path, line, "task (yaw)");
    } else {
      fail(path, line, "neznamy task '" + cols[4] + "'");
    }
    mission.push_back(wp);
  }

  if (mission.empty()) {
    throw std::runtime_error("misia neobsahuje ziadny waypoint: " + path);
  }
  return mission;
}

std::string toString(Precision p) {return p == Precision::Hard ? "hard" : "soft";}

std::string toString(TaskType t)
{
  switch (t) {
    case TaskType::None: return "-";
    case TaskType::Takeoff: return "takeoff";
    case TaskType::Land: return "land";
    case TaskType::LandTakeoff: return "landtakeoff";
    case TaskType::Yaw: return "yaw";
  }
  return "?";
}

std::string describe(const Waypoint & wp)
{
  char buf[128];
  std::snprintf(buf, sizeof(buf), "(%.2f, %.2f, %.2f) %s %s", wp.p.x, wp.p.y, wp.p.z,
    toString(wp.precision).c_str(), toString(wp.task).c_str());
  std::string s(buf);
  if (wp.task == TaskType::Yaw) {
    char y[32];
    std::snprintf(y, sizeof(y), "%g", wp.yaw_deg);
    s += y;
  }
  return s;
}

}  // namespace lrs_control
