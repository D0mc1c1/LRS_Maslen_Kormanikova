#include "lrs_planning/planning_algorithm.hpp"

#include <iostream>
#include <chrono>
#include <queue>
#include <map>
#include <set>
#include <cmath>
#include <algorithm>

namespace lrs_planning
{

struct AStarNode {
  int x, y, z;
  double g;
  double f;
  
  bool operator>(const AStarNode& other) const {
    return f > other.f;
  }
};

bool astar(
  const VoxelMap& map,
  Vec3 start,
  Vec3 goal,
  double safety_radius,
  double voxel_size,
  Path& out_path,
  double& out_time_ms)
{
  auto start_time = std::chrono::high_resolution_clock::now();
  
  //prevod na voxel indexy a spat
  auto world_to_voxel = [&](const Vec3& p, int& ix, int& iy, int& iz) {
    const auto mn = map.minBound();
    ix = static_cast<int>((p.x - mn.x) / voxel_size);
    iy = static_cast<int>((p.y - mn.y) / voxel_size);
    iz = static_cast<int>((p.z - mn.z) / voxel_size);
  };
  
  auto voxel_to_world = [&](int ix, int iy, int iz) -> Vec3 {
    const auto mn = map.minBound();
    return {
      mn.x + (ix + 0.5) * voxel_size,
      mn.y + (iy + 0.5) * voxel_size,
      mn.z + (iz + 0.5) * voxel_size
    };
  };
  

  auto heuristic = [&](int x, int y, int z, int gx, int gy, int gz) -> double {
    double dx = (x - gx) * voxel_size;
    double dy = (y - gy) * voxel_size;
    double dz = (z - gz) * voxel_size;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
  };
  
  //prevod start a goal
  int start_x, start_y, start_z;
  int goal_x, goal_y, goal_z;
  world_to_voxel(start, start_x, start_y, start_z);
  world_to_voxel(goal, goal_x, goal_y, goal_z);
  
  std::cout << "Start voxel: " << start_x << " " << start_y << " " << start_z << "\n";
  std::cout << "Goal voxel: " << goal_x << " " << goal_y << " " << goal_z << "\n";
  
  if (map.isOccupied(start.x, start.y, start.z, safety_radius)) {
    std::cerr << "Start is occupied!\n";
    return false;
  }
  if (map.isOccupied(goal.x, goal.y, goal.z, safety_radius)) {
    std::cerr << "Goal is occupied!\n";
    return false;
  }
  

  std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open;
  std::map<std::tuple<int,int,int>, double> g_score;
  std::map<std::tuple<int,int,int>, std::tuple<int,int,int>> parent;
  std::set<std::tuple<int,int,int>> closed;
  

  AStarNode start_node{start_x, start_y, start_z, 0.0, 0.0};
  start_node.f = heuristic(start_x, start_y, start_z, goal_x, goal_y, goal_z);
  open.push(start_node);
  g_score[{start_x, start_y, start_z}] = 0.0;
  
  int iterations = 0;
  const int MAX_ITERATIONS = 2000000;  // mriezka ma ~1.6 M buniek

  // vsetkych 26 susedov = kazda kombinacia -1/0/1 okrem (0,0,0)
  int dx[26], dy[26], dz[26];
  double cost[26];  // dlzka kroku v metroch: 0.1 / 0.141 / 0.173 pri voxeli 0.1
  int n = 0;
  for (int a = -1; a <= 1; ++a) {
    for (int b = -1; b <= 1; ++b) {
      for (int c = -1; c <= 1; ++c) {
        if (a == 0 && b == 0 && c == 0) continue;
        dx[n] = a; dy[n] = b; dz[n] = c;
        cost[n] = std::sqrt(double(a * a + b * b + c * c)) * voxel_size;
        ++n;
      }
    }
  }
  
  while (!open.empty() && iterations < MAX_ITERATIONS) {
    iterations++;
    
    AStarNode current = open.top();
    open.pop();
    
    auto current_key = std::make_tuple(current.x, current.y, current.z);
    

    if (closed.count(current_key)) continue;
    closed.insert(current_key);
    

    if (current.x == goal_x && current.y == goal_y && current.z == goal_z) {

      out_path.clear();
      auto key = current_key;
      
      while (parent.count(key)) {
        auto [x, y, z] = key;
        out_path.push_back(voxel_to_world(x, y, z));
        key = parent[key];
      }

      out_path.push_back(start);
      std::reverse(out_path.begin(), out_path.end());
      // posledny bod = presny ciel, nie stred voxelu (odchylka az 8 cm)
      if (out_path.size() >= 2) {
        out_path.back() = goal;
      } else {
        out_path.push_back(goal);
      }
      
      auto end_time = std::chrono::high_resolution_clock::now();
      out_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
      
      std::cout << "A* iterations: " << iterations << "\n";
      return true;
    }
    

    for (int i = 0; i < 26; ++i) {
      int nx = current.x + dx[i];
      int ny = current.y + dy[i];
      int nz = current.z + dz[i];
      
      auto neighbor_key = std::make_tuple(nx, ny, nz);
      
      if (closed.count(neighbor_key)) continue;
      
      Vec3 neighbor_world = voxel_to_world(nx, ny, nz);
      if (map.isOccupied(neighbor_world.x, neighbor_world.y, neighbor_world.z, safety_radius)) {
        continue;
      }
      
      double move_cost = cost[i];

      double new_g = g_score[{current.x, current.y, current.z}] + move_cost;
      

      if (!g_score.count(neighbor_key) || new_g < g_score[neighbor_key]) {
        g_score[neighbor_key] = new_g;
        parent[neighbor_key] = {current.x, current.y, current.z};
        
        double h = heuristic(nx, ny, nz, goal_x, goal_y, goal_z);
        double f = new_g + h;
        
        open.push({nx, ny, nz, new_g, f});
      }
    }
  }
  
  auto end_time = std::chrono::high_resolution_clock::now();
  out_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
  
  std::cerr << "No path found! Iterations: " << iterations << "\n";
  return false;
}

}  // namespace lrs_planning