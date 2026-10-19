#pragma once
#include <cstddef>
#include <cstdint> 
#include <string>
#include <vector>

#include "lrs_planning/types.hpp"

namespace lrs_planning
{

class VoxelMap
{
public:
  bool load(const std::string & pcd_path, double voxel_size);

  // plny kvader (regal): prekazka
  void addSolidBox(const Vec3 & lo, const Vec3 & hi) {boxes_.push_back({lo, hi});}

  bool isOccupied(double x, double y, double z, double safety_radius = 0.0) const;

  // len mracno bodov (bez boxov regalov a stropu, bez inflacie) - pre vizualizaciu
  bool isOccupiedCloud(double x, double y, double z) const;
  bool isOccupied(const Vec3 & p, double safety_radius = 0.0) const;

  // min a max suradnice
  std::size_t pointCount() const {return point_count_;}
  Vec3 minBound() const {return min_;}
  Vec3 maxBound() const {return max_;}

  // pocet voxelov v troch smeroch
  double voxelSize() const {return voxel_;}
  size_t gridX() const {return nx_;}
  size_t gridY() const {return ny_;}
  size_t gridZ() const {return nz_;}

private:
  std::size_t point_count_{0};
  double voxel_{0.1};
  Vec3 min_{}, max_{};

  size_t nx_{0}, ny_{0}, nz_{0};
  std::vector<uint8_t> occupancy_;  // 1 = obsadene, 0 = volne

  struct Box { Vec3 lo, hi; };
  std::vector<Box> boxes_;

  bool coordToIndex(double x, double y, double z, int& ix, int& iy, int& iz) const;
  size_t indexToLinear(int ix, int iy, int iz) const;
};

}
