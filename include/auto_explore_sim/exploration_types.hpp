#pragma once

#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace auto_explore_sim
{

struct Frontier
{
  int size{0};
  double min_distance{std::numeric_limits<double>::infinity()};
  double heuristic_distance{std::numeric_limits<double>::infinity()};
  double cost{0.0};
  double centroid_x{0.0};
  double centroid_y{0.0};
};

struct GridMapView
{
  const std::vector<int8_t> * cells{nullptr};
  int width{0};
  int height{0};
  double resolution{0.05};
  double origin_x{0.0};
  double origin_y{0.0};

  bool valid() const
  {
    return cells != nullptr && !cells->empty() && width > 0 && height > 0;
  }

  bool in_bounds(int x, int y) const
  {
    return x >= 0 && x < width && y >= 0 && y < height;
  }

  int index(int x, int y) const
  {
    return y * width + x;
  }

  int8_t value(int x, int y) const
  {
    return (*cells)[static_cast<size_t>(index(x, y))];
  }

  bool is_obstacle(int idx) const
  {
    return (*cells)[static_cast<size_t>(idx)] > 50;
  }

  std::pair<double, double> cell_world(int x, int y) const
  {
    return {
      static_cast<double>(x) * resolution + origin_x,
      static_cast<double>(y) * resolution + origin_y};
  }

  std::pair<double, double> cell_center_world(int x, int y) const
  {
    return {
      (static_cast<double>(x) + 0.5) * resolution + origin_x,
      (static_cast<double>(y) + 0.5) * resolution + origin_y};
  }
};

}  // namespace auto_explore_sim
