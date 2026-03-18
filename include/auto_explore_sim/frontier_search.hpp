#pragma once

#include <optional>
#include <utility>
#include <vector>

#include "auto_explore_sim/exploration_types.hpp"

namespace auto_explore_sim
{

struct FrontierSearchConfig
{
  int min_frontier_size{5};
  double clearance_scale{0.3};
  double frontier_update_radius{3.0};
};

class FrontierSearch
{
public:
  static std::vector<Frontier> search(
    const GridMapView & grid,
    const std::pair<double, double> & robot_xy,
    const std::vector<double> & dist_map,
    const FrontierSearchConfig & config);

  static std::optional<Frontier> revalidate_nearby_frontier(
    const GridMapView & grid,
    const Frontier & frontier,
    const std::pair<double, double> & robot_xy,
    const std::vector<double> & dist_map,
    const FrontierSearchConfig & config,
    double match_radius);
};

}  // namespace auto_explore_sim
