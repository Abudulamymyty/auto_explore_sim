#pragma once

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "auto_explore_sim/exploration_types.hpp"

namespace auto_explore_sim
{

struct GvdConfig
{
  int min_clearance{3};
  double snap_radius{2.0};
  double distance_tolerance_cells{1.0};
  double max_site_direction_dot{0.7};
  bool allow_bridge{false};
  double heuristic_chain_sample_step{0.5};
  double heuristic_chain_max_offset{1.0};
};

struct GvdData
{
  std::vector<double> dist_map;
  std::vector<int> label_map;
  std::vector<int> component_map;
  std::vector<uint8_t> mask;
  std::vector<int> site_xs;
  std::vector<int> site_ys;
  std::vector<double> world_x;
  std::vector<double> world_y;
  size_t boundary_site_count{0};
  int gvd_cell_count{0};
  int component_count{0};

  bool empty() const
  {
    return mask.empty();
  }

  std::pair<double, double> world_point(const GridMapView & grid, int x, int y) const
  {
    const int idx = grid.index(x, y);
    if (idx >= 0 && static_cast<size_t>(idx) < world_x.size()) {
      return {world_x[static_cast<size_t>(idx)], world_y[static_cast<size_t>(idx)]};
    }
    return grid.cell_center_world(x, y);
  }
};

struct GvdSnapResult
{
  std::pair<double, double> point;
  bool found{false};
  bool moved{false};
};

struct GvdPathResult
{
  std::vector<std::pair<double, double>> path;
  int skeleton_cell_count{0};
  int explored_cells{0};
  bool used_bridge{false};
  bool used_heuristic_chain{false};
  int bridge_trigger_explored_cells{0};
};

struct GvdComponentEntryResult
{
  std::pair<double, double> point;
  int component_id{-1};
  double min_clearance_cells{0.0};
  int explored_cells{0};
};

struct FreePathResult
{
  std::pair<double, double> goal;
  double min_clearance_cells{0.0};
  int explored_cells{0};
  bool adjusted_goal{false};
};

class GvdMap
{
public:
  static GvdData build(const GridMapView & grid, const GvdConfig & config);

  static GvdSnapResult snap_to_gvd(
    const GridMapView & grid,
    const GvdData & gvd,
    const std::pair<double, double> & point,
    double snap_radius_m);

  static std::optional<GvdPathResult> find_path(
    const GridMapView & grid,
    const GvdData & gvd,
    const std::pair<double, double> & robot_xy,
    const std::pair<double, double> & goal_xy,
    const GvdConfig & config);

  static std::optional<GvdComponentEntryResult> find_goal_component_entry(
    const GridMapView & grid,
    const GvdData & gvd,
    const std::pair<double, double> & robot_xy,
    const std::pair<double, double> & goal_xy,
    const GvdConfig & config);

  static std::optional<FreePathResult> find_safe_direct_goal(
    const GridMapView & grid,
    const GvdData & gvd,
    const std::pair<double, double> & robot_xy,
    const std::pair<double, double> & goal_xy,
    const GvdConfig & config,
    double goal_search_radius_m);
};

}  // namespace auto_explore_sim
