#include "auto_explore_sim/gvd_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

namespace auto_explore_sim
{
namespace
{

struct IndexedCost
{
  double cost;
  int counter;
  int index;

  bool operator<(const IndexedCost & other) const
  {
    if (cost != other.cost) {
      return cost > other.cost;
    }
    return counter > other.counter;
  }
};

struct WavefrontEntry
{
  double cost;
  int counter;
  int index;
  int site_id;

  bool operator<(const WavefrontEntry & other) const
  {
    if (cost != other.cost) {
      return cost > other.cost;
    }
    return counter > other.counter;
  }
};

constexpr std::array<std::pair<int, int>, 8> kNeighbors8{{
  std::pair<int, int>{-1, -1},
  std::pair<int, int>{-1, 0},
  std::pair<int, int>{-1, 1},
  std::pair<int, int>{0, -1},
  std::pair<int, int>{0, 1},
  std::pair<int, int>{1, -1},
  std::pair<int, int>{1, 0},
  std::pair<int, int>{1, 1},
}};

constexpr double kLowClearancePenaltyScale = 0.5;
constexpr double kDefaultGoalSearchRadius = 1.0;

bool is_obstacle_boundary_cell(const GridMapView & grid, int x, int y)
{
  const int idx0 = grid.index(x, y);
  if (!grid.is_obstacle(idx0)) {
    return false;
  }
  for (const auto & [dx, dy] : kNeighbors8) {
    const int nx = x + dx;
    const int ny = y + dy;
    if (!grid.in_bounds(nx, ny)) {
      return true;
    }
    if (!grid.is_obstacle(grid.index(nx, ny))) {
      return true;
    }
  }
  return false;
}

double distance_to_site_cells(const GvdData & gvd, int x, int y, int site_id)
{
  if (site_id < 0 || static_cast<size_t>(site_id) >= gvd.site_xs.size()) {
    return std::numeric_limits<double>::infinity();
  }
  return std::hypot(
    static_cast<double>(x - gvd.site_xs[static_cast<size_t>(site_id)]),
    static_cast<double>(y - gvd.site_ys[static_cast<size_t>(site_id)]));
}

std::pair<double, double> project_to_voronoi_bisector(
  const GridMapView & grid,
  const GvdData & gvd,
  int x,
  int y,
  int site_a,
  int site_b)
{
  const auto point = grid.cell_center_world(x, y);
  if (site_a < 0 || site_b < 0 ||
    static_cast<size_t>(site_a) >= gvd.site_xs.size() ||
    static_cast<size_t>(site_b) >= gvd.site_xs.size() ||
    site_a == site_b)
  {
    return point;
  }

  const auto site_a_world = grid.cell_center_world(
    gvd.site_xs[static_cast<size_t>(site_a)],
    gvd.site_ys[static_cast<size_t>(site_a)]);
  const auto site_b_world = grid.cell_center_world(
    gvd.site_xs[static_cast<size_t>(site_b)],
    gvd.site_ys[static_cast<size_t>(site_b)]);
  const double dx = site_b_world.first - site_a_world.first;
  const double dy = site_b_world.second - site_a_world.second;
  const double length = std::hypot(dx, dy);
  if (length <= 1e-9) {
    return point;
  }

  const double nx = dx / length;
  const double ny = dy / length;
  const double mid_x = 0.5 * (site_a_world.first + site_b_world.first);
  const double mid_y = 0.5 * (site_a_world.second + site_b_world.second);
  const double shift = (mid_x - point.first) * nx + (mid_y - point.second) * ny;
  return {point.first + shift * nx, point.second + shift * ny};
}

std::optional<std::pair<int, int>> nearest_gvd_cell(
  const GridMapView & grid,
  const GvdData & gvd,
  double wx,
  double wy,
  double max_radius_m = 1.5)
{
  if (gvd.empty()) {
    return std::nullopt;
  }

  int px = std::clamp(static_cast<int>((wx - grid.origin_x) / grid.resolution), 0, grid.width - 1);
  int py = std::clamp(static_cast<int>((wy - grid.origin_y) / grid.resolution), 0, grid.height - 1);
  if (gvd.mask[static_cast<size_t>(grid.index(px, py))]) {
    return std::make_pair(px, py);
  }

  const int max_cells = static_cast<int>(max_radius_m / grid.resolution);
  bool found = false;
  int best_x = px;
  int best_y = py;
  int best_d2 = std::numeric_limits<int>::max();
  for (int y = std::max(0, py - max_cells); y <= std::min(grid.height - 1, py + max_cells); ++y) {
    for (int x = std::max(0, px - max_cells); x <= std::min(grid.width - 1, px + max_cells); ++x) {
      if (!gvd.mask[static_cast<size_t>(grid.index(x, y))]) {
        continue;
      }
      const int d2 = (x - px) * (x - px) + (y - py) * (y - py);
      if (d2 < best_d2) {
        best_d2 = d2;
        best_x = x;
        best_y = y;
        found = true;
      }
    }
  }

  if (!found) {
    return std::nullopt;
  }
  return std::make_pair(best_x, best_y);
}

std::optional<std::pair<int, int>> nearest_free_cell(
  const GridMapView & grid,
  double wx,
  double wy,
  double max_radius_m = kDefaultGoalSearchRadius)
{
  if (!grid.valid()) {
    return std::nullopt;
  }

  int px = std::clamp(static_cast<int>((wx - grid.origin_x) / grid.resolution), 0, grid.width - 1);
  int py = std::clamp(static_cast<int>((wy - grid.origin_y) / grid.resolution), 0, grid.height - 1);
  if ((*grid.cells)[static_cast<size_t>(grid.index(px, py))] == 0) {
    return std::make_pair(px, py);
  }

  const int max_cells = std::max(1, static_cast<int>(std::ceil(max_radius_m / grid.resolution)));
  bool found = false;
  int best_x = px;
  int best_y = py;
  int best_d2 = std::numeric_limits<int>::max();
  for (int y = std::max(0, py - max_cells); y <= std::min(grid.height - 1, py + max_cells); ++y) {
    for (int x = std::max(0, px - max_cells); x <= std::min(grid.width - 1, px + max_cells); ++x) {
      if ((*grid.cells)[static_cast<size_t>(grid.index(x, y))] != 0) {
        continue;
      }
      const int d2 = (x - px) * (x - px) + (y - py) * (y - py);
      if (d2 < best_d2) {
        best_d2 = d2;
        best_x = x;
        best_y = y;
        found = true;
      }
    }
  }

  if (!found) {
    return std::nullopt;
  }
  return std::make_pair(best_x, best_y);
}

std::vector<int> reconstruct_path_cells(
  int start_idx,
  int end_idx,
  const std::vector<int> & came_from)
{
  std::vector<int> path_cells;
  for (int node = end_idx; node != -1; node = came_from[static_cast<size_t>(node)]) {
    path_cells.push_back(node);
    if (node == start_idx) {
      break;
    }
  }
  std::reverse(path_cells.begin(), path_cells.end());
  return path_cells;
}

double min_clearance_on_path(const GvdData & gvd, const std::vector<int> & path_cells)
{
  double min_clearance = std::numeric_limits<double>::infinity();
  for (int idx : path_cells) {
    min_clearance = std::min(min_clearance, gvd.dist_map[static_cast<size_t>(idx)]);
  }
  if (!std::isfinite(min_clearance)) {
    return 0.0;
  }
  return min_clearance;
}

std::optional<std::vector<int>> find_cell_path(
  const GridMapView & grid,
  const GvdData & gvd,
  int start_idx,
  int end_idx,
  bool skeleton_only,
  int min_clearance,
  int & explored_cells)
{
  const int total = grid.width * grid.height;
  std::vector<double> g_score(static_cast<size_t>(total), std::numeric_limits<double>::infinity());
  std::vector<int> came_from(static_cast<size_t>(total), -1);
  std::priority_queue<IndexedCost> open_set;

  g_score[static_cast<size_t>(start_idx)] = 0.0;
  int counter = 0;
  open_set.push({0.0, counter, start_idx});

  const int ex = end_idx % grid.width;
  const int ey = end_idx / grid.width;

  while (!open_set.empty()) {
    const auto current = open_set.top();
    open_set.pop();
    const int idx0 = current.index;
    ++explored_cells;
    if (idx0 == end_idx) {
      std::vector<int> path_cells;
      for (int node = end_idx; node != -1; node = came_from[static_cast<size_t>(node)]) {
        path_cells.push_back(node);
        if (node == start_idx) {
          break;
        }
      }
      std::reverse(path_cells.begin(), path_cells.end());
      return path_cells;
    }

    const int cx = idx0 % grid.width;
    const int cy = idx0 / grid.width;
    for (const auto & [dx, dy] : kNeighbors8) {
      const int nx = cx + dx;
      const int ny = cy + dy;
      if (!grid.in_bounds(nx, ny)) {
        continue;
      }

      const int nidx = grid.index(nx, ny);
      if ((*grid.cells)[static_cast<size_t>(nidx)] != 0) {
        continue;
      }
      double step_cost = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
      if (!gvd.mask[static_cast<size_t>(nidx)]) {
        if (skeleton_only) {
          continue;
        }
      }
      const double clearance_deficit = std::max(
        0.0, static_cast<double>(min_clearance) - gvd.dist_map[static_cast<size_t>(nidx)]);
      step_cost += clearance_deficit * kLowClearancePenaltyScale;

      const double tentative_g = g_score[static_cast<size_t>(idx0)] + step_cost;
      if (tentative_g < g_score[static_cast<size_t>(nidx)]) {
        came_from[static_cast<size_t>(nidx)] = idx0;
        g_score[static_cast<size_t>(nidx)] = tentative_g;
        const double heur = std::hypot(nx - ex, ny - ey);
        open_set.push({tentative_g + heur, ++counter, nidx});
      }
    }
  }

  return std::nullopt;
}

std::optional<std::vector<int>> find_path_to_component_entry(
  const GridMapView & grid,
  const GvdData & gvd,
  int start_idx,
  int target_component,
  int goal_hint_idx,
  int min_clearance,
  int & explored_cells)
{
  const int total = grid.width * grid.height;
  std::vector<double> g_score(static_cast<size_t>(total), std::numeric_limits<double>::infinity());
  std::vector<int> came_from(static_cast<size_t>(total), -1);
  std::priority_queue<IndexedCost> open_set;

  const int goal_hint_x = goal_hint_idx % grid.width;
  const int goal_hint_y = goal_hint_idx / grid.width;

  g_score[static_cast<size_t>(start_idx)] = 0.0;
  int counter = 0;
  const int start_x = start_idx % grid.width;
  const int start_y = start_idx / grid.width;
  const double start_heur = std::hypot(start_x - goal_hint_x, start_y - goal_hint_y);
  open_set.push({start_heur, counter, start_idx});

  while (!open_set.empty()) {
    const auto current = open_set.top();
    open_set.pop();
    const int idx0 = current.index;
    const int cx = idx0 % grid.width;
    const int cy = idx0 / grid.width;
    const double current_heur = std::hypot(cx - goal_hint_x, cy - goal_hint_y);
    if (current.cost > g_score[static_cast<size_t>(idx0)] + current_heur + 1e-9) {
      continue;
    }

    ++explored_cells;
    if (gvd.mask[static_cast<size_t>(idx0)] &&
      gvd.component_map[static_cast<size_t>(idx0)] == target_component)
    {
      return reconstruct_path_cells(start_idx, idx0, came_from);
    }

    for (const auto & [dx, dy] : kNeighbors8) {
      const int nx = cx + dx;
      const int ny = cy + dy;
      if (!grid.in_bounds(nx, ny)) {
        continue;
      }

      const int nidx = grid.index(nx, ny);
      if ((*grid.cells)[static_cast<size_t>(nidx)] != 0) {
        continue;
      }

      double step_cost = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
      const double clearance_deficit = std::max(
        0.0, static_cast<double>(min_clearance) - gvd.dist_map[static_cast<size_t>(nidx)]);
      step_cost += clearance_deficit * kLowClearancePenaltyScale;

      const double tentative_g = g_score[static_cast<size_t>(idx0)] + step_cost;
      if (tentative_g < g_score[static_cast<size_t>(nidx)]) {
        came_from[static_cast<size_t>(nidx)] = idx0;
        g_score[static_cast<size_t>(nidx)] = tentative_g;
        const double heur = std::hypot(nx - goal_hint_x, ny - goal_hint_y);
        open_set.push({tentative_g + heur, ++counter, nidx});
      }
    }
  }

  return std::nullopt;
}

}  // namespace

GvdData GvdMap::build(const GridMapView & grid, const GvdConfig & config)
{
  GvdData gvd;
  const int total = grid.width * grid.height;
  if (!grid.valid() || total <= 0) {
    return gvd;
  }

  gvd.dist_map.assign(static_cast<size_t>(total), std::numeric_limits<double>::infinity());
  gvd.label_map.assign(static_cast<size_t>(total), -1);
  gvd.component_map.assign(static_cast<size_t>(total), -1);
  gvd.mask.assign(static_cast<size_t>(total), 0);
  gvd.world_x.assign(static_cast<size_t>(total), 0.0);
  gvd.world_y.assign(static_cast<size_t>(total), 0.0);

  std::priority_queue<WavefrontEntry> open_set;
  int counter = 0;
  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const int idx = grid.index(x, y);
      const auto center = grid.cell_center_world(x, y);
      gvd.world_x[static_cast<size_t>(idx)] = center.first;
      gvd.world_y[static_cast<size_t>(idx)] = center.second;

      if (!grid.is_obstacle(idx) || !is_obstacle_boundary_cell(grid, x, y)) {
        continue;
      }
      const int site_id = static_cast<int>(gvd.site_xs.size());
      gvd.site_xs.push_back(x);
      gvd.site_ys.push_back(y);
      gvd.dist_map[static_cast<size_t>(idx)] = 0.0;
      gvd.label_map[static_cast<size_t>(idx)] = site_id;
      open_set.push({0.0, counter++, idx, site_id});
    }
  }
  gvd.boundary_site_count = gvd.site_xs.size();

  while (!open_set.empty()) {
    const auto current = open_set.top();
    open_set.pop();
    const int idx0 = current.index;
    if (current.cost > gvd.dist_map[static_cast<size_t>(idx0)] + 1e-9 ||
      gvd.label_map[static_cast<size_t>(idx0)] != current.site_id)
    {
      continue;
    }

    const int cx = idx0 % grid.width;
    const int cy = idx0 / grid.width;
    for (const auto & [dx, dy] : kNeighbors8) {
      const int nx = cx + dx;
      const int ny = cy + dy;
      if (!grid.in_bounds(nx, ny)) {
        continue;
      }
      const int nidx = grid.index(nx, ny);
      if (grid.is_obstacle(nidx)) {
        continue;
      }

      const double candidate = distance_to_site_cells(gvd, nx, ny, current.site_id);
      double & current_dist = gvd.dist_map[static_cast<size_t>(nidx)];
      int & current_label = gvd.label_map[static_cast<size_t>(nidx)];
      if (candidate + 1e-9 < current_dist ||
        (std::abs(candidate - current_dist) <= 1e-9 &&
        (current_label < 0 || current.site_id < current_label)))
      {
        current_dist = candidate;
        current_label = current.site_id;
        open_set.push({candidate, counter++, nidx, current.site_id});
      }
    }
  }

  for (auto & value : gvd.dist_map) {
    if (!std::isfinite(value)) {
      value = 0.0;
    }
  }

  for (int y = 0; y < grid.height; ++y) {
    for (int x = 0; x < grid.width; ++x) {
      const int idx0 = grid.index(x, y);
      if ((*grid.cells)[static_cast<size_t>(idx0)] != 0 ||
        gvd.dist_map[static_cast<size_t>(idx0)] < static_cast<double>(config.min_clearance))
      {
        continue;
      }

      std::array<int, 9> candidate_sites{};
      int candidate_count = 0;
      auto add_candidate = [&](int site_id) {
          if (site_id < 0) {
            return;
          }
          for (int i = 0; i < candidate_count; ++i) {
            if (candidate_sites[static_cast<size_t>(i)] == site_id) {
              return;
            }
          }
          if (candidate_count < static_cast<int>(candidate_sites.size())) {
            candidate_sites[static_cast<size_t>(candidate_count++)] = site_id;
          }
        };

      add_candidate(gvd.label_map[static_cast<size_t>(idx0)]);
      for (const auto & [dx, dy] : kNeighbors8) {
        const int nx = x + dx;
        const int ny = y + dy;
        if (!grid.in_bounds(nx, ny)) {
          continue;
        }
        add_candidate(gvd.label_map[static_cast<size_t>(grid.index(nx, ny))]);
      }

      if (candidate_count < 2) {
        continue;
      }

      double best_pair_diff = std::numeric_limits<double>::infinity();
      int best_site = -1;
      int second_site = -1;
      for (int i = 0; i < candidate_count; ++i) {
        const int site_a = candidate_sites[static_cast<size_t>(i)];
        const double dist_a = distance_to_site_cells(gvd, x, y, site_a);
        for (int j = i + 1; j < candidate_count; ++j) {
          const int site_b = candidate_sites[static_cast<size_t>(j)];
          const double dist_b = distance_to_site_cells(gvd, x, y, site_b);
          const double diff = std::abs(dist_a - dist_b);
          if (diff > config.distance_tolerance_cells || diff >= best_pair_diff) {
            continue;
          }

          const double ax = static_cast<double>(gvd.site_xs[static_cast<size_t>(site_a)] - x);
          const double ay = static_cast<double>(gvd.site_ys[static_cast<size_t>(site_a)] - y);
          const double bx = static_cast<double>(gvd.site_xs[static_cast<size_t>(site_b)] - x);
          const double by = static_cast<double>(gvd.site_ys[static_cast<size_t>(site_b)] - y);
          const double a_norm = std::hypot(ax, ay);
          const double b_norm = std::hypot(bx, by);
          if (a_norm <= 1e-9 || b_norm <= 1e-9) {
            continue;
          }
          const double dot = (ax * bx + ay * by) / (a_norm * b_norm);
          if (dot > config.max_site_direction_dot) {
            continue;
          }

          best_pair_diff = diff;
          best_site = site_a;
          second_site = site_b;
        }
      }

      if (best_site < 0 || second_site < 0) {
        continue;
      }

      gvd.mask[static_cast<size_t>(idx0)] = 1;
      const auto projected = project_to_voronoi_bisector(grid, gvd, x, y, best_site, second_site);
      gvd.world_x[static_cast<size_t>(idx0)] = projected.first;
      gvd.world_y[static_cast<size_t>(idx0)] = projected.second;
      ++gvd.gvd_cell_count;
    }
  }

  std::queue<int> component_queue;
  for (int idx0 = 0; idx0 < total; ++idx0) {
    if (!gvd.mask[static_cast<size_t>(idx0)] ||
      gvd.component_map[static_cast<size_t>(idx0)] >= 0)
    {
      continue;
    }

    const int component_id = gvd.component_count++;
    gvd.component_map[static_cast<size_t>(idx0)] = component_id;
    component_queue.push(idx0);

    while (!component_queue.empty()) {
      const int cell = component_queue.front();
      component_queue.pop();
      const int cx = cell % grid.width;
      const int cy = cell / grid.width;
      for (const auto & [dx, dy] : kNeighbors8) {
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (!grid.in_bounds(nx, ny)) {
          continue;
        }

        const int nidx = grid.index(nx, ny);
        if (!gvd.mask[static_cast<size_t>(nidx)] ||
          gvd.component_map[static_cast<size_t>(nidx)] >= 0)
        {
          continue;
        }

        gvd.component_map[static_cast<size_t>(nidx)] = component_id;
        component_queue.push(nidx);
      }
    }
  }

  return gvd;
}

GvdSnapResult GvdMap::snap_to_gvd(
  const GridMapView & grid,
  const GvdData & gvd,
  const std::pair<double, double> & point,
  double snap_radius_m)
{
  GvdSnapResult result{point, false, false};
  if (!grid.valid() || gvd.empty()) {
    return result;
  }

  int px = static_cast<int>((point.first - grid.origin_x) / grid.resolution);
  int py = static_cast<int>((point.second - grid.origin_y) / grid.resolution);
  px = std::clamp(px, 0, grid.width - 1);
  py = std::clamp(py, 0, grid.height - 1);
  if (gvd.mask[static_cast<size_t>(grid.index(px, py))]) {
    result.point = gvd.world_point(grid, px, py);
    result.found = true;
    result.moved = std::hypot(
      result.point.first - point.first, result.point.second - point.second) > 1e-3;
    return result;
  }

  const int max_cells = static_cast<int>(snap_radius_m / grid.resolution);
  double best_dist = std::numeric_limits<double>::infinity();
  for (int y = std::max(0, py - max_cells); y <= std::min(grid.height - 1, py + max_cells); ++y) {
    for (int x = std::max(0, px - max_cells); x <= std::min(grid.width - 1, px + max_cells); ++x) {
      if (!gvd.mask[static_cast<size_t>(grid.index(x, y))]) {
        continue;
      }
      const auto [wx, wy] = gvd.world_point(grid, x, y);
      const double cand_to_frontier = std::hypot(wx - point.first, wy - point.second);
      if (cand_to_frontier > snap_radius_m) {
        continue;
      }
      if (cand_to_frontier < best_dist) {
        best_dist = cand_to_frontier;
        result.point = {wx, wy};
        result.found = true;
      }
    }
  }

  if (result.found) {
    result.moved = std::hypot(
      result.point.first - point.first, result.point.second - point.second) > 1e-3;
  }

  return result;
}

GvdSnapResult GvdMap::snap_to_same_component_gvd(
  const GridMapView & grid,
  const GvdData & gvd,
  const std::pair<double, double> & robot_xy,
  const std::pair<double, double> & point,
  double snap_radius_m)
{
  GvdSnapResult result{point, false, false};
  if (!grid.valid() || gvd.empty() || gvd.component_map.empty()) {
    return result;
  }

  const auto robot_cell = nearest_gvd_cell(
    grid, gvd, robot_xy.first, robot_xy.second, std::max(1.5, snap_radius_m));
  if (!robot_cell) {
    return result;
  }

  const int robot_idx = grid.index(robot_cell->first, robot_cell->second);
  const int robot_component = gvd.component_map[static_cast<size_t>(robot_idx)];
  if (robot_component < 0) {
    return result;
  }

  int px = static_cast<int>((point.first - grid.origin_x) / grid.resolution);
  int py = static_cast<int>((point.second - grid.origin_y) / grid.resolution);
  px = std::clamp(px, 0, grid.width - 1);
  py = std::clamp(py, 0, grid.height - 1);

  const int point_idx = grid.index(px, py);
  if (
    gvd.mask[static_cast<size_t>(point_idx)] &&
    gvd.component_map[static_cast<size_t>(point_idx)] == robot_component)
  {
    result.point = gvd.world_point(grid, px, py);
    result.found = true;
    result.moved = std::hypot(
      result.point.first - point.first, result.point.second - point.second) > 1e-3;
    return result;
  }

  const int max_cells = static_cast<int>(snap_radius_m / grid.resolution);
  double best_dist = std::numeric_limits<double>::infinity();
  for (int y = std::max(0, py - max_cells); y <= std::min(grid.height - 1, py + max_cells); ++y) {
    for (int x = std::max(0, px - max_cells); x <= std::min(grid.width - 1, px + max_cells); ++x) {
      const int idx = grid.index(x, y);
      if (
        !gvd.mask[static_cast<size_t>(idx)] ||
        gvd.component_map[static_cast<size_t>(idx)] != robot_component)
      {
        continue;
      }

      const auto [wx, wy] = gvd.world_point(grid, x, y);
      const double candidate_distance = std::hypot(wx - point.first, wy - point.second);
      if (candidate_distance > snap_radius_m) {
        continue;
      }
      if (candidate_distance < best_dist) {
        best_dist = candidate_distance;
        result.point = {wx, wy};
        result.found = true;
      }
    }
  }

  if (result.found) {
    result.moved = std::hypot(
      result.point.first - point.first, result.point.second - point.second) > 1e-3;
  }

  return result;
}

std::optional<GvdPathResult> GvdMap::find_path(
  const GridMapView & grid,
  const GvdData & gvd,
  const std::pair<double, double> & robot_xy,
  const std::pair<double, double> & goal_xy,
  const GvdConfig & config)
{
  if (!grid.valid() || gvd.empty()) {
    return std::nullopt;
  }

  const auto start = nearest_gvd_cell(grid, gvd, robot_xy.first, robot_xy.second);
  const auto end = nearest_gvd_cell(grid, gvd, goal_xy.first, goal_xy.second);
  if (!start || !end || *start == *end) {
    return std::nullopt;
  }

  const int start_idx = grid.index(start->first, start->second);
  const int end_idx = grid.index(end->first, end->second);
  int explored = 0;
  bool used_bridge = false;
  bool used_heuristic_chain = false;
  int bridge_trigger_explored_cells = 0;
  auto path_cells = find_cell_path(
    grid, gvd, start_idx, end_idx, true, config.min_clearance, explored);
  if (!path_cells && config.allow_bridge) {
    used_bridge = true;
    bridge_trigger_explored_cells = explored;
    explored = 0;
    path_cells = find_cell_path(
      grid, gvd, start_idx, end_idx, false, config.min_clearance, explored);
  }

  if (!path_cells) {
    return std::nullopt;
  }

  GvdPathResult result;
  result.used_bridge = used_bridge;
  result.used_heuristic_chain = used_heuristic_chain;
  result.explored_cells = explored;
  result.bridge_trigger_explored_cells = bridge_trigger_explored_cells;
  result.path.reserve(path_cells->size());
  for (int cell : *path_cells) {
    const int x = cell % grid.width;
    const int y = cell / grid.width;
    if (gvd.mask[static_cast<size_t>(cell)]) {
      ++result.skeleton_cell_count;
    }
    result.path.push_back(gvd.world_point(grid, x, y));
  }

  return result;
}

std::optional<GvdComponentEntryResult> GvdMap::find_goal_component_entry(
  const GridMapView & grid,
  const GvdData & gvd,
  const std::pair<double, double> & robot_xy,
  const std::pair<double, double> & goal_xy,
  const GvdConfig & config)
{
  if (!grid.valid() || gvd.empty() || gvd.component_map.empty()) {
    return std::nullopt;
  }

  const auto goal_cell = nearest_gvd_cell(
    grid, gvd, goal_xy.first, goal_xy.second, config.snap_radius);
  if (!goal_cell) {
    return std::nullopt;
  }

  const int goal_idx = grid.index(goal_cell->first, goal_cell->second);
  const int target_component = gvd.component_map[static_cast<size_t>(goal_idx)];
  if (target_component < 0) {
    return std::nullopt;
  }

  const auto start_cell = nearest_free_cell(
    grid, robot_xy.first, robot_xy.second, config.snap_radius);
  if (!start_cell) {
    return std::nullopt;
  }

  const int start_idx = grid.index(start_cell->first, start_cell->second);
  int explored_cells = 0;
  const auto path_cells = find_path_to_component_entry(
    grid, gvd, start_idx, target_component, goal_idx, config.min_clearance, explored_cells);
  if (!path_cells || path_cells->empty()) {
    return std::nullopt;
  }

  const int entry_idx = path_cells->back();
  const int ex = entry_idx % grid.width;
  const int ey = entry_idx / grid.width;

  GvdComponentEntryResult result;
  result.point = gvd.world_point(grid, ex, ey);
  result.component_id = target_component;
  result.min_clearance_cells = min_clearance_on_path(gvd, *path_cells);
  result.explored_cells = explored_cells;
  return result;
}

std::optional<FreePathResult> GvdMap::find_safe_direct_goal(
  const GridMapView & grid,
  const GvdData & gvd,
  const std::pair<double, double> & robot_xy,
  const std::pair<double, double> & goal_xy,
  const GvdConfig & config,
  double goal_search_radius_m)
{
  if (!grid.valid() || gvd.empty()) {
    return std::nullopt;
  }

  const auto start_cell = nearest_free_cell(
    grid, robot_xy.first, robot_xy.second, goal_search_radius_m);
  const auto goal_cell = nearest_free_cell(
    grid, goal_xy.first, goal_xy.second, goal_search_radius_m);
  if (!start_cell || !goal_cell) {
    return std::nullopt;
  }

  const int start_idx = grid.index(start_cell->first, start_cell->second);
  const int goal_idx = grid.index(goal_cell->first, goal_cell->second);
  int explored_cells = 0;
  const auto path_cells = find_cell_path(
    grid, gvd, start_idx, goal_idx, false, config.min_clearance, explored_cells);
  if (!path_cells || path_cells->empty()) {
    return std::nullopt;
  }

  FreePathResult result;
  result.goal = grid.cell_center_world(goal_cell->first, goal_cell->second);
  result.min_clearance_cells = min_clearance_on_path(gvd, *path_cells);
  result.explored_cells = explored_cells;
  result.adjusted_goal = std::hypot(
    result.goal.first - goal_xy.first,
    result.goal.second - goal_xy.second) > 1e-3;
  return result;
}

}  // namespace auto_explore_sim
