#include "auto_explore_sim/frontier_search.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <queue>
#include <utility>
#include <vector>

namespace auto_explore_sim
{
namespace
{

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

bool is_frontier_cell(const GridMapView & grid, int x, int y)
{
  if (grid.value(x, y) != -1) {
    return false;
  }
  for (const auto & [dx, dy] : kNeighbors8) {
    const int nx = x + dx;
    const int ny = y + dy;
    if (grid.in_bounds(nx, ny) && grid.value(nx, ny) == 0) {
      return true;
    }
  }
  return false;
}

std::optional<std::pair<int, int>> nearest_free_cell(
  const GridMapView & grid, int sx, int sy, int max_radius)
{
  for (int r = 1; r < max_radius; ++r) {
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy : {-r, r}) {
        const int nx = sx + dx;
        const int ny = sy + dy;
        if (grid.in_bounds(nx, ny) && grid.value(nx, ny) == 0) {
          return std::make_pair(nx, ny);
        }
      }
    }
    for (int dy = -r + 1; dy < r; ++dy) {
      for (int dx : {-r, r}) {
        const int nx = sx + dx;
        const int ny = sy + dy;
        if (grid.in_bounds(nx, ny) && grid.value(nx, ny) == 0) {
          return std::make_pair(nx, ny);
        }
      }
    }
  }
  return std::nullopt;
}

Frontier build_frontier(
  const GridMapView & grid,
  int sx,
  int sy,
  std::vector<uint8_t> & state,
  const std::pair<double, double> & robot_xy,
  const std::vector<double> & dist_map,
  const FrontierSearchConfig & config)
{
  Frontier frontier;
  double sum_x = 0.0;
  double sum_y = 0.0;
  std::queue<std::pair<int, int>> queue;
  queue.emplace(sx, sy);
  state[static_cast<size_t>(grid.index(sx, sy))] = 3;

  while (!queue.empty()) {
    const auto [cx, cy] = queue.front();
    queue.pop();
    const int current_idx = grid.index(cx, cy);
    if (state[static_cast<size_t>(current_idx)] == 4) {
      continue;
    }
    state[static_cast<size_t>(current_idx)] = 4;

    const auto [wx, wy] = grid.cell_world(cx, cy);
    sum_x += wx;
    sum_y += wy;
    frontier.size += 1;
    frontier.min_distance = std::min(
      frontier.min_distance,
      std::hypot(wx - robot_xy.first, wy - robot_xy.second));

    for (const auto & [dx, dy] : kNeighbors8) {
      const int nx = cx + dx;
      const int ny = cy + dy;
      if (!grid.in_bounds(nx, ny)) {
        continue;
      }
      const int nidx = grid.index(nx, ny);
      if (state[static_cast<size_t>(nidx)] != 3 &&
        state[static_cast<size_t>(nidx)] != 4 &&
        is_frontier_cell(grid, nx, ny))
      {
        state[static_cast<size_t>(nidx)] = 3;
        queue.emplace(nx, ny);
      }
    }
  }

  if (frontier.size > 0) {
    frontier.centroid_x = sum_x / frontier.size;
    frontier.centroid_y = sum_y / frontier.size;
    frontier.heuristic_distance = std::hypot(
      frontier.centroid_x - robot_xy.first,
      frontier.centroid_y - robot_xy.second);
    // Keep cost aligned with the selected frontier ordering metric so the
    // caller can continue logging a single comparable scalar.
    frontier.cost = frontier.heuristic_distance;
  }

  return frontier;
}

bool is_within_update_radius(
  const Frontier & frontier, const FrontierSearchConfig & config)
{
  return config.frontier_update_radius <= 0.0 ||
         frontier.min_distance <= config.frontier_update_radius;
}

}  // namespace

std::vector<Frontier> FrontierSearch::search(
  const GridMapView & grid,
  const std::pair<double, double> & robot_xy,
  const std::vector<double> & dist_map,
  const FrontierSearchConfig & config)
{
  std::vector<Frontier> frontiers;
  if (!grid.valid() || dist_map.size() != grid.cells->size()) {
    return frontiers;
  }

  int mx = static_cast<int>((robot_xy.first - grid.origin_x) / grid.resolution);
  int my = static_cast<int>((robot_xy.second - grid.origin_y) / grid.resolution);
  if (!grid.in_bounds(mx, my)) {
    return frontiers;
  }

  if (grid.value(mx, my) != 0) {
    const auto free_cell = nearest_free_cell(grid, mx, my, 50);
    if (!free_cell) {
      return frontiers;
    }
    mx = free_cell->first;
    my = free_cell->second;
  }

  enum : uint8_t { UNVISITED = 0, MAP_OPEN = 1, MAP_CLOSED = 2, FRONTIER_OPEN = 3, FRONTIER_CLOSED = 4 };
  std::vector<uint8_t> state(static_cast<size_t>(grid.width * grid.height), UNVISITED);
  std::queue<std::pair<int, int>> bfs_queue;
  bfs_queue.emplace(mx, my);
  state[static_cast<size_t>(grid.index(mx, my))] = MAP_OPEN;

  while (!bfs_queue.empty()) {
    const auto [cx, cy] = bfs_queue.front();
    bfs_queue.pop();
    const int current_idx = grid.index(cx, cy);
    if (state[static_cast<size_t>(current_idx)] == MAP_CLOSED) {
      continue;
    }
    state[static_cast<size_t>(current_idx)] = MAP_CLOSED;

    for (const auto & [dx, dy] : kNeighbors8) {
      const int nx = cx + dx;
      const int ny = cy + dy;
      if (!grid.in_bounds(nx, ny)) {
        continue;
      }

      const int nidx = grid.index(nx, ny);
      if (state[static_cast<size_t>(nidx)] != FRONTIER_OPEN &&
        state[static_cast<size_t>(nidx)] != FRONTIER_CLOSED &&
        is_frontier_cell(grid, nx, ny))
      {
        auto frontier = build_frontier(grid, nx, ny, state, robot_xy, dist_map, config);
        if (frontier.size >= config.min_frontier_size &&
          is_within_update_radius(frontier, config))
        {
          frontiers.push_back(frontier);
        }
      }

      if (grid.value(nx, ny) == 0 &&
        state[static_cast<size_t>(nidx)] != MAP_OPEN &&
        state[static_cast<size_t>(nidx)] != MAP_CLOSED)
      {
        state[static_cast<size_t>(nidx)] = MAP_OPEN;
        bfs_queue.emplace(nx, ny);
      }
    }
  }

  std::sort(
    frontiers.begin(), frontiers.end(),
    [](const Frontier & a, const Frontier & b) {
      if (a.heuristic_distance != b.heuristic_distance) {
        return a.heuristic_distance < b.heuristic_distance;
      }
      if (a.min_distance != b.min_distance) {
        return a.min_distance < b.min_distance;
      }
      return a.size > b.size;
    });
  return frontiers;
}

}  // namespace auto_explore_sim
