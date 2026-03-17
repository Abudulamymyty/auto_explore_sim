#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace
{
struct Frontier
{
  int size{0};
  double min_distance{std::numeric_limits<double>::infinity()};
  double cost{0.0};
  double centroid_x{0.0};
  double centroid_y{0.0};
};

struct BlacklistedPoint
{
  double x{0.0};
  double y{0.0};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

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

constexpr double kVoronoiDistanceToleranceCells = 0.75;
constexpr double kVoronoiMaxDotProduct = 0.5;
constexpr double kOffSkeletonStepPenalty = 3.0;
constexpr double kLowClearancePenaltyScale = 0.5;

inline bool same_stamp(
  const builtin_interfaces::msg::Time & a,
  const builtin_interfaces::msg::Time & b)
{
  return a.sec == b.sec && a.nanosec == b.nanosec;
}

inline geometry_msgs::msg::Point make_point(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}
}  // namespace

class FrontierExplorerNode : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavTo = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using GoalHandleNavThrough = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  FrontierExplorerNode()
  : rclcpp::Node("frontier_explorer")
  {
    declare_parameter("planner_frequency", 0.5);
    declare_parameter("min_frontier_size", 5);
    declare_parameter("robot_base_frame", "base");
    declare_parameter("odom_frame", "odom");
    declare_parameter("odom_topic", "/lightning/odometry");
    declare_parameter("global_frame", "map");
    declare_parameter("map_topic", "/map");
    declare_parameter("live_map_ready_topic", "/map_live_ready");
    declare_parameter("require_live_map", true);
    declare_parameter("transform_tolerance", 2.0);
    declare_parameter("blacklist_radius", 0.5);
    declare_parameter("blacklist_timeout", 60.0);
    declare_parameter("progress_timeout", 30.0);
    declare_parameter("visualize", true);
    declare_parameter("clearance_scale", 0.3);
    declare_parameter("return_to_init", false);
    declare_parameter("gvd_min_clearance", 3);
    declare_parameter("gvd_snap_radius", 2.0);
    declare_parameter("visualize_gvd", true);
    declare_parameter("gvd_marker_publish_every", 1);
    declare_parameter("gvd_marker_stride", 1);
    declare_parameter("profile_timing", false);
    declare_parameter("profile_log_interval", 15.0);
    declare_parameter("profile_callbacks", false);
    declare_parameter("profile_callback_log_interval", 10.0);

    planner_freq_ = get_parameter("planner_frequency").as_double();
    min_frontier_size_ = get_parameter("min_frontier_size").as_int();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    map_topic_ = get_parameter("map_topic").as_string();
    live_map_ready_topic_ = get_parameter("live_map_ready_topic").as_string();
    require_live_map_ = get_parameter("require_live_map").as_bool();
    tf_tolerance_ = get_parameter("transform_tolerance").as_double();
    blacklist_radius_ = get_parameter("blacklist_radius").as_double();
    blacklist_timeout_ = get_parameter("blacklist_timeout").as_double();
    progress_timeout_ = get_parameter("progress_timeout").as_double();
    visualize_ = get_parameter("visualize").as_bool();
    clearance_scale_ = get_parameter("clearance_scale").as_double();
    return_to_init_ = get_parameter("return_to_init").as_bool();
    gvd_min_clearance_ = get_parameter("gvd_min_clearance").as_int();
    gvd_snap_radius_ = get_parameter("gvd_snap_radius").as_double();
    visualize_gvd_ = get_parameter("visualize_gvd").as_bool();
    gvd_marker_publish_every_ =
      std::max(1, static_cast<int>(get_parameter("gvd_marker_publish_every").as_int()));
    gvd_marker_stride_ =
      std::max(1, static_cast<int>(get_parameter("gvd_marker_stride").as_int()));
    profile_timing_ = get_parameter("profile_timing").as_bool();
    profile_log_interval_ = get_parameter("profile_log_interval").as_double();
    profile_callbacks_ = get_parameter("profile_callbacks").as_bool();
    profile_callback_log_interval_ =
      get_parameter("profile_callback_log_interval").as_double();

    rclcpp::QoS tf_qos(rclcpp::KeepLast(100));
    tf_qos.best_effort();
    tf_qos.durability_volatile();
    rclcpp::QoS tf_static_qos(rclcpp::KeepLast(100));
    tf_static_qos.reliable();
    tf_static_qos.transient_local();
    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, map_qos,
      std::bind(&FrontierExplorerNode::map_callback, this, std::placeholders::_1));
    live_map_ready_sub_ = create_subscription<std_msgs::msg::Bool>(
      live_map_ready_topic_, map_qos,
      std::bind(&FrontierExplorerNode::live_map_ready_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&FrontierExplorerNode::odom_callback, this, std::placeholders::_1));
    tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", tf_qos,
      std::bind(&FrontierExplorerNode::tf_callback, this, std::placeholders::_1));
    tf_static_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf_static", tf_static_qos,
      std::bind(&FrontierExplorerNode::tf_callback, this, std::placeholders::_1));
    resume_sub_ = create_subscription<std_msgs::msg::Bool>(
      "explore/resume", 10,
      std::bind(&FrontierExplorerNode::resume_callback, this, std::placeholders::_1));

    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    nav_through_client_ =
      rclcpp_action::create_client<NavigateThroughPoses>(this, "navigate_through_poses");

    if (visualize_) {
      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("explore/frontiers", 10);
    }
    if (visualize_gvd_) {
      gvd_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("explore/gvd", 10);
    }
    path_marker_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("explore/gvd_path", 10);

    timing_last_log_ = std::chrono::steady_clock::now();
    callback_last_log_ = timing_last_log_;

    const double period_s = 1.0 / std::max(planner_freq_, 0.01);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period_s),
      std::bind(&FrontierExplorerNode::explore_tick, this));

    RCLCPP_INFO(
      get_logger(),
      "Frontier Explorer C++ ready (freq=%.2f Hz, min_frontier=%d cells, clearance_scale=%.2f, "
      "map_topic=%s, gvd_snap_radius=%.2fm, gvd_marker_every=%d, gvd_marker_stride=%d)",
      planner_freq_, min_frontier_size_, clearance_scale_, map_topic_.c_str(), gvd_snap_radius_,
      gvd_marker_publish_every_, gvd_marker_stride_);
  }

private:
  using Clock = std::chrono::steady_clock;

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    auto started = Clock::now();
    map_msg_ = msg;
    if (!live_map_ready_) {
      live_map_ready_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Received live map data directly from the map topic; marking live map as ready");
    }
    width_ = static_cast<int>(msg->info.width);
    height_ = static_cast<int>(msg->info.height);
    resolution_ = msg->info.resolution;
    origin_x_ = msg->info.origin.position.x;
    origin_y_ = msg->info.origin.position.y;
    map_array_ = msg->data;

    clear_map_caches();
    record_callback_timing("map_callback", started);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    auto started = Clock::now();
    const auto frame_id = normalize_frame_id(msg->header.frame_id);
    if (!frame_id.empty() && frame_id != odom_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring odom pose in unexpected frame '%s'; expected '%s'",
        frame_id.c_str(), odom_frame_.c_str());
      record_callback_timing("odom_callback", started);
      return;
    }

    const auto prev_odom_pose = odom_pose_;
    odom_pose_ = std::make_pair(msg->pose.pose.position.x, msg->pose.pose.position.y);
    odom_stamp_ = msg->header.stamp;

    if (!tf_sub_ && !tf_static_sub_ && prev_odom_pose && robot_pose_ && map_to_odom_yaw_) {
      const double dx = odom_pose_->first - prev_odom_pose->first;
      const double dy = odom_pose_->second - prev_odom_pose->second;
      const double cos_y = std::cos(*map_to_odom_yaw_);
      const double sin_y = std::sin(*map_to_odom_yaw_);
      robot_pose_ = std::make_pair(
        robot_pose_->first + cos_y * dx - sin_y * dy,
        robot_pose_->second + sin_y * dx + cos_y * dy);
      robot_pose_stamp_ = odom_stamp_;
      record_callback_timing("odom_callback", started);
      return;
    }

    update_robot_pose_cache();
    record_callback_timing("odom_callback", started);
  }

  void tf_callback(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    auto started = Clock::now();
    bool updated = false;
    for (const auto & transform : msg->transforms) {
      const auto parent = normalize_frame_id(transform.header.frame_id);
      const auto child = normalize_frame_id(transform.child_frame_id);
      if (parent == global_frame_ && child == odom_frame_) {
        store_map_to_odom(
          transform.transform.translation.x,
          transform.transform.translation.y,
          yaw_from_quaternion(transform.transform.rotation),
          transform.header.stamp);
        updated = true;
      } else if (parent == odom_frame_ && child == global_frame_) {
        const double tx = transform.transform.translation.x;
        const double ty = transform.transform.translation.y;
        const double yaw = yaw_from_quaternion(transform.transform.rotation);
        const double cos_y = std::cos(yaw);
        const double sin_y = std::sin(yaw);
        store_map_to_odom(
          -(cos_y * tx + sin_y * ty),
          sin_y * tx - cos_y * ty,
          -yaw,
          transform.header.stamp);
        updated = true;
      }
    }
    if (updated) {
      update_robot_pose_cache();
    }
    record_callback_timing("tf_callback", started);
  }

  void resume_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    auto started = Clock::now();
    if (msg->data) {
      RCLCPP_INFO(get_logger(), "Exploration RESUMED");
      exploring_ = true;
    } else {
      RCLCPP_INFO(get_logger(), "Exploration STOPPED");
      exploring_ = false;
      cancel_current_goal();
      navigating_ = false;
    }
    record_callback_timing("resume_callback", started);
  }

  void live_map_ready_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    live_map_ready_ = msg->data;
  }

  void nav_feedback_cb(const geometry_msgs::msg::PoseStamped & current_pose)
  {
    auto started = Clock::now();
    const auto frame_id = normalize_frame_id(current_pose.header.frame_id);
    if (!frame_id.empty() && frame_id != global_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring Nav2 feedback pose in unexpected frame '%s'; expected '%s'",
        frame_id.c_str(), global_frame_.c_str());
      record_callback_timing("nav_feedback_cb", started);
      return;
    }

    robot_pose_ = std::make_pair(current_pose.pose.position.x, current_pose.pose.position.y);
    robot_pose_stamp_ = current_pose.header.stamp;

    if (!using_nav_feedback_pose_) {
      using_nav_feedback_pose_ = true;
      if (odom_sub_) {
        RCLCPP_INFO(get_logger(), "Switching from /odom tracking to Nav2 feedback pose tracking");
        odom_sub_.reset();
      }
    }
    record_callback_timing("nav_feedback_cb", started);
  }

  void explore_tick()
  {
    auto started = Clock::now();
    if (exploring_) {
      make_plan();
    }
    record_callback_timing("explore_tick", started);
  }

  void make_plan()
  {
    if (!map_msg_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for map...");
      return;
    }
    if (require_live_map_ && !live_map_ready_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for live map update from the current SLAM session...");
      return;
    }

    auto robot_xy = get_robot_position();
    if (!robot_xy) {
      return;
    }

    if (!initial_pose_) {
      initial_pose_ = robot_xy;
      RCLCPP_INFO(
        get_logger(), "Initial pose stored: (%.2f, %.2f)",
        initial_pose_->first, initial_pose_->second);
    }

    check_progress(*robot_xy);
    if (navigating_) {
      return;
    }

    const auto plan_started = Clock::now();
    auto stage_started = Clock::now();
    auto dist_map = get_distance_transform();
    record_timing("gvd_build", stage_started);
    auto search_started = Clock::now();
    auto frontiers = search_from(*robot_xy, dist_map);
    record_timing("frontier_search", search_started);

    if (frontiers.empty()) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "No frontiers found - exploration may be complete!");
      if (return_to_init_ && initial_pose_) {
        return_to_initial_pose();
      }
      return;
    }

    prune_blacklist();
    std::vector<Frontier> valid;
    valid.reserve(frontiers.size());
    for (const auto & frontier : frontiers) {
      if (!is_blacklisted(frontier.centroid_x, frontier.centroid_y)) {
        valid.push_back(frontier);
      }
    }
    if (valid.empty()) {
      RCLCPP_WARN(get_logger(), "All frontiers blacklisted - clearing blacklist");
      blacklisted_.clear();
      valid = frontiers;
    }

    const Frontier & best = valid.front();
    RCLCPP_INFO(
      get_logger(), "Best frontier: (%.2f, %.2f) min_dist=%.2fm size=%d cost=%.1f",
      best.centroid_x, best.centroid_y, best.min_distance, best.size, best.cost);

    if (visualize_) {
      auto marker_started = Clock::now();
      publish_markers(valid, best);
      record_timing("frontier_markers", marker_started);
    }
    if (visualize_gvd_) {
      auto marker_started = Clock::now();
      publish_gvd_markers();
      record_timing("gvd_markers", marker_started);
    }

    auto gvd_started = Clock::now();
    auto gvd_path = find_gvd_path(*robot_xy, {best.centroid_x, best.centroid_y});
    record_timing("gvd_path_search", gvd_started);

    std::optional<std::vector<std::pair<double, double>>> waypoints;
    double gx = best.centroid_x;
    double gy = best.centroid_y;
    if (gvd_path && gvd_path->size() >= 2U) {
      auto waypoint_started = Clock::now();
      waypoints = sample_waypoints(*gvd_path, 0.5);
      record_timing("waypoint_sampling", waypoint_started);
      gx = waypoints->back().first;
      gy = waypoints->back().second;
      RCLCPP_INFO(
        get_logger(), "GVD path found: %zu cells -> %zu waypoints",
        gvd_path->size(), waypoints->size());
    } else {
      auto snap_started = Clock::now();
      const auto snapped = snap_to_gvd(
        {best.centroid_x, best.centroid_y}, *robot_xy);
      record_timing("gvd_snap_fallback", snap_started);
      gx = snapped.first;
      gy = snapped.second;
      RCLCPP_INFO(get_logger(), "No GVD path - using single goal fallback");
    }

    const double dist_to_goal = std::hypot(gx - robot_xy->first, gy - robot_xy->second);
    if (dist_to_goal < 0.3) {
      RCLCPP_WARN(
        get_logger(), "Goal (%.2f, %.2f) only %.2fm away - blacklisting frontier",
        gx, gy, dist_to_goal);
      blacklist_point(best.centroid_x, best.centroid_y);
      return;
    }

    if (prev_goal_) {
      const double dx = gx - prev_goal_->first;
      const double dy = gy - prev_goal_->second;
      if (std::sqrt(dx * dx + dy * dy) < 0.01) {
        RCLCPP_DEBUG(get_logger(), "Same goal as before - skipping");
        return;
      }
    }

    current_frontier_ = std::make_pair(best.centroid_x, best.centroid_y);
    auto send_started = Clock::now();
    if (waypoints && waypoints->size() >= 2U) {
      auto path_marker_started = Clock::now();
      publish_path_markers(*waypoints);
      record_timing("path_markers", path_marker_started);
      navigate_through_poses(*waypoints);
    } else {
      navigate_to(gx, gy);
    }
    record_timing("send_goal", send_started);
    record_timing("plan_total", plan_started);
    ++timing_plan_counter_;
    maybe_log_timing();
  }

  std::vector<Frontier> search_from(
    const std::pair<double, double> & robot_xy,
    const std::vector<double> & dist_map)
  {
    std::vector<Frontier> frontiers;
    if (width_ <= 0 || height_ <= 0 || map_array_.empty()) {
      return frontiers;
    }

    int mx = static_cast<int>((robot_xy.first - origin_x_) / resolution_);
    int my = static_cast<int>((robot_xy.second - origin_y_) / resolution_);
    if (!in_bounds(mx, my)) {
      RCLCPP_WARN(get_logger(), "Robot position outside map bounds");
      return frontiers;
    }

    if (map_value(mx, my) != 0) {
      const auto free_cell = nearest_free_cell(mx, my, 50);
      if (!free_cell) {
        RCLCPP_WARN(get_logger(), "Cannot find free cell near robot");
        return frontiers;
      }
      mx = free_cell->first;
      my = free_cell->second;
    }

    enum : uint8_t { UNVISITED = 0, MAP_OPEN = 1, MAP_CLOSED = 2, FRONTIER_OPEN = 3, FRONTIER_CLOSED = 4 };
    std::vector<uint8_t> state(static_cast<size_t>(width_ * height_), UNVISITED);
    std::queue<std::pair<int, int>> bfs_queue;
    bfs_queue.emplace(mx, my);
    state[index(mx, my)] = MAP_OPEN;

    while (!bfs_queue.empty()) {
      const auto [cx, cy] = bfs_queue.front();
      bfs_queue.pop();
      const auto current_idx = index(cx, cy);
      if (state[current_idx] == MAP_CLOSED) {
        continue;
      }
      state[current_idx] = MAP_CLOSED;

      for (const auto & [dx, dy] : neighbors8_) {
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (!in_bounds(nx, ny)) {
          continue;
        }

        const auto nidx = index(nx, ny);
        if (state[nidx] != FRONTIER_OPEN && state[nidx] != FRONTIER_CLOSED &&
          is_frontier_cell(nx, ny))
        {
          auto frontier = build_frontier(nx, ny, state, robot_xy, dist_map);
          if (frontier.size >= min_frontier_size_) {
            frontiers.push_back(frontier);
          }
        }

        if (map_value(nx, ny) == 0 && state[nidx] != MAP_OPEN && state[nidx] != MAP_CLOSED) {
          state[nidx] = MAP_OPEN;
          bfs_queue.emplace(nx, ny);
        }
      }
    }

    std::sort(
      frontiers.begin(), frontiers.end(),
      [](const Frontier & a, const Frontier & b) {return a.cost < b.cost;});
    return frontiers;
  }

  Frontier build_frontier(
    int sx, int sy, std::vector<uint8_t> & state,
    const std::pair<double, double> & robot_xy,
    const std::vector<double> & dist_map)
  {
    Frontier frontier;
    double sum_x = 0.0;
    double sum_y = 0.0;
    std::queue<std::pair<int, int>> queue;
    queue.emplace(sx, sy);
    state[index(sx, sy)] = 3;

    while (!queue.empty()) {
      const auto [cx, cy] = queue.front();
      queue.pop();
      const auto current_idx = index(cx, cy);
      if (state[current_idx] == 4) {
        continue;
      }
      state[current_idx] = 4;

      const double wx = cx * resolution_ + origin_x_;
      const double wy = cy * resolution_ + origin_y_;
      sum_x += wx;
      sum_y += wy;
      frontier.size += 1;
      frontier.min_distance = std::min(frontier.min_distance, std::hypot(wx - robot_xy.first, wy - robot_xy.second));

      for (const auto & [dx, dy] : neighbors8_) {
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (!in_bounds(nx, ny)) {
          continue;
        }
        const auto nidx = index(nx, ny);
        if (state[nidx] != 3 && state[nidx] != 4 && is_frontier_cell(nx, ny)) {
          state[nidx] = 3;
          queue.emplace(nx, ny);
        }
      }
    }

    if (frontier.size > 0) {
      frontier.centroid_x = sum_x / frontier.size;
      frontier.centroid_y = sum_y / frontier.size;
      int cx = static_cast<int>((frontier.centroid_x - origin_x_) / resolution_);
      int cy = static_cast<int>((frontier.centroid_y - origin_y_) / resolution_);
      cx = std::clamp(cx, 0, width_ - 1);
      cy = std::clamp(cy, 0, height_ - 1);
      const double clearance = dist_map[index(cx, cy)] * resolution_;
      frontier.cost = frontier.min_distance - clearance_scale_ * clearance;
    }
    return frontier;
  }

  bool is_frontier_cell(int x, int y) const
  {
    if (map_value(x, y) != -1) {
      return false;
    }
    for (const auto & [dx, dy] : neighbors8_) {
      const int nx = x + dx;
      const int ny = y + dy;
      if (in_bounds(nx, ny) && map_value(nx, ny) == 0) {
        return true;
      }
    }
    return false;
  }

  std::optional<std::pair<int, int>> nearest_free_cell(int sx, int sy, int max_radius) const
  {
    for (int r = 1; r < max_radius; ++r) {
      for (int dx = -r; dx <= r; ++dx) {
        for (int dy : {-r, r}) {
          const int nx = sx + dx;
          const int ny = sy + dy;
          if (in_bounds(nx, ny) && map_value(nx, ny) == 0) {
            return std::make_pair(nx, ny);
          }
        }
      }
      for (int dy = -r + 1; dy < r; ++dy) {
        for (int dx : {-r, r}) {
          const int nx = sx + dx;
          const int ny = sy + dy;
          if (in_bounds(nx, ny) && map_value(nx, ny) == 0) {
            return std::make_pair(nx, ny);
          }
        }
      }
    }
    return std::nullopt;
  }

  bool is_obstacle_boundary_cell(int x, int y) const
  {
    const int idx0 = index(x, y);
    if (!is_obstacle(idx0)) {
      return false;
    }
    for (const auto & [dx, dy] : neighbors8_) {
      const int nx = x + dx;
      const int ny = y + dy;
      if (!in_bounds(nx, ny)) {
        return true;
      }
      if (!is_obstacle(index(nx, ny))) {
        return true;
      }
    }
    return false;
  }

  std::pair<double, double> cell_center_world(int x, int y) const
  {
    return {
      (static_cast<double>(x) + 0.5) * resolution_ + origin_x_,
      (static_cast<double>(y) + 0.5) * resolution_ + origin_y_};
  }

  double distance_to_site_cells(int x, int y, int site_id) const
  {
    if (site_id < 0 || static_cast<size_t>(site_id) >= site_xs_.size()) {
      return std::numeric_limits<double>::infinity();
    }
    return std::hypot(
      static_cast<double>(x - site_xs_[static_cast<size_t>(site_id)]),
      static_cast<double>(y - site_ys_[static_cast<size_t>(site_id)]));
  }

  std::pair<double, double> project_to_voronoi_bisector(int x, int y, int site_a, int site_b) const
  {
    const auto point = cell_center_world(x, y);
    if (site_a < 0 || site_b < 0 || static_cast<size_t>(site_a) >= site_xs_.size() ||
      static_cast<size_t>(site_b) >= site_xs_.size() || site_a == site_b)
    {
      return point;
    }

    const auto site_a_world =
      cell_center_world(site_xs_[static_cast<size_t>(site_a)], site_ys_[static_cast<size_t>(site_a)]);
    const auto site_b_world =
      cell_center_world(site_xs_[static_cast<size_t>(site_b)], site_ys_[static_cast<size_t>(site_b)]);
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

  std::pair<double, double> gvd_world_point_for_cell(int x, int y) const
  {
    const int idx0 = index(x, y);
    if (idx0 >= 0 && static_cast<size_t>(idx0) < gvd_world_x_.size()) {
      return {
        gvd_world_x_[static_cast<size_t>(idx0)],
        gvd_world_y_[static_cast<size_t>(idx0)]};
    }
    return cell_center_world(x, y);
  }

  const std::vector<double> & get_distance_transform()
  {
    if (!map_msg_) {
      return dist_map_;
    }
    if (!dist_map_.empty() && same_stamp(cached_map_stamp_, map_msg_->header.stamp)) {
      return dist_map_;
    }

    compute_distance_and_labels();
    extract_gvd_mask();
    cached_map_stamp_ = map_msg_->header.stamp;
    cached_gvd_marker_valid_ = false;
    return dist_map_;
  }

  void compute_distance_and_labels()
  {
    const int total = width_ * height_;
    dist_map_.assign(static_cast<size_t>(total), std::numeric_limits<double>::infinity());
    label_map_.assign(static_cast<size_t>(total), -1);
    site_xs_.clear();
    site_ys_.clear();

    std::priority_queue<WavefrontEntry> open_set;
    int counter = 0;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const int idx = index(x, y);
        if (!is_obstacle(idx) || !is_obstacle_boundary_cell(x, y)) {
          continue;
        }
        const int site_id = static_cast<int>(site_xs_.size());
        site_xs_.push_back(x);
        site_ys_.push_back(y);
        dist_map_[static_cast<size_t>(idx)] = 0.0;
        label_map_[static_cast<size_t>(idx)] = site_id;
        open_set.push({0.0, counter++, idx, site_id});
      }
    }

    while (!open_set.empty()) {
      const auto current = open_set.top();
      open_set.pop();
      const int idx0 = current.index;
      if (current.cost > dist_map_[static_cast<size_t>(idx0)] + 1e-9 ||
        label_map_[static_cast<size_t>(idx0)] != current.site_id)
      {
        continue;
      }

      const int cx = idx0 % width_;
      const int cy = idx0 / width_;
      for (const auto & [dx, dy] : neighbors8_) {
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (!in_bounds(nx, ny)) {
          continue;
        }
        const int nidx = index(nx, ny);
        if (is_obstacle(nidx)) {
          continue;
        }

        const double candidate = distance_to_site_cells(nx, ny, current.site_id);
        double & current_dist = dist_map_[static_cast<size_t>(nidx)];
        int & current_label = label_map_[static_cast<size_t>(nidx)];
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

    for (auto & value : dist_map_) {
      if (!std::isfinite(value)) {
        value = 0.0;
      }
    }
  }

  void extract_gvd_mask()
  {
    const int total = width_ * height_;
    gvd_mask_.assign(static_cast<size_t>(total), 0);
    gvd_world_x_.assign(static_cast<size_t>(total), 0.0);
    gvd_world_y_.assign(static_cast<size_t>(total), 0.0);
    int gvd_count = 0;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        const int idx0 = index(x, y);
        const auto center = cell_center_world(x, y);
        gvd_world_x_[static_cast<size_t>(idx0)] = center.first;
        gvd_world_y_[static_cast<size_t>(idx0)] = center.second;

        if (map_array_[idx0] != 0 ||
          dist_map_[static_cast<size_t>(idx0)] < static_cast<double>(gvd_min_clearance_))
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

        add_candidate(label_map_[static_cast<size_t>(idx0)]);
        for (const auto & [dx, dy] : neighbors8_) {
          const int nx = x + dx;
          const int ny = y + dy;
          if (!in_bounds(nx, ny)) {
            continue;
          }
          add_candidate(label_map_[static_cast<size_t>(index(nx, ny))]);
        }

        if (candidate_count < 2) {
          continue;
        }

        double best_pair_diff = std::numeric_limits<double>::infinity();
        int best_site = -1;
        int second_site = -1;
        for (int i = 0; i < candidate_count; ++i) {
          const int site_a = candidate_sites[static_cast<size_t>(i)];
          const double dist_a = distance_to_site_cells(x, y, site_a);
          for (int j = i + 1; j < candidate_count; ++j) {
            const int site_b = candidate_sites[static_cast<size_t>(j)];
            const double dist_b = distance_to_site_cells(x, y, site_b);
            const double diff = std::abs(dist_a - dist_b);
            if (diff > kVoronoiDistanceToleranceCells || diff >= best_pair_diff) {
              continue;
            }

            const double ax = static_cast<double>(site_xs_[static_cast<size_t>(site_a)] - x);
            const double ay = static_cast<double>(site_ys_[static_cast<size_t>(site_a)] - y);
            const double bx = static_cast<double>(site_xs_[static_cast<size_t>(site_b)] - x);
            const double by = static_cast<double>(site_ys_[static_cast<size_t>(site_b)] - y);
            const double a_norm = std::hypot(ax, ay);
            const double b_norm = std::hypot(bx, by);
            if (a_norm <= 1e-9 || b_norm <= 1e-9) {
              continue;
            }
            const double dot = (ax * bx + ay * by) / (a_norm * b_norm);
            if (dot > kVoronoiMaxDotProduct) {
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

        gvd_mask_[static_cast<size_t>(idx0)] = 1;
        const auto projected = project_to_voronoi_bisector(x, y, best_site, second_site);
        gvd_world_x_[static_cast<size_t>(idx0)] = projected.first;
        gvd_world_y_[static_cast<size_t>(idx0)] = projected.second;
        ++gvd_count;
      }
    }

    RCLCPP_INFO(
      get_logger(), "GVD rebuilt: %zu boundary sites, %d Voronoi cells",
      site_xs_.size(), gvd_count);
  }

  std::pair<double, double> snap_to_gvd(
    const std::pair<double, double> & point,
    const std::pair<double, double> & robot_xy)
  {
    get_distance_transform();
    if (gvd_mask_.empty()) {
      return point;
    }

    int px = static_cast<int>((point.first - origin_x_) / resolution_);
    int py = static_cast<int>((point.second - origin_y_) / resolution_);
    px = std::clamp(px, 0, width_ - 1);
    py = std::clamp(py, 0, height_ - 1);
    if (gvd_mask_[index(px, py)]) {
      return gvd_world_point_for_cell(px, py);
    }

    const double robot_to_frontier = std::hypot(point.first - robot_xy.first, point.second - robot_xy.second);
    const int max_cells = static_cast<int>(gvd_snap_radius_ / resolution_);
    bool found = false;
    double best_dist = std::numeric_limits<double>::infinity();
    std::pair<double, double> best_point = point;
    for (int y = std::max(0, py - max_cells); y <= std::min(height_ - 1, py + max_cells); ++y) {
      for (int x = std::max(0, px - max_cells); x <= std::min(width_ - 1, px + max_cells); ++x) {
        if (!gvd_mask_[index(x, y)]) {
          continue;
        }
        const auto [wx, wy] = gvd_world_point_for_cell(x, y);
        const double cand_to_robot = std::hypot(wx - robot_xy.first, wy - robot_xy.second);
        if (cand_to_robot >= robot_to_frontier) {
          continue;
        }
        const double cand_to_frontier = std::hypot(wx - point.first, wy - point.second);
        if (cand_to_frontier < best_dist) {
          best_dist = cand_to_frontier;
          best_point = {wx, wy};
          found = true;
        }
      }
    }
    if (found) {
      RCLCPP_INFO(
        get_logger(), "Frontier snapped to GVD: (%.2f, %.2f) -> (%.2f, %.2f)",
        point.first, point.second, best_point.first, best_point.second);
    }
    return best_point;
  }

  std::optional<std::pair<int, int>> nearest_gvd_cell(double wx, double wy, double max_radius_m = 1.5)
  {
    get_distance_transform();
    if (gvd_mask_.empty()) {
      return std::nullopt;
    }
    int px = std::clamp(static_cast<int>((wx - origin_x_) / resolution_), 0, width_ - 1);
    int py = std::clamp(static_cast<int>((wy - origin_y_) / resolution_), 0, height_ - 1);
    if (gvd_mask_[index(px, py)]) {
      return std::make_pair(px, py);
    }

    const int max_cells = static_cast<int>(max_radius_m / resolution_);
    bool found = false;
    int best_x = px;
    int best_y = py;
    int best_d2 = std::numeric_limits<int>::max();
    for (int y = std::max(0, py - max_cells); y <= std::min(height_ - 1, py + max_cells); ++y) {
      for (int x = std::max(0, px - max_cells); x <= std::min(width_ - 1, px + max_cells); ++x) {
        if (!gvd_mask_[index(x, y)]) {
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

  std::optional<std::vector<int>> find_cell_path(
    int start_idx, int end_idx, bool skeleton_only, int & explored_cells) const
  {
    const int total = width_ * height_;
    std::vector<double> g_score(static_cast<size_t>(total), std::numeric_limits<double>::infinity());
    std::vector<int> came_from(static_cast<size_t>(total), -1);
    std::priority_queue<IndexedCost> open_set;

    g_score[static_cast<size_t>(start_idx)] = 0.0;
    int counter = 0;
    open_set.push({0.0, counter, start_idx});

    const int ex = end_idx % width_;
    const int ey = end_idx / width_;

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

      const int cx = idx0 % width_;
      const int cy = idx0 / width_;
      for (const auto & [dx, dy] : neighbors8_) {
        const int nx = cx + dx;
        const int ny = cy + dy;
        if (!in_bounds(nx, ny)) {
          continue;
        }

        const int nidx = index(nx, ny);
        if (map_array_[static_cast<size_t>(nidx)] != 0) {
          continue;
        }
        if (skeleton_only && !gvd_mask_[static_cast<size_t>(nidx)]) {
          continue;
        }

        double step_cost = (dx != 0 && dy != 0) ? std::sqrt(2.0) : 1.0;
        if (!gvd_mask_[static_cast<size_t>(nidx)]) {
          step_cost += kOffSkeletonStepPenalty;
        }
        const double clearance_deficit = std::max(
          0.0, static_cast<double>(gvd_min_clearance_) - dist_map_[static_cast<size_t>(nidx)]);
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

  std::optional<std::vector<std::pair<double, double>>> find_gvd_path(
    const std::pair<double, double> & robot_xy,
    const std::pair<double, double> & goal_xy)
  {
    get_distance_transform();
    if (gvd_mask_.empty()) {
      return std::nullopt;
    }
    const auto start = nearest_gvd_cell(robot_xy.first, robot_xy.second);
    const auto end = nearest_gvd_cell(goal_xy.first, goal_xy.second);
    if (!start || !end) {
      RCLCPP_INFO(
        get_logger(), "GVD path: no GVD cell near %s",
        !start ? "robot" : "goal");
      return std::nullopt;
    }
    if (*start == *end) {
      return std::nullopt;
    }

    const int start_idx = index(start->first, start->second);
    const int end_idx = index(end->first, end->second);
    int explored = 0;
    auto path_cells = find_cell_path(start_idx, end_idx, true, explored);
    if (!path_cells) {
      RCLCPP_INFO(
        get_logger(),
        "GVD path: skeleton disconnected after exploring %d cells, retrying with free-space bridge",
        explored);
      explored = 0;
      path_cells = find_cell_path(start_idx, end_idx, false, explored);
    }

    if (!path_cells) {
      RCLCPP_INFO(get_logger(), "GVD path: A* failed (explored %d cells)", explored);
      return std::nullopt;
    }

    std::vector<std::pair<double, double>> path;
    path.reserve(path_cells->size());
    int skeleton_count = 0;
    for (int cell : *path_cells) {
      const int x = cell % width_;
      const int y = cell / width_;
      if (gvd_mask_[static_cast<size_t>(cell)]) {
        ++skeleton_count;
      }
      path.push_back(gvd_world_point_for_cell(x, y));
    }

    RCLCPP_INFO(
      get_logger(), "GVD path: %zu cells, %d on skeleton (%d%%)",
      path.size(), skeleton_count,
      static_cast<int>(100 * skeleton_count / std::max<size_t>(1, path.size())));
    return path;
  }

  std::vector<std::pair<double, double>> sample_waypoints(
    const std::vector<std::pair<double, double>> & path,
    double spacing) const
  {
    if (path.size() <= 2U) {
      return path;
    }
    std::vector<std::pair<double, double>> sampled;
    sampled.push_back(path.front());
    double accum = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
      accum += std::hypot(path[i].first - path[i - 1].first, path[i].second - path[i - 1].second);
      if (accum >= spacing) {
        sampled.push_back(path[i]);
        accum = 0.0;
      }
    }
    if (sampled.back() != path.back()) {
      sampled.push_back(path.back());
    }
    return sampled;
  }

  void navigate_to(double x, double y)
  {
    if (!nav_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "Nav2 action server not available!");
      return;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = global_frame_;
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.w = 1.0;

    ++goal_seq_;
    const int seq = goal_seq_;
    RCLCPP_INFO(get_logger(), "Sending goal: (%.2f, %.2f)", x, y);

    typename rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;
    options.goal_response_callback = [this, seq](GoalHandleNavTo::SharedPtr handle) {
        goal_response_cb(handle != nullptr, seq, [this, handle]() {
            nav_to_goal_handle_ = handle;
            nav_through_goal_handle_.reset();
          });
      };
    options.feedback_callback = [this](GoalHandleNavTo::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        nav_feedback_cb(feedback->current_pose);
      };
    options.result_callback = [this, seq](const GoalHandleNavTo::WrappedResult & result) {
        navigation_result_cb(result.code, seq);
      };
    nav_client_->async_send_goal(goal, options);

    current_goal_ = std::make_pair(x, y);
    prev_goal_ = current_goal_;
    navigating_ = true;
    last_progress_time_ = now();
    last_robot_pos_ = get_robot_position();
  }

  void navigate_through_poses(const std::vector<std::pair<double, double>> & waypoints)
  {
    if (!nav_through_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "NavigateThroughPoses action server not available!");
      return;
    }

    NavigateThroughPoses::Goal goal;
    const auto stamp = now();
    for (const auto & [wx, wy] : waypoints) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = global_frame_;
      pose.header.stamp = stamp;
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.orientation.w = 1.0;
      goal.poses.push_back(pose);
    }

    ++goal_seq_;
    const int seq = goal_seq_;
    const auto & last = waypoints.back();
    RCLCPP_INFO(
      get_logger(), "Navigating through %zu GVD waypoints -> (%.2f, %.2f)",
      waypoints.size(), last.first, last.second);

    typename rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions options;
    options.goal_response_callback = [this, seq](GoalHandleNavThrough::SharedPtr handle) {
        goal_response_cb(handle != nullptr, seq, [this, handle]() {
            nav_through_goal_handle_ = handle;
            nav_to_goal_handle_.reset();
          });
      };
    options.feedback_callback = [this](GoalHandleNavThrough::SharedPtr,
        const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback) {
        nav_feedback_cb(feedback->current_pose);
      };
    options.result_callback = [this, seq](const GoalHandleNavThrough::WrappedResult & result) {
        navigation_result_cb(result.code, seq);
      };
    nav_through_client_->async_send_goal(goal, options);

    current_goal_ = last;
    prev_goal_ = current_goal_;
    navigating_ = true;
    last_progress_time_ = now();
    last_robot_pos_ = get_robot_position();
  }

  void goal_response_cb(bool accepted, int seq, const std::function<void()> & store_handle)
  {
    auto started = Clock::now();
    if (seq != goal_seq_) {
      record_callback_timing("goal_response_cb", started);
      return;
    }

    if (!accepted) {
      RCLCPP_WARN(
        get_logger(),
        "Goal rejected by Nav2; blacklisting the current target to avoid a retry loop");
      blacklist_current_target();
      navigating_ = false;
      prev_goal_.reset();
      record_callback_timing("goal_response_cb", started);
      return;
    }

    RCLCPP_INFO(get_logger(), "Goal accepted");
    store_handle();
    record_callback_timing("goal_response_cb", started);
  }

  void navigation_result_cb(rclcpp_action::ResultCode code, int seq)
  {
    auto started = Clock::now();
    if (seq != goal_seq_) {
      record_callback_timing("navigation_result_cb", started);
      return;
    }

    if (code == rclcpp_action::ResultCode::SUCCEEDED) {
      RCLCPP_INFO(get_logger(), "Navigation succeeded");
      if (current_frontier_) {
        blacklist_point(current_frontier_->first, current_frontier_->second);
      }
    } else if (code == rclcpp_action::ResultCode::ABORTED) {
      RCLCPP_WARN(
        get_logger(),
        "Navigation aborted by Nav2; blacklisting the current target to avoid a retry loop");
      blacklist_current_target();
    } else if (code == rclcpp_action::ResultCode::CANCELED) {
      RCLCPP_INFO(get_logger(), "Navigation cancelled");
    } else {
      RCLCPP_WARN(get_logger(), "Navigation ended with status %d", static_cast<int>(code));
    }

    navigating_ = false;
    nav_to_goal_handle_.reset();
    nav_through_goal_handle_.reset();
    prev_goal_.reset();
    record_callback_timing("navigation_result_cb", started);
  }

  void return_to_initial_pose()
  {
    if (!initial_pose_ || navigating_) {
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Exploration complete - returning to initial pose (%.2f, %.2f)",
      initial_pose_->first, initial_pose_->second);
    exploring_ = false;
    navigate_to(initial_pose_->first, initial_pose_->second);
  }

  void check_progress(const std::pair<double, double> & robot_xy)
  {
    if (!navigating_ || !last_robot_pos_) {
      return;
    }

    const double dist_moved = std::hypot(
      robot_xy.first - last_robot_pos_->first,
      robot_xy.second - last_robot_pos_->second);
    if (dist_moved > 0.3) {
      last_progress_time_ = now();
      last_robot_pos_ = robot_xy;
      return;
    }

    const double elapsed = (now() - *last_progress_time_).seconds();
    if (elapsed > progress_timeout_) {
      RCLCPP_WARN(get_logger(), "No progress for %.0fs - cancelling goal", elapsed);
      cancel_current_goal();
      blacklist_current_goal();
      navigating_ = false;
    }
  }

  void cancel_current_goal()
  {
    if (nav_to_goal_handle_) {
      RCLCPP_INFO(get_logger(), "Cancelling current NavigateToPose goal...");
      nav_client_->async_cancel_goal(nav_to_goal_handle_);
      nav_to_goal_handle_.reset();
    }
    if (nav_through_goal_handle_) {
      RCLCPP_INFO(get_logger(), "Cancelling current NavigateThroughPoses goal...");
      nav_through_client_->async_cancel_goal(nav_through_goal_handle_);
      nav_through_goal_handle_.reset();
    }
  }

  void blacklist_current_goal()
  {
    if (current_goal_) {
      blacklist_point(current_goal_->first, current_goal_->second);
    }
  }

  void blacklist_current_target()
  {
    if (current_goal_ && !is_blacklisted(current_goal_->first, current_goal_->second)) {
      blacklist_point(current_goal_->first, current_goal_->second);
    }
    if (current_frontier_ &&
      !is_blacklisted(current_frontier_->first, current_frontier_->second))
    {
      blacklist_point(current_frontier_->first, current_frontier_->second);
    }
  }

  void blacklist_point(double x, double y)
  {
    RCLCPP_INFO(get_logger(), "Blacklisting (%.2f, %.2f)", x, y);
    blacklisted_.push_back({x, y, now()});
  }

  bool is_blacklisted(double x, double y) const
  {
    for (const auto & item : blacklisted_) {
      if (std::hypot(x - item.x, y - item.y) < blacklist_radius_) {
        return true;
      }
    }
    return false;
  }

  void prune_blacklist()
  {
    const auto now_time = now();
    blacklisted_.erase(
      std::remove_if(
        blacklisted_.begin(), blacklisted_.end(),
        [&](const BlacklistedPoint & item) {
          return (now_time - item.stamp).seconds() >= blacklist_timeout_;
        }),
      blacklisted_.end());
  }

  std::optional<std::pair<double, double>> get_robot_position()
  {
    if (!robot_pose_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Robot pose cache not ready yet");
      return std::nullopt;
    }

    if (using_nav_feedback_pose_) {
      const double pose_age = stamp_age_seconds(robot_pose_stamp_);
      if (navigating_ && pose_age > tf_tolerance_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Nav2 feedback pose is stale (%.2fs old)", pose_age);
      }
      return robot_pose_;
    }

    const double odom_age = stamp_age_seconds(odom_stamp_);
    if (odom_age > tf_tolerance_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Odom pose is stale (%.2fs old)", odom_age);
      return std::nullopt;
    }
    return robot_pose_;
  }

  void publish_path_markers(const std::vector<std::pair<double, double>> & waypoints)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.ns = "gvd_path";
    markers.markers.push_back(delete_marker);

    if (waypoints.size() >= 2U) {
      visualization_msgs::msg::Marker line;
      line.header.frame_id = global_frame_;
      line.header.stamp = now();
      line.ns = "gvd_path";
      line.id = 1;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.scale.x = 0.05;
      line.color.r = 1.0;
      line.color.b = 1.0;
      line.color.a = 1.0;
      line.lifetime = rclcpp::Duration::from_seconds(30.0);
      line.pose.orientation.w = 1.0;
      for (const auto & [wx, wy] : waypoints) {
        line.points.push_back(make_point(wx, wy, 0.08));
      }
      markers.markers.push_back(line);

      int marker_id = 10;
      for (const auto & [wx, wy] : waypoints) {
        visualization_msgs::msg::Marker point;
        point.header = line.header;
        point.ns = "gvd_path";
        point.id = marker_id++;
        point.type = visualization_msgs::msg::Marker::SPHERE;
        point.action = visualization_msgs::msg::Marker::ADD;
        point.pose.position.x = wx;
        point.pose.position.y = wy;
        point.pose.position.z = 0.08;
        point.pose.orientation.w = 1.0;
        point.scale.x = 0.1;
        point.scale.y = 0.1;
        point.scale.z = 0.1;
        point.color.r = 1.0;
        point.color.b = 1.0;
        point.color.a = 1.0;
        point.lifetime = rclcpp::Duration::from_seconds(30.0);
        markers.markers.push_back(point);
      }
    }

    path_marker_pub_->publish(markers);
  }

  void publish_gvd_markers()
  {
    if (!gvd_marker_pub_) {
      return;
    }
    ++gvd_marker_publish_counter_;
    if (((gvd_marker_publish_counter_ - 1) % gvd_marker_publish_every_) != 0) {
      return;
    }

    get_distance_transform();
    if (gvd_mask_.empty()) {
      return;
    }

    if (cached_gvd_marker_valid_ && same_stamp(cached_gvd_marker_stamp_, map_msg_->header.stamp)) {
      gvd_marker_pub_->publish(cached_gvd_marker_array_);
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    delete_marker.ns = "gvd";
    markers.markers.push_back(delete_marker);

    visualization_msgs::msg::Marker line;
    line.header.frame_id = global_frame_;
    line.header.stamp = now();
    line.ns = "gvd";
    line.id = 1;
    line.type = visualization_msgs::msg::Marker::LINE_LIST;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = 0.02;
    line.color.g = 1.0;
    line.color.b = 1.0;
    line.color.a = 0.8;
    line.pose.orientation.w = 1.0;

    int edge_counter = 0;
    for (int y = 0; y < height_; ++y) {
      for (int x = 0; x < width_; ++x) {
        if (!gvd_mask_[index(x, y)]) {
          continue;
        }
        for (const auto & edge : std::array<std::pair<int, int>, 4>{{{1, 0}, {0, 1}, {1, 1}, {1, -1}}}) {
          const int nx = x + edge.first;
          const int ny = y + edge.second;
          if (!in_bounds(nx, ny) || !gvd_mask_[index(nx, ny)]) {
            continue;
          }
          if ((edge_counter++ % gvd_marker_stride_) != 0) {
            continue;
          }
          const auto [wx0, wy0] = gvd_world_point_for_cell(x, y);
          const auto [wx1, wy1] = gvd_world_point_for_cell(nx, ny);
          line.points.push_back(make_point(wx0, wy0, 0.05));
          line.points.push_back(make_point(wx1, wy1, 0.05));
        }
      }
    }
    if (!line.points.empty()) {
      markers.markers.push_back(line);
    }

    cached_gvd_marker_array_ = markers;
    cached_gvd_marker_stamp_ = map_msg_->header.stamp;
    cached_gvd_marker_valid_ = true;
    gvd_marker_pub_->publish(markers);
  }

  void publish_markers(const std::vector<Frontier> & frontiers, const Frontier & chosen)
  {
    if (!marker_pub_) {
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_marker);

    int marker_id = 1;
    const auto stamp = now();
    for (const auto & frontier : frontiers) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = global_frame_;
      marker.header.stamp = stamp;
      marker.ns = "frontiers";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = frontier.centroid_x;
      marker.pose.position.y = frontier.centroid_y;
      marker.pose.position.z = 0.1;
      marker.pose.orientation.w = 1.0;
      const double scale = std::clamp(frontier.size * 0.005, 0.15, 0.6);
      marker.scale.x = scale;
      marker.scale.y = scale;
      marker.scale.z = scale;
      const bool chosen_marker =
        std::abs(frontier.centroid_x - chosen.centroid_x) < 1e-6 &&
        std::abs(frontier.centroid_y - chosen.centroid_y) < 1e-6;
      if (chosen_marker) {
        marker.color.g = 1.0;
        marker.color.a = 1.0;
      } else {
        marker.color.r = 0.2;
        marker.color.g = 0.4;
        marker.color.b = 1.0;
        marker.color.a = 0.8;
      }
      marker.lifetime = rclcpp::Duration::from_seconds(10.0);
      markers.markers.push_back(marker);
    }
    marker_pub_->publish(markers);
  }

  void clear_map_caches()
  {
    dist_map_.clear();
    label_map_.clear();
    gvd_mask_.clear();
    site_xs_.clear();
    site_ys_.clear();
    gvd_world_x_.clear();
    gvd_world_y_.clear();
    cached_gvd_marker_valid_ = false;
  }

  void store_map_to_odom(double tx, double ty, double yaw, const builtin_interfaces::msg::Time & stamp)
  {
    map_to_odom_xy_ = std::make_pair(tx, ty);
    map_to_odom_yaw_ = yaw;
    map_to_odom_stamp_ = stamp;
  }

  void update_robot_pose_cache()
  {
    if (!map_to_odom_xy_ || !odom_pose_ || !map_to_odom_yaw_) {
      return;
    }
    const double cos_y = std::cos(*map_to_odom_yaw_);
    const double sin_y = std::sin(*map_to_odom_yaw_);
    robot_pose_ = std::make_pair(
      map_to_odom_xy_->first + cos_y * odom_pose_->first - sin_y * odom_pose_->second,
      map_to_odom_xy_->second + sin_y * odom_pose_->first + cos_y * odom_pose_->second);
    robot_pose_stamp_ = odom_stamp_;
    if (tf_sub_ || tf_static_sub_) {
      RCLCPP_INFO(get_logger(), "Initial map pose cached; switching to odom-delta tracking");
      tf_sub_.reset();
      tf_static_sub_.reset();
    }
  }

  double stamp_age_seconds(const builtin_interfaces::msg::Time & stamp) const
  {
    const auto now_time = now();
    if (now_time.nanoseconds() <= 0) {
      return 0.0;
    }
    return std::max(0.0, (now_time - rclcpp::Time(stamp)).seconds());
  }

  void record_timing(const std::string & stage, const Clock::time_point & started)
  {
    if (!profile_timing_) {
      return;
    }
    const double duration = std::chrono::duration<double>(Clock::now() - started).count();
    timing_totals_[stage] += duration;
    timing_counts_[stage] += 1;
    timing_max_[stage] = std::max(timing_max_[stage], duration);
  }

  void maybe_log_timing()
  {
    if (!profile_timing_ || timing_totals_.empty()) {
      return;
    }
    const auto now_tp = Clock::now();
    const double elapsed = std::chrono::duration<double>(now_tp - timing_last_log_).count();
    if (elapsed < profile_log_interval_) {
      return;
    }
    const double total_time = timing_totals_["plan_total"];
    std::vector<std::pair<std::string, double>> entries(timing_totals_.begin(), timing_totals_.end());
    std::sort(entries.begin(), entries.end(), [](const auto & a, const auto & b) {return a.second > b.second;});
    RCLCPP_INFO(get_logger(), "Frontier timing over %d plans (%.1fs window):", timing_plan_counter_, elapsed);
    for (const auto & [stage, total] : entries) {
      const int count = std::max(1, timing_counts_[stage]);
      const double avg_ms = 1000.0 * total / count;
      const double max_ms = 1000.0 * timing_max_[stage];
      const double share = total_time > 0.0 ? 100.0 * total / total_time : 0.0;
      RCLCPP_INFO(get_logger(), "  %s: avg=%.1fms  max=%.1fms  share=%.1f%%", stage.c_str(), avg_ms, max_ms, share);
    }
    timing_totals_.clear();
    timing_counts_.clear();
    timing_max_.clear();
    timing_last_log_ = now_tp;
    timing_plan_counter_ = 0;
  }

  void record_callback_timing(const std::string & callback_name, const Clock::time_point & started)
  {
    if (!profile_callbacks_) {
      return;
    }
    const double duration = std::chrono::duration<double>(Clock::now() - started).count();
    callback_totals_[callback_name] += duration;
    callback_counts_[callback_name] += 1;
    callback_max_[callback_name] = std::max(callback_max_[callback_name], duration);
    maybe_log_callback_timing();
  }

  void maybe_log_callback_timing()
  {
    if (!profile_callbacks_ || callback_totals_.empty()) {
      return;
    }
    const auto now_tp = Clock::now();
    const double window_s = std::chrono::duration<double>(now_tp - callback_last_log_).count();
    if (window_s < profile_callback_log_interval_) {
      return;
    }
    std::vector<std::pair<std::string, double>> entries(callback_totals_.begin(), callback_totals_.end());
    std::sort(entries.begin(), entries.end(), [](const auto & a, const auto & b) {return a.second > b.second;});
    double total = 0.0;
    for (const auto & entry : entries) {
      total += entry.second;
    }
    RCLCPP_INFO(get_logger(), "Frontier callback timing (%.1fs window):", window_s);
    for (const auto & [name, value] : entries) {
      const int count = std::max(1, callback_counts_[name]);
      const double avg_ms = 1000.0 * value / count;
      const double max_ms = 1000.0 * callback_max_[name];
      const double rate_hz = count / std::max(window_s, 1e-6);
      const double share = total > 0.0 ? 100.0 * value / total : 0.0;
      const double wall = 100.0 * value / std::max(window_s, 1e-6);
      RCLCPP_INFO(
        get_logger(),
        "  %s: rate=%.1fHz  avg=%.3fms  max=%.3fms  share=%.1f%%  wall=%.1f%%",
        name.c_str(), rate_hz, avg_ms, max_ms, share, wall);
    }
    RCLCPP_INFO(get_logger(), "  callback_busy_total: %.1f%%", 100.0 * total / std::max(window_s, 1e-6));
    callback_totals_.clear();
    callback_counts_.clear();
    callback_max_.clear();
    callback_last_log_ = now_tp;
  }

  static std::string normalize_frame_id(const std::string & frame_id)
  {
    if (!frame_id.empty() && frame_id.front() == '/') {
      return frame_id.substr(1);
    }
    return frame_id;
  }

  static double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & quat)
  {
    const double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    const double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  bool in_bounds(int x, int y) const
  {
    return x >= 0 && x < width_ && y >= 0 && y < height_;
  }

  int index(int x, int y) const
  {
    return y * width_ + x;
  }

  int8_t map_value(int x, int y) const
  {
    return map_array_[static_cast<size_t>(index(x, y))];
  }

  bool is_obstacle(int idx) const
  {
    return map_array_[static_cast<size_t>(idx)] > 50;
  }

  double planner_freq_{0.5};
  int min_frontier_size_{5};
  std::string robot_base_frame_;
  std::string odom_frame_;
  std::string odom_topic_;
  std::string global_frame_;
  std::string map_topic_;
  std::string live_map_ready_topic_;
  double tf_tolerance_{2.0};
  double blacklist_radius_{0.5};
  double blacklist_timeout_{60.0};
  double progress_timeout_{30.0};
  bool visualize_{true};
  double clearance_scale_{0.3};
  bool return_to_init_{false};
  int gvd_min_clearance_{3};
  double gvd_snap_radius_{2.0};
  bool visualize_gvd_{true};
  int gvd_marker_publish_every_{1};
  int gvd_marker_stride_{1};
  bool profile_timing_{false};
  double profile_log_interval_{15.0};
  bool profile_callbacks_{false};
  double profile_callback_log_interval_{10.0};
  bool require_live_map_{true};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr live_map_ready_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr resume_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gvd_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr nav_through_client_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
  std::vector<int8_t> map_array_;
  int width_{0};
  int height_{0};
  double resolution_{0.05};
  double origin_x_{0.0};
  double origin_y_{0.0};

  bool exploring_{true};
  bool live_map_ready_{false};
  bool navigating_{false};
  int goal_seq_{0};
  std::optional<std::pair<double, double>> current_goal_;
  std::optional<std::pair<double, double>> current_frontier_;
  std::optional<std::pair<double, double>> prev_goal_;
  std::optional<std::pair<double, double>> last_robot_pos_;
  std::optional<std::pair<double, double>> initial_pose_;
  std::optional<rclcpp::Time> last_progress_time_;
  std::vector<BlacklistedPoint> blacklisted_;

  GoalHandleNavTo::SharedPtr nav_to_goal_handle_;
  GoalHandleNavThrough::SharedPtr nav_through_goal_handle_;

  std::vector<double> dist_map_;
  std::vector<int> label_map_;
  std::vector<uint8_t> gvd_mask_;
  std::vector<int> site_xs_;
  std::vector<int> site_ys_;
  std::vector<double> gvd_world_x_;
  std::vector<double> gvd_world_y_;
  builtin_interfaces::msg::Time cached_map_stamp_;

  visualization_msgs::msg::MarkerArray cached_gvd_marker_array_;
  builtin_interfaces::msg::Time cached_gvd_marker_stamp_;
  bool cached_gvd_marker_valid_{false};
  int gvd_marker_publish_counter_{0};

  std::optional<std::pair<double, double>> odom_pose_;
  builtin_interfaces::msg::Time odom_stamp_;
  std::optional<std::pair<double, double>> map_to_odom_xy_;
  std::optional<double> map_to_odom_yaw_;
  builtin_interfaces::msg::Time map_to_odom_stamp_;
  std::optional<std::pair<double, double>> robot_pose_;
  builtin_interfaces::msg::Time robot_pose_stamp_;
  bool using_nav_feedback_pose_{false};

  std::unordered_map<std::string, double> timing_totals_;
  std::unordered_map<std::string, int> timing_counts_;
  std::unordered_map<std::string, double> timing_max_;
  Clock::time_point timing_last_log_;
  int timing_plan_counter_{0};

  std::unordered_map<std::string, double> callback_totals_;
  std::unordered_map<std::string, int> callback_counts_;
  std::unordered_map<std::string, double> callback_max_;
  Clock::time_point callback_last_log_;

  const std::array<std::pair<int, int>, 4> neighbors4_{{
    std::pair<int, int>{1, 0},
    std::pair<int, int>{-1, 0},
    std::pair<int, int>{0, 1},
    std::pair<int, int>{0, -1},
  }};
  const std::array<std::pair<int, int>, 8> neighbors8_{{
    std::pair<int, int>{-1, -1},
    std::pair<int, int>{-1, 0},
    std::pair<int, int>{-1, 1},
    std::pair<int, int>{0, -1},
    std::pair<int, int>{0, 1},
    std::pair<int, int>{1, -1},
    std::pair<int, int>{1, 0},
    std::pair<int, int>{1, 1},
  }};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierExplorerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
