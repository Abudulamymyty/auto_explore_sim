#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
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

#include "auto_explore_sim/exploration_types.hpp"
#include "auto_explore_sim/frontier_search.hpp"
#include "auto_explore_sim/gvd_map.hpp"

namespace
{

constexpr double kDirectFrontierGoalSearchRadius = 1.0;

struct BlacklistedPoint
{
  double x{0.0};
  double y{0.0};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
};

struct CompletedPoint
{
  double x{0.0};
  double y{0.0};
};

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

inline geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion quat;
  quat.z = std::sin(0.5 * yaw);
  quat.w = std::cos(0.5 * yaw);
  return quat;
}

inline void hash_u64(std::uint64_t * hash, std::uint64_t value)
{
  *hash ^= value;
  *hash *= 1099511628211ULL;
}

inline std::uint64_t quantize_hash_value(double value, double scale)
{
  return static_cast<std::uint64_t>(std::llround(value * scale));
}

inline std::uint64_t compute_map_content_hash(const nav_msgs::msg::OccupancyGrid & map)
{
  std::uint64_t hash = 1469598103934665603ULL;
  hash_u64(&hash, static_cast<std::uint64_t>(map.info.width));
  hash_u64(&hash, static_cast<std::uint64_t>(map.info.height));
  hash_u64(&hash, quantize_hash_value(map.info.resolution, 1e6));
  hash_u64(&hash, quantize_hash_value(map.info.origin.position.x, 1e6));
  hash_u64(&hash, quantize_hash_value(map.info.origin.position.y, 1e6));
  for (const int8_t value : map.data) {
    hash_u64(&hash, static_cast<std::uint8_t>(value));
  }
  return hash;
}

}  // namespace

class FrontierExplorerNode : public rclcpp::Node
{
public:
  using Frontier = auto_explore_sim::Frontier;
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
    declare_parameter("frontier_plan_retry_limit", 3);
    declare_parameter("progress_timeout", 5.0);
    declare_parameter("visualize", true);
    declare_parameter("clearance_scale", 0.3);
    declare_parameter("frontier_update_radius", 3.0);
    declare_parameter("return_to_init", false);
    declare_parameter("gvd_min_clearance", 3);
    declare_parameter("gvd_snap_radius", 2.0);
    declare_parameter("gvd_distance_tolerance_cells", 1.0);
    declare_parameter("gvd_max_site_direction_dot", 0.7);
    declare_parameter("allow_gvd_bridge", false);
    declare_parameter("allow_safe_direct_frontier_nav", true);
    declare_parameter("direct_frontier_min_clearance_cells", 6.0);
    declare_parameter("gvd_heuristic_chain_sample_step", 0.6);
    declare_parameter("gvd_heuristic_chain_max_offset", 1.0);
    declare_parameter("visualize_gvd", true);
    declare_parameter("gvd_marker_publish_every", 1);
    declare_parameter("gvd_marker_stride", 1);
    declare_parameter("gvd_waypoint_spacing", 0.18);
    declare_parameter("gvd_waypoint_smoothing_passes", 2);
    declare_parameter("gvd_segment_length", 1.0);
    declare_parameter("gvd_segment_length_min", 0.4);
    declare_parameter("gvd_segment_length_max", 6.0);
    declare_parameter("gvd_jerk_feedback_threshold", 5);
    declare_parameter("gvd_jerk_progress_epsilon", 0.03);
    declare_parameter("gvd_jerk_replan_cooldown", 2.0);
    declare_parameter("stuck_recovery_gvd_radius", 3.0);
    declare_parameter("replan_while_navigating", true);
    declare_parameter("gvd_replan_interval", 3.0);
    declare_parameter("gvd_replan_min_remaining_distance", 0.8);
    declare_parameter("gvd_goal_pass_max_goal_distance", 0.45);
    declare_parameter("gvd_goal_pass_projection_margin", 0.08);
    declare_parameter("gvd_goal_pass_lateral_tolerance", 0.35);
    declare_parameter("profile_timing", false);
    declare_parameter("profile_log_interval", 15.0);
    declare_parameter("profile_callbacks", false);
    declare_parameter("profile_callback_log_interval", 10.0);

    planner_freq_ = get_parameter("planner_frequency").as_double();
    frontier_search_config_.min_frontier_size = get_parameter("min_frontier_size").as_int();
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
    frontier_plan_retry_limit_ =
      std::max(1, static_cast<int>(get_parameter("frontier_plan_retry_limit").as_int()));
    progress_timeout_ = get_parameter("progress_timeout").as_double();
    visualize_ = get_parameter("visualize").as_bool();
    frontier_search_config_.clearance_scale = get_parameter("clearance_scale").as_double();
    frontier_search_config_.frontier_update_radius =
      std::max(0.0, get_parameter("frontier_update_radius").as_double());
    return_to_init_ = get_parameter("return_to_init").as_bool();
    gvd_config_.min_clearance = get_parameter("gvd_min_clearance").as_int();
    gvd_config_.snap_radius = get_parameter("gvd_snap_radius").as_double();
    gvd_config_.distance_tolerance_cells =
      std::max(0.1, get_parameter("gvd_distance_tolerance_cells").as_double());
    gvd_config_.max_site_direction_dot = std::clamp(
      get_parameter("gvd_max_site_direction_dot").as_double(), -1.0, 1.0);
    gvd_config_.allow_bridge = get_parameter("allow_gvd_bridge").as_bool();
    allow_safe_direct_frontier_nav_ =
      get_parameter("allow_safe_direct_frontier_nav").as_bool();
    direct_frontier_min_clearance_cells_ = std::max(
      0.0, get_parameter("direct_frontier_min_clearance_cells").as_double());
    gvd_config_.heuristic_chain_sample_step =
      std::max(0.05, get_parameter("gvd_heuristic_chain_sample_step").as_double());
    gvd_config_.heuristic_chain_max_offset =
      std::max(0.05, get_parameter("gvd_heuristic_chain_max_offset").as_double());
    visualize_gvd_ = get_parameter("visualize_gvd").as_bool();
    gvd_marker_publish_every_ =
      std::max(1, static_cast<int>(get_parameter("gvd_marker_publish_every").as_int()));
    gvd_marker_stride_ =
      std::max(1, static_cast<int>(get_parameter("gvd_marker_stride").as_int()));
    gvd_waypoint_spacing_ = std::max(0.05, get_parameter("gvd_waypoint_spacing").as_double());
    gvd_waypoint_smoothing_passes_ =
      std::max(0, static_cast<int>(get_parameter("gvd_waypoint_smoothing_passes").as_int()));
    gvd_segment_length_ = std::max(0.2, get_parameter("gvd_segment_length").as_double());
    gvd_segment_length_min_ = std::max(
      0.1, get_parameter("gvd_segment_length_min").as_double());
    gvd_segment_length_max_ = std::max(
      gvd_segment_length_min_, get_parameter("gvd_segment_length_max").as_double());
    adaptive_gvd_segment_length_ = std::clamp(
      gvd_segment_length_, gvd_segment_length_min_, gvd_segment_length_max_);
    gvd_jerk_feedback_threshold_ = std::max(
      2, static_cast<int>(get_parameter("gvd_jerk_feedback_threshold").as_int()));
    gvd_jerk_progress_epsilon_ = std::max(
      0.005, get_parameter("gvd_jerk_progress_epsilon").as_double());
    gvd_jerk_replan_cooldown_ = std::max(
      0.2, get_parameter("gvd_jerk_replan_cooldown").as_double());
    stuck_recovery_gvd_radius_ =
      std::max(0.5, get_parameter("stuck_recovery_gvd_radius").as_double());
    replan_while_navigating_ = get_parameter("replan_while_navigating").as_bool();
    gvd_replan_interval_ = get_parameter("gvd_replan_interval").as_double();
    gvd_replan_min_remaining_distance_ =
      get_parameter("gvd_replan_min_remaining_distance").as_double();
    gvd_goal_pass_max_goal_distance_ = std::max(
      0.05, get_parameter("gvd_goal_pass_max_goal_distance").as_double());
    gvd_goal_pass_projection_margin_ = std::max(
      0.01, get_parameter("gvd_goal_pass_projection_margin").as_double());
    gvd_goal_pass_lateral_tolerance_ = std::max(
      0.05, get_parameter("gvd_goal_pass_lateral_tolerance").as_double());
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
      marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("explore/frontiers", 10);
    }
    if (visualize_gvd_) {
      gvd_marker_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("explore/gvd", 10);
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
      "frontier_update_radius=%.2fm, map_topic=%s, gvd_snap_radius=%.2fm, "
      "gvd_distance_tol=%.2f cells, gvd_max_dot=%.2f, allow_gvd_bridge=%s, "
      "allow_safe_direct_frontier_nav=%s, direct_frontier_min_clearance=%.2f cells, "
      "gvd_marker_every=%d, "
      "gvd_marker_stride=%d, "
      "gvd_segment_length(base=%.2fm, min=%.2fm, max=%.2fm), "
      "stuck_recovery_gvd_radius=%.2fm, "
      "frontier_plan_retry_limit=%d)",
      planner_freq_, frontier_search_config_.min_frontier_size,
      frontier_search_config_.clearance_scale,
      frontier_search_config_.frontier_update_radius,
      map_topic_.c_str(), gvd_config_.snap_radius,
      gvd_config_.distance_tolerance_cells, gvd_config_.max_site_direction_dot,
      gvd_config_.allow_bridge ? "true" : "false",
      allow_safe_direct_frontier_nav_ ? "true" : "false",
      direct_frontier_min_clearance_cells_,
      gvd_marker_publish_every_, gvd_marker_stride_,
      gvd_segment_length_, gvd_segment_length_min_, gvd_segment_length_max_,
      stuck_recovery_gvd_radius_, frontier_plan_retry_limit_);
  }

private:
  using Clock = std::chrono::steady_clock;
  enum class ActiveNavigationMode { None, SingleGoal, GvdWaypoints };
  enum class FrontierPlanMode { None, DirectGoal, GvdWaypoints };
  enum class FrontierNodeState {
    kObserved,
    kDeferred,
    kDirectFallbackReady,
    kLocked,
    kCompleted,
    kBlacklisted,
  };

  struct FrontierNavigationPlan
  {
    std::pair<double, double> frontier;
    std::pair<double, double> anchor;
    std::pair<double, double> goal;
    FrontierPlanMode mode{FrontierPlanMode::None};
    bool has_gvd_anchor{false};
    bool frontier_snapped{false};
    bool using_gvd_path{false};
    bool used_bridge{false};
    bool used_heuristic_chain{false};
    bool direct_goal_is_frontier{false};
    bool direct_goal_is_gvd_entry{false};
    bool direct_goal_adjusted{false};
    bool segment_reaches_anchor{false};
    bool path_quality_good{false};
    int bridge_trigger_explored_cells{0};
    int skeleton_cell_count{0};
    size_t path_cell_count{0};
    size_t full_waypoint_count{0};
    double full_path_length{0.0};
    double max_waypoint_gap{0.0};
    double segment_length_limit{0.0};
    double direct_path_min_clearance_cells{0.0};
    int direct_path_explored_cells{0};
    std::vector<std::pair<double, double>> waypoints;
  };

  struct FrontierGraphNode
  {
    Frontier frontier;
    FrontierNodeState state{FrontierNodeState::kObserved};
    FrontierPlanMode preferred_plan_mode{FrontierPlanMode::None};
    bool in_current_observation{false};
    bool has_gvd_anchor{false};
    bool has_same_component_gvd_path{false};
    bool has_direct_fallback{false};
    std::string last_reason;
    std::vector<std::string> neighbors;
  };

  struct FrontierPlanEvaluation
  {
    std::optional<FrontierNavigationPlan> plan;
    FrontierNodeState node_state{FrontierNodeState::kDeferred};
    bool should_complete_frontier{false};
    bool has_gvd_anchor{false};
    bool has_same_component_gvd_path{false};
    bool has_direct_fallback{false};
    std::string reason;
  };

  auto_explore_sim::GridMapView make_grid_view() const
  {
    return auto_explore_sim::GridMapView{
      &map_array_,
      width_,
      height_,
      resolution_,
      origin_x_,
      origin_y_,
    };
  }

  static double polyline_length(const std::vector<std::pair<double, double>> & points)
  {
    if (points.size() < 2U) {
      return 0.0;
    }

    double length = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
      length += std::hypot(
        points[i].first - points[i - 1].first,
        points[i].second - points[i - 1].second);
    }
    return length;
  }

  static double max_waypoint_gap(const std::vector<std::pair<double, double>> & points)
  {
    double max_gap = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
      max_gap = std::max(
        max_gap,
        std::hypot(
          points[i].first - points[i - 1].first,
          points[i].second - points[i - 1].second));
    }
    return max_gap;
  }

  bool is_gvd_path_quality_good(const FrontierNavigationPlan & plan) const
  {
    if (!plan.using_gvd_path || plan.full_waypoint_count < 2U) {
      return false;
    }
    if (plan.used_bridge || plan.used_heuristic_chain) {
      return false;
    }

    const double skeleton_ratio = static_cast<double>(plan.skeleton_cell_count) /
      std::max<double>(1.0, static_cast<double>(plan.path_cell_count));
    const double gap_limit = std::max(2.0 * gvd_waypoint_spacing_, 4.0 * resolution_);
    return skeleton_ratio >= 0.98 && plan.max_waypoint_gap <= gap_limit;
  }

  double select_gvd_segment_length(const FrontierNavigationPlan & plan) const
  {
    if (plan.full_path_length <= 0.0) {
      return gvd_segment_length_min_;
    }
    if (plan.path_quality_good) {
      return plan.full_path_length;
    }

    return std::clamp(
      adaptive_gvd_segment_length_,
      gvd_segment_length_min_,
      std::min(gvd_segment_length_max_, plan.full_path_length));
  }

  void grow_adaptive_gvd_segment_length()
  {
    const double next = std::min(
      gvd_segment_length_max_,
      std::max(gvd_segment_length_min_, adaptive_gvd_segment_length_ * 1.35));
    if (next <= adaptive_gvd_segment_length_ + 1e-3) {
      return;
    }

    adaptive_gvd_segment_length_ = next;
    RCLCPP_INFO(
      get_logger(),
      "GVD tracking stayed stable; increasing adaptive gvd_segment_length to %.2fm",
      adaptive_gvd_segment_length_);
  }

  void shrink_adaptive_gvd_segment_length(const char * reason)
  {
    const double next = std::max(
      gvd_segment_length_min_,
      std::min(gvd_segment_length_max_, adaptive_gvd_segment_length_ * 0.5));
    if (next >= adaptive_gvd_segment_length_ - 1e-3) {
      return;
    }

    adaptive_gvd_segment_length_ = next;
    RCLCPP_WARN(
      get_logger(),
      "%s; shortening adaptive gvd_segment_length to %.2fm",
      reason, adaptive_gvd_segment_length_);
  }

  void maybe_force_shorter_gvd_replan(const std::pair<double, double> & robot_xy)
  {
    if (!navigating_ ||
      active_navigation_mode_ != ActiveNavigationMode::GvdWaypoints ||
      !current_goal_ || !current_frontier_ || pending_replan_after_recovery_)
    {
      return;
    }

    if (last_gvd_jerk_time_ &&
      (now() - *last_gvd_jerk_time_).seconds() < gvd_jerk_replan_cooldown_)
    {
      return;
    }

    shrink_adaptive_gvd_segment_length(
      "Detected repeated low-progress GVD feedback while following the current segment");
    last_gvd_jerk_time_ = now();
    force_gvd_replan_ = true;
    active_plan_robot_xy_ = robot_xy;
    last_feedback_goal_distance_.reset();
    last_feedback_xy_.reset();
    gvd_low_progress_feedback_count_ = 0;
  }

  void clear_waiting_for_usable_frontier_on_current_map()
  {
    waiting_for_usable_frontier_on_current_map_ = false;
    usable_frontier_wait_map_hash_ = 0;
    usable_frontier_wait_map_hash_valid_ = false;
  }

  bool has_passed_active_gvd_goal(const std::pair<double, double> & robot_xy) const
  {
    if (
      active_navigation_mode_ != ActiveNavigationMode::GvdWaypoints ||
      active_waypoints_.size() < 2U || !current_goal_ || !last_feedback_goal_distance_)
    {
      return false;
    }

    const auto & goal = active_waypoints_.back();
    const auto & previous = active_waypoints_[active_waypoints_.size() - 2];
    const double segment_dx = goal.first - previous.first;
    const double segment_dy = goal.second - previous.second;
    const double segment_length = std::hypot(segment_dx, segment_dy);
    if (segment_length <= 1e-6) {
      return false;
    }

    const double goal_distance = std::hypot(
      robot_xy.first - goal.first, robot_xy.second - goal.second);
    if (goal_distance > gvd_goal_pass_max_goal_distance_) {
      return false;
    }

    const double goal_to_robot_x = robot_xy.first - goal.first;
    const double goal_to_robot_y = robot_xy.second - goal.second;
    const double projection_after_goal =
      (goal_to_robot_x * segment_dx + goal_to_robot_y * segment_dy) / segment_length;
    if (projection_after_goal < gvd_goal_pass_projection_margin_) {
      return false;
    }

    const double lateral_offset = std::abs(
      goal_to_robot_x * segment_dy - goal_to_robot_y * segment_dx) / segment_length;
    return lateral_offset <= gvd_goal_pass_lateral_tolerance_;
  }

  double frontier_match_radius() const
  {
    return std::max(1.0, 2.0 * blacklist_radius_);
  }

  double frontier_completion_radius() const
  {
    return std::max(blacklist_radius_, 1.0);
  }

  bool frontier_matches_point(const Frontier & frontier, double x, double y) const
  {
    return std::hypot(frontier.centroid_x - x, frontier.centroid_y - y) <= frontier_match_radius();
  }

  bool frontiers_match(const Frontier & lhs, const Frontier & rhs) const
  {
    return frontier_matches_point(lhs, rhs.centroid_x, rhs.centroid_y);
  }

  bool satisfies_min_frontier_size(const Frontier & frontier) const
  {
    return frontier.size >= frontier_search_config_.min_frontier_size;
  }

  void refresh_frontier_metrics(
    Frontier * frontier, const std::pair<double, double> & robot_xy) const
  {
    frontier->heuristic_distance = std::hypot(
      frontier->centroid_x - robot_xy.first, frontier->centroid_y - robot_xy.second);
    frontier->min_distance = frontier->heuristic_distance;
    frontier->cost = frontier->heuristic_distance;
  }

  bool goal_reaches_frontier(
    const std::pair<double, double> & goal,
    const std::pair<double, double> & frontier) const
  {
    return std::hypot(goal.first - frontier.first, goal.second - frontier.second) <=
           frontier_completion_radius();
  }

  double minimum_dispatch_goal_distance() const
  {
    return std::max(0.3, gvd_goal_pass_max_goal_distance_);
  }

  bool points_are_near(
    const std::pair<double, double> & lhs,
    const std::pair<double, double> & rhs,
    double tolerance) const
  {
    return std::hypot(lhs.first - rhs.first, lhs.second - rhs.second) <= tolerance;
  }

  void remember_reached_intermediate_anchor()
  {
    if (!current_goal_ || !current_frontier_) {
      return;
    }

    last_reached_intermediate_anchor_ = *current_goal_;
    last_reached_intermediate_frontier_ = *current_frontier_;
    last_reached_intermediate_anchor_map_hash_ = map_content_hash_;
    last_reached_intermediate_anchor_map_hash_valid_ = map_content_hash_valid_;
  }

  void clear_reached_intermediate_anchor()
  {
    last_reached_intermediate_anchor_.reset();
    last_reached_intermediate_frontier_.reset();
    last_reached_intermediate_anchor_map_hash_ = 0;
    last_reached_intermediate_anchor_map_hash_valid_ = false;
  }

  bool repeats_last_reached_intermediate_anchor(
    const FrontierNavigationPlan & plan) const
  {
    if (!last_reached_intermediate_anchor_ ||
      !last_reached_intermediate_frontier_)
    {
      return false;
    }

    if (!points_are_near(
        plan.frontier, *last_reached_intermediate_frontier_, frontier_match_radius()))
    {
      return false;
    }

    return points_are_near(
      plan.goal, *last_reached_intermediate_anchor_, minimum_dispatch_goal_distance());
  }

  bool plan_reuses_stale_intermediate_anchor(
    const FrontierNavigationPlan & plan) const
  {
    if (!repeats_last_reached_intermediate_anchor(plan)) {
      return false;
    }

    const double previous_anchor_distance_to_frontier = std::hypot(
      last_reached_intermediate_anchor_->first - plan.frontier.first,
      last_reached_intermediate_anchor_->second - plan.frontier.second);
    const double new_goal_distance_to_frontier = std::hypot(
      plan.goal.first - plan.frontier.first,
      plan.goal.second - plan.frontier.second);
    const double required_progress = std::max(0.1, 2.0 * resolution_);

    return new_goal_distance_to_frontier >=
           previous_anchor_distance_to_frontier - required_progress;
  }

  static std::string frontier_storage_key(double x, double y)
  {
    const auto ix = static_cast<long long>(std::llround(x * 100.0));
    const auto iy = static_cast<long long>(std::llround(y * 100.0));
    return std::to_string(ix) + ":" + std::to_string(iy);
  }

  const FrontierGraphNode * find_frontier_graph_node(double x, double y) const
  {
    const auto exact = frontier_graph_.find(frontier_storage_key(x, y));
    if (exact != frontier_graph_.end()) {
      return &exact->second;
    }

    const Frontier probe = [x, y]() {
        Frontier frontier;
        frontier.centroid_x = x;
        frontier.centroid_y = y;
        return frontier;
      }();
    for (const auto & [key, node] : frontier_graph_) {
      (void)key;
      if (frontiers_match(node.frontier, probe)) {
        return &node;
      }
    }
    return nullptr;
  }

  FrontierGraphNode * find_frontier_graph_node(double x, double y)
  {
    return const_cast<FrontierGraphNode *>(
      static_cast<const FrontierExplorerNode *>(this)->find_frontier_graph_node(x, y));
  }

  std::optional<std::string> find_matching_frontier_graph_key(
    const Frontier & frontier) const
  {
    const auto exact_key = frontier_storage_key(frontier.centroid_x, frontier.centroid_y);
    if (frontier_graph_.find(exact_key) != frontier_graph_.end()) {
      return exact_key;
    }

    for (const auto & [key, node] : frontier_graph_) {
      if (frontiers_match(node.frontier, frontier)) {
        return key;
      }
    }
    return std::nullopt;
  }

  FrontierGraphNode & upsert_frontier_graph_node(const Frontier & frontier)
  {
    const auto matched_key = find_matching_frontier_graph_key(frontier);
    if (matched_key) {
      auto & node = frontier_graph_.at(*matched_key);
      node.frontier = frontier;
      return node;
    }

    const auto key = frontier_storage_key(frontier.centroid_x, frontier.centroid_y);
    auto [it, inserted] = frontier_graph_.try_emplace(key);
    (void)inserted;
    it->second.frontier = frontier;
    return it->second;
  }

  void set_frontier_graph_node_state(
    double x, double y, FrontierNodeState state, FrontierPlanMode preferred_plan_mode,
    const std::string & reason, bool has_gvd_anchor, bool has_same_component_gvd_path,
    bool has_direct_fallback)
  {
    FrontierGraphNode * node = find_frontier_graph_node(x, y);
    if (!node) {
      Frontier frontier;
      frontier.centroid_x = x;
      frontier.centroid_y = y;
      node = &upsert_frontier_graph_node(frontier);
    }

    node->state = state;
    node->preferred_plan_mode = preferred_plan_mode;
    node->has_gvd_anchor = has_gvd_anchor;
    node->has_same_component_gvd_path = has_same_component_gvd_path;
    node->has_direct_fallback = has_direct_fallback;
    node->last_reason = reason;
  }

  void rebuild_frontier_graph_neighbors(const std::vector<std::string> & observed_keys)
  {
    const double neighbor_radius = std::max(
      frontier_match_radius() * 4.0,
      frontier_search_config_.frontier_update_radius > 0.0 ?
      frontier_search_config_.frontier_update_radius : 2.0);

    for (const auto & key : observed_keys) {
      auto it = frontier_graph_.find(key);
      if (it != frontier_graph_.end()) {
        it->second.neighbors.clear();
      }
    }

    for (size_t i = 0; i < observed_keys.size(); ++i) {
      for (size_t j = i + 1; j < observed_keys.size(); ++j) {
        auto lhs_it = frontier_graph_.find(observed_keys[i]);
        auto rhs_it = frontier_graph_.find(observed_keys[j]);
        if (lhs_it == frontier_graph_.end() || rhs_it == frontier_graph_.end()) {
          continue;
        }

        const double distance = std::hypot(
          lhs_it->second.frontier.centroid_x - rhs_it->second.frontier.centroid_x,
          lhs_it->second.frontier.centroid_y - rhs_it->second.frontier.centroid_y);
        if (distance > neighbor_radius) {
          continue;
        }

        lhs_it->second.neighbors.push_back(observed_keys[j]);
        rhs_it->second.neighbors.push_back(observed_keys[i]);
      }
    }
  }

  void forget_known_frontier(double x, double y)
  {
    for (auto it = frontier_graph_.begin(); it != frontier_graph_.end();) {
      if (frontier_matches_point(it->second.frontier, x, y)) {
        it = frontier_graph_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void prune_known_frontiers()
  {
    for (auto it = frontier_graph_.begin(); it != frontier_graph_.end();) {
      auto & node = it->second;
      if (!satisfies_min_frontier_size(node.frontier)) {
        it = frontier_graph_.erase(it);
        continue;
      }

      if (is_blacklisted(node.frontier.centroid_x, node.frontier.centroid_y)) {
        node.state = FrontierNodeState::kBlacklisted;
        node.in_current_observation = false;
      } else if (is_completed(node.frontier.centroid_x, node.frontier.centroid_y)) {
        node.state = FrontierNodeState::kCompleted;
        node.in_current_observation = false;
      }
      ++it;
    }
  }

  void refresh_known_frontiers(
    const std::vector<Frontier> & observed_frontiers,
    const std::pair<double, double> & robot_xy)
  {
    prune_known_frontiers();
    for (auto & [key, node] : frontier_graph_) {
      (void)key;
      node.in_current_observation = false;
      if (
        node.state != FrontierNodeState::kCompleted &&
        node.state != FrontierNodeState::kBlacklisted)
      {
        node.state = FrontierNodeState::kDeferred;
      }
      node.has_gvd_anchor = false;
      node.has_same_component_gvd_path = false;
      node.has_direct_fallback = false;
      node.preferred_plan_mode = FrontierPlanMode::None;
      node.last_reason.clear();
    }

    std::vector<std::string> observed_keys;
    for (const auto & frontier : observed_frontiers) {
      if (!satisfies_min_frontier_size(frontier) ||
        is_blacklisted(frontier.centroid_x, frontier.centroid_y) ||
        is_completed(frontier.centroid_x, frontier.centroid_y))
      {
        continue;
      }

      auto & node = upsert_frontier_graph_node(frontier);
      refresh_frontier_metrics(&node.frontier, robot_xy);
      node.in_current_observation = true;
      node.state = FrontierNodeState::kObserved;
      node.preferred_plan_mode = FrontierPlanMode::None;
      observed_keys.push_back(frontier_storage_key(
          node.frontier.centroid_x, node.frontier.centroid_y));
    }

    rebuild_frontier_graph_neighbors(observed_keys);
  }

  std::vector<Frontier> build_observed_frontier_candidates(
    const std::pair<double, double> & robot_xy) const
  {
    std::vector<Frontier> candidates;
    for (const auto & [key, node] : frontier_graph_) {
      (void)key;
      if (!node.in_current_observation) {
        continue;
      }
      if (
        node.state == FrontierNodeState::kCompleted ||
        node.state == FrontierNodeState::kBlacklisted)
      {
        continue;
      }

      Frontier frontier = node.frontier;
      refresh_frontier_metrics(&frontier, robot_xy);
      candidates.push_back(frontier);
    }

    std::sort(
      candidates.begin(), candidates.end(),
      [](const Frontier & lhs, const Frontier & rhs) {
        if (lhs.heuristic_distance != rhs.heuristic_distance) {
          return lhs.heuristic_distance < rhs.heuristic_distance;
        }
        if (lhs.min_distance != rhs.min_distance) {
          return lhs.min_distance < rhs.min_distance;
        }
        return lhs.size > rhs.size;
      });
    return candidates;
  }

  std::vector<Frontier> build_cached_frontier_candidates(
    const std::vector<Frontier> & observed_frontiers,
    const std::pair<double, double> & robot_xy) const
  {
    (void)observed_frontiers;
    std::vector<Frontier> candidates;
    for (const auto & [key, node] : frontier_graph_) {
      (void)key;
      if (!satisfies_min_frontier_size(node.frontier)) {
        continue;
      }
      if (node.in_current_observation) {
        continue;
      }
      if (
        node.state == FrontierNodeState::kCompleted ||
        node.state == FrontierNodeState::kBlacklisted)
      {
        continue;
      }

      Frontier cached_frontier = node.frontier;
      refresh_frontier_metrics(&cached_frontier, robot_xy);
      candidates.push_back(cached_frontier);
    }

    std::sort(
      candidates.begin(), candidates.end(),
      [](const Frontier & lhs, const Frontier & rhs) {
        if (lhs.heuristic_distance != rhs.heuristic_distance) {
          return lhs.heuristic_distance < rhs.heuristic_distance;
        }
        if (lhs.min_distance != rhs.min_distance) {
          return lhs.min_distance < rhs.min_distance;
        }
        return lhs.size > rhs.size;
      });
    return candidates;
  }

  bool maybe_advance_past_active_gvd_goal(const std::pair<double, double> & robot_xy)
  {
    if (
      !navigating_ || pending_replan_after_recovery_ ||
      !has_passed_active_gvd_goal(robot_xy))
    {
      return false;
    }

    const auto goal = active_waypoints_.back();
    RCLCPP_INFO(
      get_logger(),
      "Robot has already passed the active GVD segment goal (%.2f, %.2f) while staying close to it; cancelling this segment and replanning from the current pose",
      goal.first, goal.second);

    ++goal_seq_;
    cancel_current_goal();
    navigating_ = false;
    current_goal_.reset();
    prev_goal_.reset();
    nav_to_goal_handle_.reset();
    nav_through_goal_handle_.reset();
    clear_active_navigation_state(false);
    return true;
  }

  const auto_explore_sim::GvdData & get_gvd_data()
  {
    static const auto_explore_sim::GvdData empty_gvd;
    if (!map_msg_) {
      return empty_gvd;
    }

    const auto grid = make_grid_view();
    if (!grid.valid()) {
      return empty_gvd;
    }

    if (
      !gvd_data_ || !map_content_hash_valid_ || !cached_map_hash_valid_ ||
      cached_map_hash_ != map_content_hash_)
    {
      auto built = auto_explore_sim::GvdMap::build(grid, gvd_config_);
      RCLCPP_INFO(
        get_logger(), "GVD rebuilt: %zu boundary sites, %d Voronoi cells",
        built.boundary_site_count, built.gvd_cell_count);
      gvd_data_ = std::move(built);
      cached_map_hash_ = map_content_hash_;
      cached_map_hash_valid_ = map_content_hash_valid_;
      cached_map_stamp_ = map_msg_->header.stamp;
      cached_gvd_marker_valid_ = false;
    }

    return *gvd_data_;
  }

  std::optional<FrontierNavigationPlan> build_frontier_navigation_plan(
    const std::pair<double, double> & robot_xy,
    const std::pair<double, double> & frontier_xy,
    bool force_reach_anchor = false)
  {
    const auto grid = make_grid_view();
    const auto & gvd_data = get_gvd_data();
    if (!grid.valid() || gvd_data.dist_map.empty()) {
      return std::nullopt;
    }

    FrontierNavigationPlan plan;
    plan.frontier = frontier_xy;

    const auto snapped = auto_explore_sim::GvdMap::snap_to_same_component_gvd(
      grid, gvd_data, robot_xy, frontier_xy, gvd_config_.snap_radius);
    plan.has_gvd_anchor = snapped.found;
    plan.anchor = snapped.point;
    plan.goal = plan.anchor;
    plan.frontier_snapped = snapped.moved;

    if (!plan.has_gvd_anchor) {
      return plan;
    }

    const auto gvd_path = auto_explore_sim::GvdMap::find_path(
      grid, gvd_data, robot_xy, plan.anchor, gvd_config_);
    if (gvd_path && gvd_path->path.size() >= 2U) {
      plan.mode = FrontierPlanMode::GvdWaypoints;
      plan.using_gvd_path = true;
      plan.used_bridge = gvd_path->used_bridge;
      plan.used_heuristic_chain = gvd_path->used_heuristic_chain;
      plan.bridge_trigger_explored_cells = gvd_path->bridge_trigger_explored_cells;
      plan.skeleton_cell_count = gvd_path->skeleton_cell_count;
      plan.path_cell_count = gvd_path->path.size();
      const auto full_waypoints = sample_waypoints(gvd_path->path, gvd_waypoint_spacing_);
      plan.full_waypoint_count = full_waypoints.size();
      plan.full_path_length = polyline_length(full_waypoints);
      plan.max_waypoint_gap = max_waypoint_gap(full_waypoints);
      plan.path_quality_good = is_gvd_path_quality_good(plan);
      if (force_reach_anchor) {
        plan.waypoints = full_waypoints;
        plan.segment_reaches_anchor = true;
        plan.segment_length_limit = plan.full_path_length;
      } else if (plan.path_quality_good) {
        plan.waypoints = full_waypoints;
        plan.segment_reaches_anchor = true;
        plan.segment_length_limit = plan.full_path_length;
      } else {
        plan.segment_length_limit = select_gvd_segment_length(plan);
        plan.waypoints = truncate_waypoint_segment(
          full_waypoints, plan.segment_length_limit, &plan.segment_reaches_anchor);
      }
      if (!plan.waypoints.empty()) {
        plan.goal = plan.waypoints.back();
      }
      return plan;
    }

    return plan;
  }

  std::optional<FrontierNavigationPlan> build_direct_frontier_navigation_plan(
    const std::pair<double, double> & robot_xy,
    const std::pair<double, double> & frontier_xy)
  {
    if (!allow_safe_direct_frontier_nav_) {
      return std::nullopt;
    }

    const auto grid = make_grid_view();
    const auto & gvd_data = get_gvd_data();
    if (!grid.valid() || gvd_data.dist_map.empty()) {
      return std::nullopt;
    }

    const auto direct_path = auto_explore_sim::GvdMap::find_safe_direct_goal(
      grid, gvd_data, robot_xy, frontier_xy, gvd_config_, kDirectFrontierGoalSearchRadius);
    if (!direct_path ||
      direct_path->min_clearance_cells < direct_frontier_min_clearance_cells_)
    {
      return std::nullopt;
    }

    FrontierNavigationPlan plan;
    plan.frontier = frontier_xy;
    plan.anchor = frontier_xy;
    plan.goal = direct_path->goal;
    plan.mode = FrontierPlanMode::DirectGoal;
    plan.direct_goal_is_frontier = true;
    plan.direct_goal_adjusted = direct_path->adjusted_goal;
    plan.direct_path_min_clearance_cells = direct_path->min_clearance_cells;
    plan.direct_path_explored_cells = direct_path->explored_cells;
    plan.segment_reaches_anchor = true;
    return plan;
  }

  FrontierPlanEvaluation evaluate_frontier_plan_modes(
    const Frontier & frontier,
    const std::pair<double, double> & robot_xy,
    bool force_reach_anchor = false)
  {
    FrontierPlanEvaluation evaluation;
    const auto frontier_xy = std::make_pair(frontier.centroid_x, frontier.centroid_y);
    const auto gvd_plan = build_frontier_navigation_plan(
      robot_xy, frontier_xy, force_reach_anchor);
    const auto direct_plan = build_direct_frontier_navigation_plan(robot_xy, frontier_xy);
    evaluation.has_direct_fallback = direct_plan.has_value();

    if (!gvd_plan) {
      evaluation.node_state = direct_plan ?
        FrontierNodeState::kDirectFallbackReady : FrontierNodeState::kDeferred;
      evaluation.reason = "planning inputs were not ready for this frontier yet";
      evaluation.plan = direct_plan;
      return evaluation;
    }

    evaluation.has_gvd_anchor = gvd_plan->has_gvd_anchor;
    if (!gvd_plan->has_gvd_anchor) {
      evaluation.node_state = direct_plan ?
        FrontierNodeState::kDirectFallbackReady : FrontierNodeState::kDeferred;
      evaluation.reason = "no usable GVD anchor within snap radius";
      evaluation.plan = direct_plan;
      return evaluation;
    }

    evaluation.has_same_component_gvd_path =
      gvd_plan->using_gvd_path && gvd_plan->waypoints.size() >= 2U;
    if (!gvd_plan->using_gvd_path || gvd_plan->waypoints.size() < 2U) {
      evaluation.node_state = direct_plan ?
        FrontierNodeState::kDirectFallbackReady : FrontierNodeState::kDeferred;
      evaluation.reason = "no same-component GVD route to the snapped anchor";
      evaluation.plan = direct_plan;
      return evaluation;
    }

    const double anchored_goal_distance = std::hypot(
      gvd_plan->goal.first - robot_xy.first,
      gvd_plan->goal.second - robot_xy.second);
    if (anchored_goal_distance < minimum_dispatch_goal_distance()) {
      if (goal_reaches_frontier(gvd_plan->goal, frontier_xy)) {
        evaluation.node_state = FrontierNodeState::kCompleted;
        evaluation.should_complete_frontier = true;
        evaluation.reason = "anchored GVD goal already reaches the frontier";
        return evaluation;
      }

      evaluation.node_state = direct_plan ?
        FrontierNodeState::kDirectFallbackReady : FrontierNodeState::kDeferred;
      evaluation.reason =
        "current-component GVD anchor goal is already near the robot but still far from the frontier";
      evaluation.plan = direct_plan;
      return evaluation;
    }

    if (
      !goal_reaches_frontier(gvd_plan->goal, frontier_xy) &&
      plan_reuses_stale_intermediate_anchor(*gvd_plan))
    {
      evaluation.node_state = direct_plan ?
        FrontierNodeState::kDirectFallbackReady : FrontierNodeState::kDeferred;
      evaluation.reason =
        "the replanned GVD route reuses an already-reached intermediate anchor cluster";
      evaluation.plan = direct_plan;
      return evaluation;
    }

    evaluation.plan = gvd_plan;
    evaluation.node_state = force_reach_anchor ?
      FrontierNodeState::kLocked : FrontierNodeState::kObserved;
    evaluation.reason = "selected same-component GVD route";
    return evaluation;
  }

  void log_frontier_navigation_plan(
    const FrontierNavigationPlan & plan,
    bool refreshing_existing_goal) const
  {
    const char * prefix = refreshing_existing_goal ? "Refreshing navigation" : "Planning";
    if (plan.mode == FrontierPlanMode::DirectGoal) {
      RCLCPP_INFO(
        get_logger(),
        "%s: no usable GVD route remained, falling back to direct frontier navigation -> "
        "(%.2f, %.2f), path min clearance %.2f cells",
        prefix,
        plan.goal.first, plan.goal.second,
        plan.direct_path_min_clearance_cells);
      return;
    }
    if (!plan.has_gvd_anchor) {
      RCLCPP_INFO(
        get_logger(),
        "%s: no usable GVD anchor found near frontier (%.2f, %.2f)",
        prefix, plan.frontier.first, plan.frontier.second);
      return;
    }
    if (plan.frontier_snapped) {
      RCLCPP_INFO(
        get_logger(),
        "%s: frontier anchor moved from (%.2f, %.2f) to nearest GVD point (%.2f, %.2f)",
        prefix,
        plan.frontier.first, plan.frontier.second,
        plan.anchor.first, plan.anchor.second);
    }

    if (plan.using_gvd_path) {
      if (plan.used_bridge) {
        RCLCPP_INFO(
          get_logger(),
          "GVD path: skeleton disconnected after exploring %d cells, retrying with free-space bridge",
          plan.bridge_trigger_explored_cells);
      }
      if (plan.used_heuristic_chain) {
        RCLCPP_INFO(
          get_logger(),
          "GVD path: skeleton disconnected, using a heuristic-aligned chain of nearby GVD points");
      }
      const int path_pct = static_cast<int>(
        100 * plan.skeleton_cell_count / std::max<size_t>(1U, plan.path_cell_count));
      RCLCPP_INFO(
        get_logger(), "GVD path: %zu cells, %d on skeleton (%d%%)",
        plan.path_cell_count, plan.skeleton_cell_count, path_pct);
      RCLCPP_INFO(
        get_logger(),
        "GVD path found: %zu cells -> %zu full waypoints, dispatching %zu-waypoint %ssegment (len_limit=%.2fm, max_gap=%.2fm)",
        plan.path_cell_count,
        plan.full_waypoint_count,
        plan.waypoints.size(),
        plan.segment_reaches_anchor ? "final " : "",
        plan.segment_length_limit,
        plan.max_waypoint_gap);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "No continuous same-component GVD path to the frontier anchor (%.2f, %.2f)",
      plan.anchor.first, plan.anchor.second);
  }

  void clear_active_navigation_state(bool clear_frontier_target = true)
  {
    active_navigation_mode_ = ActiveNavigationMode::None;
    if (clear_frontier_target) {
      current_frontier_.reset();
      clear_reached_intermediate_anchor();
    }
    current_frontier_anchor_.reset();
    current_frontier_missing_from_search_ = false;
    current_segment_reaches_anchor_ = false;
    pending_replan_after_recovery_ = false;
    active_waypoints_.clear();
    active_gvd_plan_quality_good_ = false;
    last_feedback_goal_distance_.reset();
    last_feedback_xy_.reset();
    gvd_low_progress_feedback_count_ = 0;
    force_gvd_replan_ = false;
    active_plan_map_hash_ = 0;
    active_plan_map_hash_valid_ = false;
    active_plan_robot_xy_.reset();
    last_gvd_replan_time_.reset();
  }

  void dispatch_frontier_navigation_plan(
    const FrontierNavigationPlan & plan,
    bool refreshing_existing_goal)
  {
    current_frontier_ = plan.frontier;
    current_frontier_anchor_ = plan.has_gvd_anchor ? std::optional<std::pair<double, double>>(plan.anchor) :
      std::optional<std::pair<double, double>>(plan.goal);
    current_segment_reaches_anchor_ = plan.segment_reaches_anchor;
    active_plan_map_hash_ = map_content_hash_;
    active_plan_map_hash_valid_ = map_content_hash_valid_;
    active_plan_robot_xy_ = get_robot_position();
    last_gvd_replan_time_ = now();

    if (refreshing_existing_goal) {
      cancel_current_goal();
    }

    if (plan.mode == FrontierPlanMode::GvdWaypoints) {
      if (!plan.using_gvd_path || plan.waypoints.size() < 2U) {
        RCLCPP_WARN(
          get_logger(),
          "Refusing to dispatch a GVD waypoint plan without a continuous GVD-guided path");
        return;
      }

      active_navigation_mode_ = ActiveNavigationMode::GvdWaypoints;
      active_waypoints_ = plan.waypoints;
      active_gvd_plan_quality_good_ = plan.path_quality_good;
      publish_path_markers(plan.waypoints);
      navigate_through_poses(plan.waypoints);
      return;
    }

    if (plan.mode == FrontierPlanMode::DirectGoal) {
      active_navigation_mode_ = ActiveNavigationMode::SingleGoal;
      active_waypoints_.clear();
      active_gvd_plan_quality_good_ = false;
      publish_path_markers({plan.goal});
      navigate_to(plan.goal.first, plan.goal.second);
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "Refusing to dispatch a frontier plan with an unsupported mode");
  }

  void maybe_refresh_navigation(const std::pair<double, double> & robot_xy)
  {
    if (
      !replan_while_navigating_ || !navigating_ ||
      active_navigation_mode_ != ActiveNavigationMode::GvdWaypoints ||
      !current_frontier_ || !map_msg_)
    {
      return;
    }
    const bool forced_shorter_segment = force_gvd_replan_;
    if (!forced_shorter_segment && last_gvd_replan_time_ &&
      (now() - *last_gvd_replan_time_).seconds() < gvd_replan_interval_)
    {
      return;
    }
    const bool map_updated =
      active_plan_map_hash_valid_ && map_content_hash_valid_ &&
      active_plan_map_hash_ != map_content_hash_;
    const bool moved_beyond_snap_radius =
      active_plan_robot_xy_ &&
      std::hypot(
        robot_xy.first - active_plan_robot_xy_->first,
        robot_xy.second - active_plan_robot_xy_->second) >= gvd_config_.snap_radius;
    if (!map_updated && !moved_beyond_snap_radius && !forced_shorter_segment) {
      return;
    }
    if (current_goal_) {
      const double remaining = std::hypot(
        current_goal_->first - robot_xy.first,
        current_goal_->second - robot_xy.second);
      if (!forced_shorter_segment && remaining < gvd_replan_min_remaining_distance_) {
        return;
      }
    }

    last_gvd_replan_time_ = now();

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "%s%s%s - keeping the current GVD segment and deferring replanning until this segment finishes",
      map_updated ? "Map updated" : "",
      moved_beyond_snap_radius ?
      (map_updated ? ", robot moved beyond gvd_snap_radius" : "Robot moved beyond gvd_snap_radius") :
      "",
      forced_shorter_segment ?
      ((map_updated || moved_beyond_snap_radius) ? ", adaptive segment shortening requested" :
      "Adaptive segment shortening requested") :
      "");

    active_plan_map_hash_ = map_content_hash_;
    active_plan_map_hash_valid_ = map_content_hash_valid_;
    active_plan_robot_xy_ = robot_xy;
    force_gvd_replan_ = false;
  }

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

    const std::uint64_t new_map_hash = compute_map_content_hash(*msg);
    const bool map_content_changed = !map_content_hash_valid_ || new_map_hash != map_content_hash_;
    map_content_hash_ = new_map_hash;
    map_content_hash_valid_ = true;
    if (map_content_changed) {
      clear_map_caches();
    } else {
      cached_map_stamp_ = msg->header.stamp;
      if (cached_gvd_marker_valid_) {
        cached_gvd_marker_stamp_ = msg->header.stamp;
      }
    }
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

    const std::pair<double, double> feedback_xy{
      current_pose.pose.position.x,
      current_pose.pose.position.y};
    if (
      active_navigation_mode_ == ActiveNavigationMode::GvdWaypoints &&
      maybe_advance_past_active_gvd_goal(feedback_xy))
    {
      record_callback_timing("nav_feedback_cb", started);
      return;
    }

    if (active_navigation_mode_ == ActiveNavigationMode::GvdWaypoints && current_goal_) {
      const double goal_distance = std::hypot(
        current_goal_->first - feedback_xy.first,
        current_goal_->second - feedback_xy.second);
      if (last_feedback_xy_ && last_feedback_goal_distance_) {
        const double moved = std::hypot(
          feedback_xy.first - last_feedback_xy_->first,
          feedback_xy.second - last_feedback_xy_->second);
        const double progress = *last_feedback_goal_distance_ - goal_distance;
        if (moved >= 0.05 && progress < gvd_jerk_progress_epsilon_) {
          ++gvd_low_progress_feedback_count_;
        } else if (progress > gvd_jerk_progress_epsilon_) {
          gvd_low_progress_feedback_count_ = 0;
        }
        if (gvd_low_progress_feedback_count_ >= gvd_jerk_feedback_threshold_) {
          maybe_force_shorter_gvd_replan(feedback_xy);
        }
      }
      last_feedback_xy_ = feedback_xy;
      last_feedback_goal_distance_ = goal_distance;
    } else {
      last_feedback_goal_distance_.reset();
      last_feedback_xy_.reset();
      gvd_low_progress_feedback_count_ = 0;
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

    if (!current_frontier_ && waiting_for_usable_frontier_on_current_map_) {
      if (
        usable_frontier_wait_map_hash_valid_ && map_content_hash_valid_ &&
        usable_frontier_wait_map_hash_ == map_content_hash_)
      {
        return;
      }
      clear_waiting_for_usable_frontier_on_current_map();
    }

    if (!initial_pose_) {
      initial_pose_ = robot_xy;
      RCLCPP_INFO(
        get_logger(), "Initial pose stored: (%.2f, %.2f)",
        initial_pose_->first, initial_pose_->second);
    }

    check_progress(*robot_xy);
    if (navigating_) {
      maybe_refresh_navigation(*robot_xy);
      return;
    }

    const auto plan_started = Clock::now();
    const auto grid = make_grid_view();
    auto stage_started = Clock::now();
    const auto & gvd_data = get_gvd_data();
    record_timing("gvd_build", stage_started);
    if (!grid.valid() || gvd_data.dist_map.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Map cache not ready for frontier planning");
      return;
    }

    auto search_started = Clock::now();
    auto frontiers = auto_explore_sim::FrontierSearch::search(
      grid, *robot_xy, gvd_data.dist_map, frontier_search_config_);
    bool expanded_frontier_search = false;
    if (frontiers.empty() && !current_frontier_) {
      auto global_frontier_config = frontier_search_config_;
      global_frontier_config.frontier_update_radius = 0.0;
      frontiers = auto_explore_sim::FrontierSearch::search(
        grid, *robot_xy, gvd_data.dist_map, global_frontier_config);
      expanded_frontier_search = !frontiers.empty();
    }
    record_timing("frontier_search", search_started);

    if (frontiers.empty() && !current_frontier_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 10000,
        "No frontiers found - exploration may be complete!");
      if (return_to_init_ && initial_pose_) {
        return_to_initial_pose();
      }
      return;
    }
    if (expanded_frontier_search) {
      RCLCPP_INFO(
        get_logger(),
        "No nearby frontier remained within %.2fm; expanding search to the full explored map",
        frontier_search_config_.frontier_update_radius);
    }

    prune_blacklist();
    refresh_known_frontiers(frontiers, *robot_xy);
    const auto filter_frontiers =
      [&](const std::vector<Frontier> & candidates) {
        std::vector<Frontier> filtered;
        filtered.reserve(candidates.size());
        for (const auto & frontier : candidates) {
          if (
            satisfies_min_frontier_size(frontier) &&
            !is_blacklisted(frontier.centroid_x, frontier.centroid_y) &&
            !is_completed(frontier.centroid_x, frontier.centroid_y))
          {
            filtered.push_back(frontier);
          }
        }
        return filtered;
      };
    std::vector<Frontier> observed_valid = filter_frontiers(
      build_observed_frontier_candidates(*robot_xy));
    std::vector<Frontier> cached_valid = filter_frontiers(
      build_cached_frontier_candidates(frontiers, *robot_xy));
    if (observed_valid.empty() && cached_valid.empty() && !current_frontier_) {
      if (!blacklisted_.empty()) {
        if (!waiting_for_new_map_after_blacklist_reset_) {
          waiting_for_new_map_after_blacklist_reset_ = true;
          blacklist_reset_wait_map_hash_ = map_content_hash_;
          blacklist_reset_wait_map_hash_valid_ = map_content_hash_valid_;
          RCLCPP_INFO(
            get_logger(),
            "All current frontiers are blacklisted on the current map; waiting for a new map update before retrying them");
          return;
        }
        if (
          blacklist_reset_wait_map_hash_valid_ && map_content_hash_valid_ &&
          blacklist_reset_wait_map_hash_ == map_content_hash_)
        {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "All frontiers on the current map are blacklisted; waiting for a new map update before retrying");
          return;
        }

        RCLCPP_INFO(
          get_logger(),
          "Map updated after all frontiers were blacklisted; clearing the blacklist and retrying frontier selection");
        waiting_for_new_map_after_blacklist_reset_ = false;
        blacklist_reset_wait_map_hash_ = 0;
        blacklist_reset_wait_map_hash_valid_ = false;
        blacklisted_.clear();
        frontier_plan_failures_.clear();
        frontier_execution_failures_.clear();
        frontier_plan_failure_hashes_.clear();
        observed_valid = filter_frontiers(build_observed_frontier_candidates(*robot_xy));
        cached_valid = filter_frontiers(build_cached_frontier_candidates(frontiers, *robot_xy));
      }
      if (observed_valid.empty() && cached_valid.empty()) {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "All frontiers are currently blacklisted - waiting for blacklist expiry");
        return;
      }
    }

    auto gvd_started = Clock::now();
    std::optional<Frontier> selected_frontier;
    std::optional<FrontierNavigationPlan> selected_plan;
    bool waiting_on_locked_frontier = false;
    std::unordered_set<std::string> attempted_frontier_keys;
    const auto make_locked_frontier =
      [&](const std::optional<Frontier> & observed_frontier) -> Frontier {
        Frontier locked_frontier;
        locked_frontier.centroid_x = current_frontier_->first;
        locked_frontier.centroid_y = current_frontier_->second;
        locked_frontier.heuristic_distance = std::hypot(
          current_frontier_->first - robot_xy->first,
          current_frontier_->second - robot_xy->second);
        locked_frontier.min_distance = locked_frontier.heuristic_distance;
        locked_frontier.cost = locked_frontier.heuristic_distance;
        if (observed_frontier) {
          locked_frontier.size = observed_frontier->size;
          locked_frontier.min_distance = observed_frontier->min_distance;
        }
        return locked_frontier;
      };
    const auto record_frontier_graph_evaluation =
      [&](const Frontier & frontier, const FrontierPlanEvaluation & evaluation,
        std::optional<FrontierNodeState> override_state = std::nullopt)
      {
        const FrontierNodeState state = override_state.value_or(evaluation.node_state);
        const FrontierPlanMode preferred_mode = evaluation.plan ?
          evaluation.plan->mode : FrontierPlanMode::None;
        set_frontier_graph_node_state(
          frontier.centroid_x, frontier.centroid_y, state, preferred_mode,
          evaluation.reason, evaluation.has_gvd_anchor,
          evaluation.has_same_component_gvd_path, evaluation.has_direct_fallback);
      };

    if (current_frontier_ &&
      (is_blacklisted(current_frontier_->first, current_frontier_->second) ||
      is_completed(current_frontier_->first, current_frontier_->second)))
    {
      current_frontier_.reset();
      current_frontier_missing_from_search_ = false;
    }

    auto try_select_frontier = [&](const Frontier & frontier) -> bool {
        const std::string key = frontier_key(frontier.centroid_x, frontier.centroid_y);
        if (!attempted_frontier_keys.insert(key).second) {
          return false;
        }
        const auto evaluation = evaluate_frontier_plan_modes(frontier, *robot_xy);
        record_frontier_graph_evaluation(frontier, evaluation);
        if (evaluation.should_complete_frontier) {
          RCLCPP_INFO(
            get_logger(),
            "Skipping frontier (%.2f, %.2f): %s",
            frontier.centroid_x, frontier.centroid_y, evaluation.reason.c_str());
          complete_frontier(frontier.centroid_x, frontier.centroid_y);
          return false;
        }
        if (!evaluation.plan) {
          RCLCPP_INFO(
            get_logger(),
            "Skipping frontier (%.2f, %.2f): %s; keeping it for a future mixed-plan update",
            frontier.centroid_x, frontier.centroid_y, evaluation.reason.c_str());
          return false;
        }
        clear_frontier_plan_failures(frontier.centroid_x, frontier.centroid_y);
        selected_frontier = frontier;
        selected_plan = evaluation.plan;
        return true;
      };

    if (current_frontier_) {
      const auto matched_frontier = match_locked_frontier(observed_valid);
      const Frontier locked_frontier = make_locked_frontier(matched_frontier);
      current_frontier_missing_from_search_ = !matched_frontier.has_value();
      if (matched_frontier) {
        RCLCPP_INFO(
          get_logger(),
          "Keeping locked frontier target at (%.2f, %.2f); matched nearby frontier at "
          "(%.2f, %.2f)",
          locked_frontier.centroid_x, locked_frontier.centroid_y,
          matched_frontier->centroid_x, matched_frontier->centroid_y);
      } else {
        RCLCPP_INFO(
          get_logger(),
          "Locked frontier moved out of the current frontier set; continuing toward stored target (%.2f, %.2f)",
          locked_frontier.centroid_x, locked_frontier.centroid_y);
      }

      waiting_on_locked_frontier = true;
      const auto locked_evaluation = evaluate_frontier_plan_modes(
        locked_frontier, *robot_xy, current_frontier_missing_from_search_);
      record_frontier_graph_evaluation(
        locked_frontier, locked_evaluation,
        locked_evaluation.plan ? std::optional<FrontierNodeState>(FrontierNodeState::kLocked) :
        std::nullopt);
      if (locked_evaluation.should_complete_frontier) {
        RCLCPP_INFO(
          get_logger(),
          "Locked frontier reached completion condition: %s",
          locked_evaluation.reason.c_str());
        complete_frontier(locked_frontier.centroid_x, locked_frontier.centroid_y);
        clear_active_navigation_state();
        waiting_on_locked_frontier = false;
      } else if (!locked_evaluation.plan) {
        const auto failure = note_frontier_plan_failure(
          current_frontier_->first, current_frontier_->second);
        if (failure.advanced) {
          RCLCPP_INFO(
            get_logger(),
            "Locked frontier cannot produce a usable mixed plan yet: %s (%d/%d)",
            locked_evaluation.reason.c_str(), failure.count, frontier_plan_retry_limit_);
        } else {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Locked frontier still cannot produce a usable mixed plan and the map "
            "content is unchanged: %s (%d/%d)",
            locked_evaluation.reason.c_str(), failure.count, frontier_plan_retry_limit_);
        }
        record_timing("gvd_path_search", gvd_started);
        if (failure.advanced && failure.count >= frontier_plan_retry_limit_) {
          RCLCPP_INFO(
            get_logger(),
            "Locked frontier exceeded the retry limit without a usable mixed plan; blacklisting it");
          blacklist_current_target();
          clear_active_navigation_state();
          waiting_on_locked_frontier = false;
        } else {
          return;
        }
      } else {
        clear_frontier_plan_failures(current_frontier_->first, current_frontier_->second);
        if (
          current_frontier_missing_from_search_ &&
          locked_evaluation.plan->mode == FrontierPlanMode::GvdWaypoints)
        {
          RCLCPP_INFO(
            get_logger(),
            "Locked frontier is no longer in the current frontier set; keeping the full remaining GVD route to its anchor");
        }
        if (locked_evaluation.plan->mode == FrontierPlanMode::DirectGoal) {
          RCLCPP_INFO(
            get_logger(),
            "Locked frontier currently prefers direct fallback inside the mixed planning framework");
        }
        selected_frontier = locked_frontier;
        selected_plan = locked_evaluation.plan;
      }
    }

    if (!selected_frontier || !selected_plan) {
      for (const auto & frontier : observed_valid) {
        if (try_select_frontier(frontier)) {
          break;
        }
      }
    }

    if (!selected_frontier || !selected_plan) {
      for (const auto & frontier : cached_valid) {
        if (try_select_frontier(frontier)) {
          break;
        }
      }
    }

    if (
      !selected_frontier && !selected_plan && !waiting_on_locked_frontier &&
      !current_frontier_ && frontier_search_config_.frontier_update_radius > 0.0 &&
      !expanded_frontier_search)
    {
      auto global_frontier_config = frontier_search_config_;
      global_frontier_config.frontier_update_radius = 0.0;
      const auto global_frontiers = auto_explore_sim::FrontierSearch::search(
        grid, *robot_xy, gvd_data.dist_map, global_frontier_config);
      refresh_known_frontiers(global_frontiers, *robot_xy);
      const auto global_observed_valid = filter_frontiers(
        build_observed_frontier_candidates(*robot_xy));
      const auto global_cached_valid = filter_frontiers(
        build_cached_frontier_candidates(global_frontiers, *robot_xy));
      if (!global_observed_valid.empty() || !global_cached_valid.empty()) {
        expanded_frontier_search = true;
        RCLCPP_INFO(
          get_logger(),
          "No usable nearby frontier remained after GVD anchoring; expanding search to the full explored map");
        for (const auto & frontier : global_observed_valid) {
          if (try_select_frontier(frontier)) {
            break;
          }
        }
        if (!selected_frontier || !selected_plan) {
          for (const auto & frontier : global_cached_valid) {
            if (try_select_frontier(frontier)) {
              break;
            }
          }
        }
      }
    }

    record_timing("gvd_path_search", gvd_started);
    if (!selected_frontier || !selected_plan) {
      if (waiting_on_locked_frontier) {
        return;
      }
      waiting_for_usable_frontier_on_current_map_ = true;
      usable_frontier_wait_map_hash_ = map_content_hash_;
      usable_frontier_wait_map_hash_valid_ = map_content_hash_valid_;
      RCLCPP_INFO(
        get_logger(),
        "No usable frontier remains after evaluating the current frontier list and then the cached frontier list on this map; waiting for a real map change before retrying");
      return;
    }

    clear_waiting_for_usable_frontier_on_current_map();

    const Frontier & best = *selected_frontier;
    current_frontier_ = std::make_pair(best.centroid_x, best.centroid_y);
    if (!waiting_on_locked_frontier) {
      current_frontier_missing_from_search_ = false;
    }
    RCLCPP_INFO(
      get_logger(),
      "Best frontier: (%.2f, %.2f) heuristic_dist=%.2fm min_dist=%.2fm size=%d",
      best.centroid_x, best.centroid_y,
      best.heuristic_distance, best.min_distance, best.size);

    if (visualize_) {
      auto marker_started = Clock::now();
      auto marker_frontiers = observed_valid;
      if (marker_frontiers.empty()) {
        marker_frontiers = cached_valid;
      }
      const bool best_in_markers = std::any_of(
        marker_frontiers.begin(), marker_frontiers.end(),
        [&](const Frontier & frontier) {
          return frontiers_match(frontier, best);
        });
      if (!best_in_markers) {
        marker_frontiers.push_back(best);
      }
      publish_frontier_markers(marker_frontiers, best);
      record_timing("frontier_markers", marker_started);
    }
    if (visualize_gvd_) {
      auto marker_started = Clock::now();
      publish_gvd_markers();
      record_timing("gvd_markers", marker_started);
    }

    auto plan = std::move(*selected_plan);
    log_frontier_navigation_plan(plan, false);

    const double dist_to_goal = std::hypot(
      plan.goal.first - robot_xy->first,
      plan.goal.second - robot_xy->second);
    if (dist_to_goal < minimum_dispatch_goal_distance()) {
      RCLCPP_WARN(
        get_logger(), "Goal (%.2f, %.2f) only %.2fm away - blacklisting frontier",
        plan.goal.first, plan.goal.second, dist_to_goal);
      blacklist_point(best.centroid_x, best.centroid_y);
      return;
    }

    if (prev_goal_) {
      const double dx = plan.goal.first - prev_goal_->first;
      const double dy = plan.goal.second - prev_goal_->second;
      if (std::sqrt(dx * dx + dy * dy) < 0.01) {
        RCLCPP_DEBUG(get_logger(), "Same goal as before - skipping");
        return;
      }
    }

    auto send_started = Clock::now();
    auto path_marker_started = Clock::now();
    dispatch_frontier_navigation_plan(plan, false);
    record_timing("path_markers", path_marker_started);
    record_timing("send_goal", send_started);
    record_timing("plan_total", plan_started);
    ++timing_plan_counter_;
    maybe_log_timing();
    return;
  }

  std::vector<std::pair<double, double>> sample_waypoints(
    const std::vector<std::pair<double, double>> & path,
    double spacing) const
  {
    if (path.empty()) {
      return {};
    }
    if (path.size() <= 2U || spacing <= 0.0) {
      return path;
    }

    std::vector<std::pair<double, double>> sampled;
    sampled.reserve(path.size());
    sampled.push_back(path.front());
    double accumulated = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
      const auto & prev = path[i - 1];
      const auto & cur = path[i];
      const double dx = cur.first - prev.first;
      const double dy = cur.second - prev.second;
      const double seg_len = std::hypot(dx, dy);
      if (seg_len <= 1e-6) {
        continue;
      }

      accumulated += seg_len;
      if (accumulated + 1e-6 < spacing) {
        continue;
      }

      if (std::hypot(
          sampled.back().first - cur.first,
          sampled.back().second - cur.second) > 1e-4)
      {
        sampled.push_back(cur);
      }
      accumulated = 0.0;
    }

    if (std::hypot(
        sampled.back().first - path.back().first,
        sampled.back().second - path.back().second) > 1e-4)
    {
      sampled.push_back(path.back());
    }

    return sampled;
  }

  std::vector<std::pair<double, double>> truncate_waypoint_segment(
    const std::vector<std::pair<double, double>> & waypoints,
    double max_length,
    bool * reaches_anchor) const
  {
    if (reaches_anchor) {
      *reaches_anchor = true;
    }
    if (waypoints.size() <= 2U || max_length <= 0.0) {
      return waypoints;
    }

    std::vector<std::pair<double, double>> segment;
    segment.reserve(waypoints.size());
    segment.push_back(waypoints.front());
    double accumulated = 0.0;

    for (size_t i = 1; i < waypoints.size(); ++i) {
      const auto & prev = waypoints[i - 1];
      const auto & cur = waypoints[i];
      const double dx = cur.first - prev.first;
      const double dy = cur.second - prev.second;
      const double seg_len = std::hypot(dx, dy);
      if (seg_len <= 1e-6) {
        continue;
      }

      if (accumulated + seg_len <= max_length + 1e-6) {
        segment.push_back(cur);
        accumulated += seg_len;
        continue;
      }

      if (segment.size() == 1U) {
        segment.push_back(cur);
      }
      if (reaches_anchor) {
        *reaches_anchor = false;
      }
      break;
    }

    if (segment.size() == 1U && waypoints.size() >= 2U) {
      segment.push_back(waypoints[1]);
      if (reaches_anchor) {
        *reaches_anchor = waypoints.size() <= 2U;
      }
    }

    const auto & last_segment = segment.back();
    const auto & last_waypoint = waypoints.back();
    if (reaches_anchor &&
      std::hypot(
        last_segment.first - last_waypoint.first,
        last_segment.second - last_waypoint.second) > 1e-4)
    {
      *reaches_anchor = false;
    }

    return segment;
  }

  std::optional<Frontier> match_locked_frontier(const std::vector<Frontier> & valid) const
  {
    if (!current_frontier_) {
      return std::nullopt;
    }

    const Frontier * best = nullptr;
    double best_dist = std::numeric_limits<double>::infinity();
    for (const auto & frontier : valid) {
      const double dist = std::hypot(
        frontier.centroid_x - current_frontier_->first,
        frontier.centroid_y - current_frontier_->second);
      if (dist < best_dist && dist <= frontier_match_radius()) {
        best = &frontier;
        best_dist = dist;
      }
    }

    if (!best) {
      return std::nullopt;
    }
    return *best;
  }

  std::optional<std::pair<double, double>> find_recovery_point_on_active_gvd_path(
    const std::pair<double, double> & robot_xy,
    double min_distance_m) const
  {
    if (active_waypoints_.size() < 2U) {
      return std::nullopt;
    }

    std::optional<std::pair<double, double>> best_point;
    double best_dist = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i + 1 < active_waypoints_.size(); ++i) {
      const auto & candidate = active_waypoints_[i];
      const double dist = std::hypot(
        candidate.first - robot_xy.first,
        candidate.second - robot_xy.second);
      if (dist < min_distance_m || dist > stuck_recovery_gvd_radius_) {
        continue;
      }
      if (dist < best_dist) {
        best_dist = dist;
        best_point = candidate;
      }
    }

    return best_point;
  }

  std::optional<std::pair<double, double>> find_nearest_recovery_gvd_point(
    const std::pair<double, double> & robot_xy,
    double min_distance_m) const
  {
    if (!gvd_data_) {
      return std::nullopt;
    }

    const auto grid = make_grid_view();
    const auto & gvd = *gvd_data_;
    if (!grid.valid() || gvd.empty()) {
      return std::nullopt;
    }

    std::optional<std::pair<double, double>> best_point;
    double best_dist = std::numeric_limits<double>::infinity();
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const int idx = grid.index(x, y);
        if (!gvd.mask[static_cast<size_t>(idx)]) {
          continue;
        }

        const auto candidate = gvd.world_point(grid, x, y);
        const double dist = std::hypot(
          candidate.first - robot_xy.first,
          candidate.second - robot_xy.second);
        if (dist < min_distance_m || dist > stuck_recovery_gvd_radius_) {
          continue;
        }
        if (dist < best_dist) {
          best_dist = dist;
          best_point = candidate;
        }
      }
    }

    return best_point;
  }

  bool trigger_stuck_recovery(const std::pair<double, double> & robot_xy, double elapsed_s)
  {
    constexpr double kRecoveryMinDistance = 0.15;
    const auto path_recovery_target =
      find_recovery_point_on_active_gvd_path(robot_xy, kRecoveryMinDistance);
    const auto recovery_target = path_recovery_target ?
      path_recovery_target :
      find_nearest_recovery_gvd_point(robot_xy, kRecoveryMinDistance);
    if (!recovery_target) {
      RCLCPP_WARN(
        get_logger(),
        "No progress for %.0fs and no nearby GVD recovery point was found within %.2fm",
        elapsed_s, stuck_recovery_gvd_radius_);
      return false;
    }

    const double yaw = std::atan2(
      recovery_target->second - robot_xy.second,
      recovery_target->first - robot_xy.first);
    const double distance = std::hypot(
      recovery_target->first - robot_xy.first,
      recovery_target->second - robot_xy.second);

    RCLCPP_WARN(
      get_logger(),
      "No progress for %.0fs - recovering toward %s GVD point (%.2f, %.2f), distance %.2fm",
      elapsed_s,
      path_recovery_target ? "active-path" : "nearest",
      recovery_target->first, recovery_target->second, distance);

    shrink_adaptive_gvd_segment_length(
      "Detected stuck motion while following the current GVD segment");
    cancel_current_goal();
    active_waypoints_.clear();
    active_navigation_mode_ = ActiveNavigationMode::SingleGoal;
    pending_replan_after_recovery_ = true;
    navigate_to(recovery_target->first, recovery_target->second, yaw);
    return true;
  }

  void navigate_to(double x, double y, std::optional<double> yaw = std::nullopt)
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
    goal.pose.pose.orientation = yaw ? yaw_to_quaternion(*yaw) : geometry_msgs::msg::Quaternion{};
    if (!yaw) {
      goal.pose.pose.orientation.w = 1.0;
    }

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
    options.feedback_callback = [this](
      GoalHandleNavTo::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback)
      {
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
    last_feedback_goal_distance_.reset();
    last_feedback_xy_.reset();
    gvd_low_progress_feedback_count_ = 0;
  }

  void navigate_through_poses(const std::vector<std::pair<double, double>> & waypoints)
  {
    if (!nav_through_client_->wait_for_action_server(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_logger(), "NavigateThroughPoses action server not available!");
      return;
    }

    NavigateThroughPoses::Goal goal;
    const auto stamp = now();
    for (size_t i = 0; i < waypoints.size(); ++i) {
      const auto & [wx, wy] = waypoints[i];
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
    options.feedback_callback = [this](
      GoalHandleNavThrough::SharedPtr,
      const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
      {
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
    last_feedback_goal_distance_.reset();
    last_feedback_xy_.reset();
    gvd_low_progress_feedback_count_ = 0;
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
      clear_active_navigation_state();
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
      if (pending_replan_after_recovery_) {
        RCLCPP_INFO(
          get_logger(),
          "GVD recovery succeeded; replanning from the nearest GVD point toward the locked frontier");
        navigating_ = false;
        nav_to_goal_handle_.reset();
        nav_through_goal_handle_.reset();
        prev_goal_.reset();
        clear_active_navigation_state(false);
        record_callback_timing("navigation_result_cb", started);
        return;
      }

      if (
        current_frontier_ &&
        (!current_segment_reaches_anchor_ ||
        (current_goal_ && !goal_reaches_frontier(*current_goal_, *current_frontier_))))
      {
        remember_reached_intermediate_anchor();
        clear_frontier_execution_failures(current_frontier_->first, current_frontier_->second);
        if (!active_gvd_plan_quality_good_) {
          grow_adaptive_gvd_segment_length();
        }
        RCLCPP_INFO(
          get_logger(),
          "Intermediate frontier anchor reached; recomputing the next step toward the locked frontier");
        navigating_ = false;
        nav_to_goal_handle_.reset();
        nav_through_goal_handle_.reset();
        prev_goal_.reset();
        clear_active_navigation_state(false);
        record_callback_timing("navigation_result_cb", started);
        return;
      }

      RCLCPP_INFO(get_logger(), "Navigation succeeded");
      if (current_frontier_) {
        clear_reached_intermediate_anchor();
        clear_frontier_execution_failures(current_frontier_->first, current_frontier_->second);
        complete_frontier(current_frontier_->first, current_frontier_->second);
      }
    } else if (code == rclcpp_action::ResultCode::ABORTED) {
      if (active_navigation_mode_ == ActiveNavigationMode::GvdWaypoints) {
        shrink_adaptive_gvd_segment_length(
          "Nav2 aborted while following the current GVD segment");
      }
      if (current_frontier_) {
        const int failure_count = note_frontier_execution_failure(
          current_frontier_->first, current_frontier_->second);
        if (failure_count < frontier_plan_retry_limit_) {
          RCLCPP_WARN(
            get_logger(),
            "Navigation aborted by Nav2, but the frontier had a valid dispatched plan; keeping the same frontier and replanning (%d/%d)",
            failure_count, frontier_plan_retry_limit_);
          navigating_ = false;
          nav_to_goal_handle_.reset();
          nav_through_goal_handle_.reset();
          prev_goal_.reset();
          clear_active_navigation_state(false);
          record_callback_timing("navigation_result_cb", started);
          return;
        }
      }
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
    clear_active_navigation_state();
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
      if (
        active_navigation_mode_ == ActiveNavigationMode::GvdWaypoints &&
        !pending_replan_after_recovery_ &&
        trigger_stuck_recovery(robot_xy, elapsed))
      {
        return;
      }

      if (current_frontier_) {
        const int failure_count = note_frontier_execution_failure(
          current_frontier_->first, current_frontier_->second);
        RCLCPP_WARN(
          get_logger(),
          "No progress for %.0fs while executing a valid frontier plan", elapsed);
        cancel_current_goal();
        if (failure_count < frontier_plan_retry_limit_) {
          RCLCPP_WARN(
            get_logger(),
            "Keeping the locked frontier and replanning instead of blacklisting it immediately (%d/%d)",
            failure_count, frontier_plan_retry_limit_);
          navigating_ = false;
          clear_active_navigation_state(false);
          return;
        }

        RCLCPP_WARN(
          get_logger(),
          "Execution retry limit reached for the locked frontier; blacklisting it (%d/%d)",
          failure_count, frontier_plan_retry_limit_);
        blacklist_current_target();
        navigating_ = false;
        clear_active_navigation_state();
        return;
      }

      RCLCPP_WARN(get_logger(), "No progress for %.0fs - cancelling goal", elapsed);
      cancel_current_goal();
      blacklist_current_goal();
      navigating_ = false;
      clear_active_navigation_state();
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
    set_frontier_graph_node_state(
      x, y, FrontierNodeState::kBlacklisted, FrontierPlanMode::None,
      "frontier blacklisted", false, false, false);
    if (auto * node = find_frontier_graph_node(x, y)) {
      node->in_current_observation = false;
    }
    if (last_reached_intermediate_frontier_ &&
      points_are_near(*last_reached_intermediate_frontier_, {x, y}, frontier_match_radius()))
    {
      clear_reached_intermediate_anchor();
    }
    clear_frontier_plan_failures(x, y);
    clear_frontier_execution_failures(x, y);
  }

  void complete_frontier(double x, double y)
  {
    if (is_completed(x, y)) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Completing frontier (%.2f, %.2f)", x, y);
    completed_frontiers_.push_back({x, y});
    set_frontier_graph_node_state(
      x, y, FrontierNodeState::kCompleted, FrontierPlanMode::None,
      "frontier completed", false, false, false);
    if (auto * node = find_frontier_graph_node(x, y)) {
      node->in_current_observation = false;
    }
    if (last_reached_intermediate_frontier_ &&
      points_are_near(*last_reached_intermediate_frontier_, {x, y}, frontier_match_radius()))
    {
      clear_reached_intermediate_anchor();
    }
    clear_frontier_plan_failures(x, y);
    clear_frontier_execution_failures(x, y);
  }

  struct FrontierFailureNote
  {
    int count{0};
    bool advanced{false};
  };

  FrontierFailureNote note_frontier_plan_failure(double x, double y)
  {
    const std::string key = frontier_key(x, y);
    int & count = frontier_plan_failures_[key];
    if (map_content_hash_valid_) {
      const auto previous = frontier_plan_failure_hashes_.find(key);
      if (previous != frontier_plan_failure_hashes_.end() &&
        previous->second == map_content_hash_)
      {
        return {count, false};
      }
      frontier_plan_failure_hashes_[key] = map_content_hash_;
    }
    ++count;
    return {count, true};
  }

  void clear_frontier_plan_failures(double x, double y)
  {
    const std::string key = frontier_key(x, y);
    frontier_plan_failures_.erase(key);
    frontier_plan_failure_hashes_.erase(key);
  }

  int note_frontier_execution_failure(double x, double y)
  {
    const std::string key = frontier_key(x, y);
    return ++frontier_execution_failures_[key];
  }

  void clear_frontier_execution_failures(double x, double y)
  {
    const std::string key = frontier_key(x, y);
    frontier_execution_failures_.erase(key);
  }

  void handle_frontier_plan_failure(
    double x, double y, const std::string & reason)
  {
    const auto failure = note_frontier_plan_failure(x, y);
    if (!failure.advanced) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Skipping frontier (%.2f, %.2f): %s; same map content as the last failed attempt, waiting for a real map change (%d/%d)",
        x, y, reason.c_str(), failure.count, frontier_plan_retry_limit_);
      return;
    }
    if (failure.count >= frontier_plan_retry_limit_) {
      RCLCPP_INFO(
        get_logger(),
        "Skipping frontier (%.2f, %.2f): %s; retry limit reached (%d/%d), blacklisting it",
        x, y, reason.c_str(), failure.count, frontier_plan_retry_limit_);
      blacklist_point(x, y);
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Skipping frontier (%.2f, %.2f): %s; deferring it for now (%d/%d)",
      x, y, reason.c_str(), failure.count, frontier_plan_retry_limit_);
  }

  static std::string frontier_key(double x, double y)
  {
    const auto ix = static_cast<long long>(std::llround(x * 100.0));
    const auto iy = static_cast<long long>(std::llround(y * 100.0));
    return std::to_string(ix) + ":" + std::to_string(iy);
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

  bool is_completed(double x, double y) const
  {
    for (const auto & item : completed_frontiers_) {
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
    if (blacklisted_.empty()) {
      waiting_for_new_map_after_blacklist_reset_ = false;
      blacklist_reset_wait_map_hash_ = 0;
      blacklist_reset_wait_map_hash_valid_ = false;
    }
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
    if (!gvd_marker_pub_ || !map_msg_) {
      return;
    }

    ++gvd_marker_publish_counter_;
    if (((gvd_marker_publish_counter_ - 1) % gvd_marker_publish_every_) != 0) {
      return;
    }

    const auto grid = make_grid_view();
    const auto & gvd_data = get_gvd_data();
    if (!grid.valid() || gvd_data.empty()) {
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

    constexpr std::array<std::pair<int, int>, 4> edges{{
      std::pair<int, int>{1, 0},
      std::pair<int, int>{0, 1},
      std::pair<int, int>{1, 1},
      std::pair<int, int>{1, -1},
    }};

    int edge_counter = 0;
    for (int y = 0; y < grid.height; ++y) {
      for (int x = 0; x < grid.width; ++x) {
        const int idx = grid.index(x, y);
        if (!gvd_data.mask[static_cast<size_t>(idx)]) {
          continue;
        }
        for (const auto & edge : edges) {
          const int nx = x + edge.first;
          const int ny = y + edge.second;
          if (!grid.in_bounds(nx, ny)) {
            continue;
          }

          const int nidx = grid.index(nx, ny);
          if (!gvd_data.mask[static_cast<size_t>(nidx)]) {
            continue;
          }
          if ((edge_counter++ % gvd_marker_stride_) != 0) {
            continue;
          }

          const auto [wx0, wy0] = gvd_data.world_point(grid, x, y);
          const auto [wx1, wy1] = gvd_data.world_point(grid, nx, ny);
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

  void publish_frontier_markers(const std::vector<Frontier> & frontiers, const Frontier & chosen)
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
    gvd_data_.reset();
    cached_map_hash_ = 0;
    cached_map_hash_valid_ = false;
    cached_map_stamp_ = builtin_interfaces::msg::Time();
    cached_gvd_marker_stamp_ = builtin_interfaces::msg::Time();
    cached_gvd_marker_valid_ = false;
  }

  void store_map_to_odom(
    double tx,
    double ty,
    double yaw,
    const builtin_interfaces::msg::Time & stamp)
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
    std::vector<std::pair<std::string, double>> entries(
      timing_totals_.begin(), timing_totals_.end());
    std::sort(
      entries.begin(), entries.end(),
      [](const auto & a, const auto & b) {return a.second > b.second;});

    RCLCPP_INFO(
      get_logger(), "Frontier timing over %d plans (%.1fs window):",
      timing_plan_counter_, elapsed);
    for (const auto & [stage, total] : entries) {
      const int count = std::max(1, timing_counts_[stage]);
      const double avg_ms = 1000.0 * total / count;
      const double max_ms = 1000.0 * timing_max_[stage];
      const double share = total_time > 0.0 ? 100.0 * total / total_time : 0.0;
      RCLCPP_INFO(
        get_logger(), "  %s: avg=%.1fms  max=%.1fms  share=%.1f%%",
        stage.c_str(), avg_ms, max_ms, share);
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

    std::vector<std::pair<std::string, double>> entries(
      callback_totals_.begin(), callback_totals_.end());
    std::sort(
      entries.begin(), entries.end(),
      [](const auto & a, const auto & b) {return a.second > b.second;});

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
    RCLCPP_INFO(
      get_logger(), "  callback_busy_total: %.1f%%",
      100.0 * total / std::max(window_s, 1e-6));

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

  double planner_freq_{0.5};
  auto_explore_sim::FrontierSearchConfig frontier_search_config_;
  std::string robot_base_frame_;
  std::string odom_frame_;
  std::string odom_topic_;
  std::string global_frame_;
  std::string map_topic_;
  std::string live_map_ready_topic_;
  double tf_tolerance_{2.0};
  double blacklist_radius_{0.5};
  double blacklist_timeout_{60.0};
  int frontier_plan_retry_limit_{3};
  double progress_timeout_{30.0};
  bool visualize_{true};
  bool return_to_init_{false};
  auto_explore_sim::GvdConfig gvd_config_;
  bool allow_safe_direct_frontier_nav_{true};
  double direct_frontier_min_clearance_cells_{6.0};
  bool visualize_gvd_{true};
  int gvd_marker_publish_every_{1};
  int gvd_marker_stride_{1};
  double gvd_waypoint_spacing_{0.18};
  int gvd_waypoint_smoothing_passes_{2};
  double gvd_segment_length_{1.0};
  double gvd_segment_length_min_{0.4};
  double gvd_segment_length_max_{6.0};
  double adaptive_gvd_segment_length_{1.0};
  int gvd_jerk_feedback_threshold_{5};
  double gvd_jerk_progress_epsilon_{0.03};
  double gvd_jerk_replan_cooldown_{2.0};
  double stuck_recovery_gvd_radius_{3.0};
  bool replan_while_navigating_{true};
  double gvd_replan_interval_{3.0};
  double gvd_replan_min_remaining_distance_{0.8};
  double gvd_goal_pass_max_goal_distance_{0.45};
  double gvd_goal_pass_projection_margin_{0.08};
  double gvd_goal_pass_lateral_tolerance_{0.35};
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
  ActiveNavigationMode active_navigation_mode_{ActiveNavigationMode::None};
  std::optional<std::pair<double, double>> current_goal_;
  std::optional<std::pair<double, double>> current_frontier_;
  std::optional<std::pair<double, double>> current_frontier_anchor_;
  std::optional<std::pair<double, double>> last_reached_intermediate_anchor_;
  std::optional<std::pair<double, double>> last_reached_intermediate_frontier_;
  bool current_frontier_missing_from_search_{false};
  bool current_segment_reaches_anchor_{false};
  bool pending_replan_after_recovery_{false};
  bool active_gvd_plan_quality_good_{false};
  bool force_gvd_replan_{false};
  bool waiting_for_new_map_after_blacklist_reset_{false};
  bool waiting_for_usable_frontier_on_current_map_{false};
  bool map_content_hash_valid_{false};
  bool cached_map_hash_valid_{false};
  bool blacklist_reset_wait_map_hash_valid_{false};
  bool active_plan_map_hash_valid_{false};
  bool usable_frontier_wait_map_hash_valid_{false};
  bool last_reached_intermediate_anchor_map_hash_valid_{false};
  std::optional<std::pair<double, double>> prev_goal_;
  std::optional<std::pair<double, double>> last_robot_pos_;
  std::optional<std::pair<double, double>> last_feedback_xy_;
  std::optional<std::pair<double, double>> initial_pose_;
  std::optional<rclcpp::Time> last_progress_time_;
  std::optional<rclcpp::Time> last_gvd_jerk_time_;
  std::optional<rclcpp::Time> last_gvd_replan_time_;
  std::optional<double> last_feedback_goal_distance_;
  std::optional<std::pair<double, double>> active_plan_robot_xy_;
  int gvd_low_progress_feedback_count_{0};
  std::vector<std::pair<double, double>> active_waypoints_;
  std::vector<BlacklistedPoint> blacklisted_;
  std::vector<CompletedPoint> completed_frontiers_;
  std::unordered_map<std::string, FrontierGraphNode> frontier_graph_;
  std::unordered_map<std::string, int> frontier_plan_failures_;
  std::unordered_map<std::string, int> frontier_execution_failures_;
  std::unordered_map<std::string, std::uint64_t> frontier_plan_failure_hashes_;

  GoalHandleNavTo::SharedPtr nav_to_goal_handle_;
  GoalHandleNavThrough::SharedPtr nav_through_goal_handle_;

  std::optional<auto_explore_sim::GvdData> gvd_data_;
  std::uint64_t map_content_hash_{0};
  std::uint64_t cached_map_hash_{0};
  std::uint64_t blacklist_reset_wait_map_hash_{0};
  std::uint64_t active_plan_map_hash_{0};
  std::uint64_t usable_frontier_wait_map_hash_{0};
  std::uint64_t last_reached_intermediate_anchor_map_hash_{0};
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FrontierExplorerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
