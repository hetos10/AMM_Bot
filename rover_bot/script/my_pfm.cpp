// apf_navigation_node.cpp
//
// Function-based, single-file implementation of the navigation stack:
//   Stage 1 (once per goal):     thetaStarSearch()      -- global planning
//   Stage 2 (every control cycle, in order):
//       computeAttractiveForce()                        -- Eq. 2
//       computeZoneClusters()                           -- Eq. 3
//       computeBoundedRepulsion()                       -- Eq. 4-5
//       computeTieBreakBias()                           -- tie-break persistence
//       MovingAverageFilter::update()                   -- smoothing
//       purePursuitCommand()                             -- Eq. 9-10, steering
//       applyGoalDeceleration() / applyKinematicLimits()  -- Eq. 11 + safety
//
// Each stage above is a free function taking plain data in and returning
// plain data out -- the ApfNavigationNode class only wires ROS I/O
// (subscriptions, the timer, publishing) around them; it does not
// contain the algorithm logic itself. This keeps every stage independently
// readable and testable outside of ROS (see the project report for the
// equation numbers referenced in each comment).
//
// Topics:
//   subscribes: /map (nav_msgs/OccupancyGrid, transient_local)
//               /initialpose (geometry_msgs/PoseWithCovarianceStamped,
//                             transient_local + reliable) -- resets
//                             stage-2 state and caches the operator/AMCL
//                             supplied start pose
//               /goal_pose (geometry_msgs/PoseStamped, transient_local +
//                           reliable, so a goal set in RViz before this
//                           node comes up is not silently dropped)
//               /scan (sensor_msgs/LaserScan)
//               /odom (nav_msgs/Odometry)
//   publishes:  /cmd_vel (geometry_msgs/Twist)
//               /global_path_markers, /apf_debug (visualization_msgs/MarkerArray)

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2/utils.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// =====================================================================
// Plain data types (no ROS dependency -- usable and testable standalone)
// =====================================================================
struct Vec2
{
  double x{0.0};
  double y{0.0};
  double norm() const {return std::sqrt(x * x + y * y);}
  Vec2 operator+(const Vec2 & o) const {return {x + o.x, y + o.y};}
  Vec2 operator-(const Vec2 & o) const {return {x - o.x, y - o.y};}
  Vec2 operator*(double s) const {return {x * s, y * s};}
  Vec2 operator/(double s) const {return {x / s, y / s};}
  Vec2 & operator+=(const Vec2 & o) {x += o.x; y += o.y; return *this;}
  Vec2 & operator-=(const Vec2 & o) {x -= o.x; y -= o.y; return *this;}
  Vec2 & operator/=(double s) {x /= s; y /= s; return *this;}
};

struct Cell
{
  int x{0};
  int y{0};
  bool operator==(const Cell & o) const {return x == o.x && y == o.y;}
};

struct CellHash
{
  std::size_t operator()(const Cell & c) const
  {
    return (static_cast<std::size_t>(static_cast<uint32_t>(c.x)) << 32) ^
           static_cast<uint32_t>(c.y);
  }
};

enum class Mode {NOMINAL, AVOID};
enum class Side {LEFT, RIGHT};

// Result of zone decomposition + clustering (Eq. 3): for each of the
// three zones, the nearest obstacle distance/position, if any.
struct ZoneScan
{
  double min_dist[3] = {
    std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity()};  // [FRONT, LEFT, RIGHT]
  Vec2 nearest[3];
  bool has_obstacle[3] = {false, false, false};
};

// All tunables in one place, hardcoded here (values match the former
// params.yaml exactly -- no ROS parameter server / launch YAML involved)
// and passed by const-reference into the free functions below.
struct NavParams
{
  // global stage
  double robot_radius_m{0.25};
  double inflation_margin_m{0.10};
  double collinear_tolerance_deg{8.0};
  // attractive force (Eq. 2)
  double k_attractive{1.0};
  double attractive_switch_radius_m{1.0};
  double waypoint_arrival_radius_m{0.3};
  // zone decomposition (Eq. 3)
  double front_half_angle_deg{30.0};
  double cluster_merge_distance_m{0.20};
  int min_cluster_points{3};
  // bounded repulsion (Eq. 4-5)
  double k_repulsive{1.0};
  double repulsive_influence_radius_m{1.2};
  double repulsive_force_max{3.0};
  // tie-break persistence
  double tiebreak_tolerance{0.05};
  double tiebreak_bias_magnitude{0.4};
  // hysteresis (Eq. 6)
  double hysteresis_band_m{0.15};
  // smoothing (moving average)
  int moving_average_window{5};
  double lookahead_min_m{0.3};
  double lookahead_max_m{1.2};
  double lookahead_speed_gain{0.5};
  // goal deceleration (Eq. 11)
  double goal_deceleration_radius_m{0.8};
  // kinematic limits
  double v_max{0.5};
  double w_max{1.5};
  double accel_limit_mps2{0.5};
  double control_rate_hz{20.0};
  // drive kinematics -- "ackermann" (steering-angle output) or
  // "differential" (yaw-rate output). Defaults to differential to match
  // params.yaml / the current robot model; see purePursuitCommand() for
  // the branch.
  std::string drive_type{"differential"};
  double wheelbase_m{0.6};             // must match model.sdf's <wheel_base>
  double max_steering_angle_rad{0.6458};  // must match model.sdf's <steering_limit>
};

namespace
{
double clampd(double v, double lo, double hi)
{
#if defined(__cpp_lib_clamp) && __cpp_lib_clamp >= 201603L
  // C++20 (and some C++17 stdlibs that back-ported it): use the standard,
  // vetted implementation directly.
  return std::clamp(v, lo, hi);
#else
  // Fallback for toolchains without <algorithm>'s std::clamp -- identical
  // semantics (lo <= hi assumed, same as std::clamp's precondition).
  return std::max(lo, std::min(hi, v));
#endif
}
}

// =====================================================================
// STAGE 1 -- Global planning: Theta* (Nash, Daniel, Koenig & Felner,
// AAAI 2007) any-angle search over the static map.
// =====================================================================
class OccupancyGridView
{
public:
  OccupancyGridView(
    const std::vector<int8_t> & data, int width, int height,
    double resolution, double origin_x, double origin_y, int8_t occupied_thresh = 50)
  : data_(data), width_(width), height_(height), resolution_(resolution),
    origin_x_(origin_x), origin_y_(origin_y), occupied_thresh_(occupied_thresh)
  {}

  bool inBounds(int x, int y) const {return x >= 0 && y >= 0 && x < width_ && y < height_;}

  // Unknown cells (-1) are treated as occupied -- conservative choice,
  // see the report's environmental-assumptions limitation.
  bool isFree(int x, int y) const
  {
    if (!inBounds(x, y)) {return false;}
    int8_t v = data_[static_cast<std::size_t>(y) * width_ + x];
    if (v < 0) {return false;}
    return v < occupied_thresh_;
  }

  Cell worldToGrid(const Vec2 & p) const
  {
    return Cell{
      static_cast<int>(std::floor((p.x - origin_x_) / resolution_)),
      static_cast<int>(std::floor((p.y - origin_y_) / resolution_))};
  }

  Vec2 gridToWorld(const Cell & c) const
  {
    return Vec2{origin_x_ + (c.x + 0.5) * resolution_, origin_y_ + (c.y + 0.5) * resolution_};
  }

  int width() const {return width_;}
  int height() const {return height_;}
  double resolution() const {return resolution_;}

private:
  const std::vector<int8_t> & data_;
  int width_, height_;
  double resolution_, origin_x_, origin_y_;
  int8_t occupied_thresh_;
};

std::vector<int8_t> inflateObstacles(
  const OccupancyGridView & grid, const std::vector<int8_t> & raw, double inflate_radius_m)
{
  std::vector<int8_t> out = raw;
  int cells = static_cast<int>(std::ceil(inflate_radius_m / grid.resolution()));
  if (cells <= 0) {return out;}
  for (int y = 0; y < grid.height(); ++y) {
    for (int x = 0; x < grid.width(); ++x) {
      int8_t v = raw[static_cast<std::size_t>(y) * grid.width() + x];
      if (v < 0 || v < 50) {continue;}
      for (int dy = -cells; dy <= cells; ++dy) {
        for (int dx = -cells; dx <= cells; ++dx) {
          if (dx * dx + dy * dy > cells * cells) {continue;}
          int nx = x + dx, ny = y + dy;
          if (!grid.inBounds(nx, ny)) {continue;}
          std::size_t idx = static_cast<std::size_t>(ny) * grid.width() + nx;
          if (out[idx] >= 0) {out[idx] = std::max<int8_t>(out[idx], 100);}
        }
      }
    }
  }
  return out;
}

// Bresenham-based line-of-sight, used by Theta* to link a neighbour
// straight to its grandparent when nothing blocks the direct line.
bool lineOfSight(const OccupancyGridView & grid, Cell a, Cell b)
{
  int x0 = a.x, y0 = a.y, x1 = b.x, y1 = b.y;
  int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  int x = x0, y = y0;
  while (true) {
    if (!grid.isFree(x, y)) {return false;}
    if (x == x1 && y == y1) {break;}
    int e2 = 2 * err;
    if (e2 > -dy) {err -= dy; x += sx;}
    if (e2 < dx) {err += dx; y += sy;}
  }
  return true;
}

std::vector<Cell> thetaStarSearch(const OccupancyGridView & grid, Cell start, Cell goal)
{
  auto heuristic = [&](const Cell & a, const Cell & b) {
      double dx = a.x - b.x, dy = a.y - b.y;
      return std::sqrt(dx * dx + dy * dy);
    };
  if (!grid.isFree(start.x, start.y) || !grid.isFree(goal.x, goal.y)) {return {};}

  struct Open {Cell cell; double f;};
  struct Cmp {bool operator()(const Open & a, const Open & b) const {return a.f > b.f;}};
  std::priority_queue<Open, std::vector<Open>, Cmp> open;
  std::unordered_map<Cell, double, CellHash> g_score;
  std::unordered_map<Cell, Cell, CellHash> parent;
  std::unordered_set<Cell, CellHash> closed;

  g_score[start] = 0.0;
  parent[start] = start;
  open.push({start, heuristic(start, goal)});
  static const int nbr[8][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

  while (!open.empty()) {
    Cell s = open.top().cell;
    open.pop();
    if (s == goal) {
      std::vector<Cell> path{s};
      while (!(s == start)) {s = parent[s]; path.push_back(s);}
      std::reverse(path.begin(), path.end());
      return path;
    }
    if (closed.count(s)) {continue;}
    closed.insert(s);

    Cell par = parent[s];
    for (const auto & n : nbr) {
      Cell s2{s.x + n[0], s.y + n[1]};
      if (!grid.isFree(s2.x, s2.y) || closed.count(s2)) {continue;}

      double new_g;
      Cell new_parent;
      if (lineOfSight(grid, par, s2)) {
        new_g = g_score[par] + heuristic(par, s2);
        new_parent = par;
      } else {
        double step = (n[0] && n[1]) ? std::sqrt(2.0) : 1.0;
        new_g = g_score[s] + step;
        new_parent = s;
      }
      auto it = g_score.find(s2);
      if (it == g_score.end() || new_g < it->second) {
        g_score[s2] = new_g;
        parent[s2] = new_parent;
        open.push({s2, new_g + heuristic(s2, goal)});
      }
    }
  }
  return {};
}

std::vector<Vec2> simplifyPath(
  const OccupancyGridView & grid, const std::vector<Cell> & cell_path, double collinear_tol_deg)
{
  std::vector<Vec2> world;
  world.reserve(cell_path.size());
  for (const auto & c : cell_path) {world.push_back(grid.gridToWorld(c));}
  if (world.size() < 3) {return world;}

  std::vector<Vec2> wps{world.front()};
  auto heading = [](const Vec2 & a, const Vec2 & b) {return std::atan2(b.y - a.y, b.x - a.x);};
  double tol = collinear_tol_deg * M_PI / 180.0;
  Vec2 anchor = world.front();
  double last_h = heading(world[0], world[1]);

  for (std::size_t i = 1; i + 1 < world.size(); ++i) {
    double h = heading(anchor, world[i + 1]);
    double diff = std::fabs(std::atan2(std::sin(h - last_h), std::cos(h - last_h)));
    if (diff > tol) {
      wps.push_back(world[i]);
      anchor = world[i];
      last_h = heading(anchor, world[i + 1]);
    }
  }
  wps.push_back(world.back());
  return wps;
}

// =====================================================================
// STAGE 2 -- Local planning and control, called every control cycle.
// =====================================================================

// Attractive force toward the current target (waypoint or goal). Eq. 2.
Vec2 computeAttractiveForce(const Vec2 & pos, const Vec2 & target, const NavParams & p)
{
  Vec2 to_goal = target - pos;
  double d = to_goal.norm();
  if (d <= p.attractive_switch_radius_m) {
    return to_goal * (2.0 * p.k_attractive);
  }
  if (d < 1e-6) {return {0, 0};}
  return (to_goal / d) * (2.0 * p.k_attractive * p.attractive_switch_radius_m);
}

// Zone decomposition (Eq. 3) via sequential-scan clustering: groups
// consecutive valid LiDAR returns into clusters, discards clusters
// smaller than min_cluster_points, keeps the nearest cluster per zone.
ZoneScan computeZoneClusters(
  const sensor_msgs::msg::LaserScan & scan, const Vec2 & pos, double yaw, const NavParams & p)
{
  ZoneScan result;
  double theta_f = p.front_half_angle_deg * M_PI / 180.0;

  std::vector<Vec2> pts;
  Vec2 prev_pt{0.0, 0.0};
  bool have_prev = false;

  auto flush = [&]() {
      if (static_cast<int>(pts.size()) < p.min_cluster_points) {pts.clear(); return;}
      Vec2 c{0, 0};
      for (const auto & pt : pts) {c += pt;}
      c /= static_cast<double>(pts.size());
      Vec2 diff = c - pos;
      double angle = std::atan2(diff.y, diff.x) - yaw;
      angle = std::atan2(std::sin(angle), std::cos(angle));
      int zi = (std::fabs(angle) <= theta_f) ? 0 : (angle > 0 ? 1 : 2);
      double d = diff.norm();
      if (d < result.min_dist[zi]) {
        result.min_dist[zi] = d;
        result.nearest[zi] = c;
        result.has_obstacle[zi] = true;
      }
      pts.clear();
    };

  for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
    float r = scan.ranges[i];
    if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) {
      if (!pts.empty()) {flush();}
      have_prev = false;
      continue;
    }
    // Laser-frame point rotated by yaw and translated to the robot's
    // world-frame position -- a single Vec2, instead of four loose
    // doubles (px_r/py_r/px/py), so the conversion reads as one step.
    double theta = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
    Vec2 local{r * std::cos(theta), r * std::sin(theta)};
    Vec2 world_pt = pos + Vec2{
      local.x * std::cos(yaw) - local.y * std::sin(yaw),
      local.x * std::sin(yaw) + local.y * std::cos(yaw)};
    if (have_prev && (world_pt - prev_pt).norm() > p.cluster_merge_distance_m) {flush();}
    pts.push_back(world_pt);
    prev_pt = world_pt; have_prev = true;
  }
  if (!pts.empty()) {flush();}
  return result;
}

// Hysteresis on the AVOID/NOMINAL switch (Eq. 6). Mutates `mode` in place.
void updateMode(const ZoneScan & zs, double rho0, double hysteresis_band, Mode & mode)
{
  double closest = std::min({zs.min_dist[0], zs.min_dist[1], zs.min_dist[2]});
  if (closest <= rho0 - hysteresis_band) {
    mode = Mode::AVOID;
  } else if (closest >= rho0 + hysteresis_band) {
    mode = Mode::NOMINAL;
  }
  // else: keep previous mode (inside the hysteresis band)
}

// Bounded repulsive force, summed over zones with an obstacle in range. Eq. 4-5.
Vec2 computeBoundedRepulsion(const ZoneScan & zs, const Vec2 & pos, const NavParams & p)
{
  Vec2 f_rep{0.0, 0.0};
  for (int z = 0; z < 3; ++z) {
    if (!zs.has_obstacle[z] || zs.min_dist[z] > p.repulsive_influence_radius_m ||
      zs.min_dist[z] < 1e-3)
    {
      continue;
    }
    Vec2 away = pos - zs.nearest[z];
    double rho = zs.min_dist[z];
    double mag = p.k_repulsive * (1.0 / rho - 1.0 / p.repulsive_influence_radius_m) / (rho * rho);
    mag = clampd(mag, 0.0, p.repulsive_force_max);  // the GNRON-preventing clamp
    double n = away.norm();
    if (n > 1e-6) {f_rep += (away / n) * mag;}
  }
  return f_rep;
}

// Deterministic lateral bias when LEFT/RIGHT obstacle density is nearly
// symmetric, so the robot commits to one side instead of stalling.
// Mutates `last_side` in place (the persistence).
Vec2 computeTieBreakBias(const ZoneScan & zs, double yaw, const NavParams & p, Side & last_side)
{
  if (!zs.has_obstacle[1] && !zs.has_obstacle[2]) {return {0.0, 0.0};}
  double rho0 = p.repulsive_influence_radius_m;
  double o_left = zs.has_obstacle[1] ? clampd(1.0 - zs.min_dist[1] / rho0, 0.0, 1.0) : 0.0;
  double o_right = zs.has_obstacle[2] ? clampd(1.0 - zs.min_dist[2] / rho0, 0.0, 1.0) : 0.0;

  Side side;
  if (o_left - o_right > p.tiebreak_tolerance) {
    side = Side::RIGHT;
  } else if (o_right - o_left > p.tiebreak_tolerance) {
    side = Side::LEFT;
  } else {
    side = last_side;  // symmetric / ambiguous -> persist previous decision
  }
  last_side = side;
  double sign = (side == Side::LEFT) ? 1.0 : -1.0;
  return Vec2{-std::sin(yaw), std::cos(yaw)} * (p.tiebreak_bias_magnitude * sign);
}

// Windowed Simple Moving Average smoothing filter over the last N force
// vectors -- replaces frame-to-frame sensor jitter with the mean of a
// short recent history before the result reaches the controller.
class MovingAverageFilter
{
public:
  explicit MovingAverageFilter(std::size_t window = 5)
  : window_(window) {}

  Vec2 update(const Vec2 & sample)
  {
    buffer_.push_back(sample);
    while (buffer_.size() > window_) {buffer_.pop_front();}
    Vec2 sum{0.0, 0.0};
    for (const auto & v : buffer_) {sum += v;}
    return sum / static_cast<double>(buffer_.size());
  }

  void setWindow(std::size_t w) {window_ = w;}

  // Drops accumulated history -- used when a fresh /initialpose invalidates
  // any smoothing built up under the previous localization/state.
  void reset() {buffer_.clear();}

private:
  std::deque<Vec2> buffer_;
  std::size_t window_;
};

struct DriveCommand
{
  double v{0.0};
  double w{0.0};       // yaw rate (differential) OR steering angle (ackermann) -- see `mode`
  const char * mode{"differential"};  // for debug logging only, not consumed downstream
};

// Speed-adaptive-lookahead Pure Pursuit curvature control (Eq. 9-10),
// combined with the goal deceleration radius (Eq. 11) and hard
// kinematic limits with acceleration limiting for extra smoothness.
//
// Two output kinematics are supported, selected by p.drive_type:
//   "differential" -- w is a yaw rate, w = kappa * v, clamped to w_max
//                      (with v reduced if that clamp would otherwise be
//                      violated -- a diff-drive robot can always achieve
//                      any yaw rate at any speed by spinning faster on
//                      one side, so the *speed* is what has to give).
//   "ackermann"    -- w is a steering angle, delta = atan(kappa * L) for
//                      wheelbase L (the standard bicycle-model Pure
//                      Pursuit relation), clamped to the vehicle's
//                      physical max_steering_angle_rad. Unlike yaw rate,
//                      a steering angle is a geometric limit independent
//                      of speed, so v is NOT reduced to respect it --
//                      clamping the angle directly is the correct fix,
//                      not a speed cut.
DriveCommand purePursuitCommand(
  const Vec2 & pos, double yaw, const Vec2 & f_smooth, const Vec2 & final_goal,
  double prev_speed, const NavParams & p)
{
  double ld = clampd(p.lookahead_speed_gain * prev_speed, p.lookahead_min_m, p.lookahead_max_m);
  double fn = f_smooth.norm();
  Vec2 dir = (fn > 1e-6) ?
    Vec2{f_smooth.x / fn, f_smooth.y / fn} : Vec2{std::cos(yaw), std::sin(yaw)};
  Vec2 target_point{pos.x + dir.x * ld, pos.y + dir.y * ld};

  double dx = target_point.x - pos.x, dy = target_point.y - pos.y;
  double y_t = -dx * std::sin(yaw) + dy * std::cos(yaw);  // lateral offset, body frame
  double kappa = (2.0 * y_t) / (ld * ld);

  double d_to_final = std::hypot(final_goal.x - pos.x, final_goal.y - pos.y);
  double v_cmd = p.v_max * clampd(d_to_final / p.goal_deceleration_radius_m, 0.0, 1.0);

  double dt = 1.0 / p.control_rate_hz;
  double max_dv = p.accel_limit_mps2 * dt;

  if (p.drive_type == "ackermann") {
    double delta = std::atan(kappa * p.wheelbase_m);
    delta = clampd(delta, -p.max_steering_angle_rad, p.max_steering_angle_rad);
    v_cmd = clampd(v_cmd, prev_speed - max_dv, prev_speed + max_dv);
    return {v_cmd, delta, "ackermann(steering_angle_rad)"};
  }

  // --- differential (yaw-rate) path, unchanged from the original design ---
  double w_cmd = kappa * v_cmd;
  if (std::fabs(w_cmd) > p.w_max && std::fabs(kappa) > 1e-6) {
    v_cmd = p.w_max / std::fabs(kappa);
    w_cmd = std::copysign(p.w_max, w_cmd);
  }
  v_cmd = clampd(v_cmd, prev_speed - max_dv, prev_speed + max_dv);
  w_cmd = clampd(kappa * v_cmd, -p.w_max, p.w_max);  // re-derive from same curvature, final clamp

  return {v_cmd, w_cmd, "differential(yaw_rate_rad_s)"};
}

// =====================================================================
// The node: wires ROS I/O around the free functions above. No algorithm
// logic lives in this class -- only subscriptions, the timer, and calls
// into the functions defined above, in order.
// =====================================================================
class ApfNavigationNode : public rclcpp::Node
{
public:
  ApfNavigationNode()
  : Node("apf_navigation_node")
  {
    // Hardcoded configuration (equivalent to former params.yaml contents).
    // params_ already carries these exact values as its default member
    // initializers -- see the NavParams struct definition above -- so no
    // ROS parameter declare/get calls are needed. Only the two string
    // frame names, which lived outside NavParams, are set explicitly here.
    global_frame_ = "map";
    base_frame_ = "base_footprint";
    RCLCPP_INFO(
      get_logger(), "drive_type='%s' wheelbase_m=%.3f max_steering_angle_rad=%.3f",
      params_.drive_type.c_str(), params_.wheelbase_m, params_.max_steering_angle_rad);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    ma_filter_.setWindow(static_cast<std::size_t>(params_.moving_average_window));

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {latest_map_ = msg;});

    // Transient local + reliable: a goal published by RViz's "2D Nav Goal"
    // tool (or a launch-time goal publisher) before this node finishes
    // starting up is still delivered, instead of being dropped on the
    // floor by a plain volatile/best-effort subscription. NOTE: for this
    // durability to actually take effect the publisher side (RViz's goal
    // panel / your goal-publishing node) must also be configured
    // transient_local, since ROS 2 QoS durability requires the publisher
    // to be at least as persistent as the subscriber for the two ends to
    // match.
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "goal_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {onGoal(msg);});

    // Transient local + reliable, matching /goal_pose above, so a
    // late-arriving "2D Pose Estimate" published from RViz before this
    // node is fully up is still received rather than dropped.
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      [this](geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {onInitialPose(msg);});

    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {latest_scan_ = msg;});
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "diff_drive_controller/odom", 10, [this](nav_msgs::msg::Odometry::SharedPtr msg) {latest_odom_ = msg;});

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("apf_debug", 10);
    waypoint_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "global_path_markers", rclcpp::QoS(1).transient_local());

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / params_.control_rate_hz),
      std::bind(&ApfNavigationNode::controlLoop, this));

    RCLCPP_INFO(
      get_logger(), "apf_navigation_node ready. Waiting for /map, /initialpose, and /goal_pose.");
  }

private:
  // --- STAGE 1: triggered once per goal ---
  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
  {
    if (!latest_map_) {
      RCLCPP_WARN(get_logger(), "Goal received but no map yet -- ignoring.");
      return;
    }
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!lookupRobotPose(robot_pose)) {
      RCLCPP_WARN(get_logger(), "Could not resolve robot pose in map frame -- ignoring goal.");
      return;
    }

    const auto & map = *latest_map_;
    OccupancyGridView raw_grid(
      map.data, static_cast<int>(map.info.width), static_cast<int>(map.info.height),
      map.info.resolution, map.info.origin.position.x, map.info.origin.position.y);

    double inflate_r = params_.robot_radius_m + params_.inflation_margin_m;
    std::vector<int8_t> inflated = inflateObstacles(raw_grid, map.data, inflate_r);
    OccupancyGridView grid(
      inflated, static_cast<int>(map.info.width), static_cast<int>(map.info.height),
      map.info.resolution, map.info.origin.position.x, map.info.origin.position.y);

    Cell start = grid.worldToGrid({robot_pose.pose.position.x, robot_pose.pose.position.y});
    Cell goal = grid.worldToGrid({goal_msg->pose.position.x, goal_msg->pose.position.y});

    auto cell_path = thetaStarSearch(grid, start, goal);
    if (cell_path.empty()) {
      RCLCPP_ERROR(get_logger(), "Theta* found no path from start to goal on the inflated map.");
      return;
    }
    path_ = simplifyPath(grid, cell_path, params_.collinear_tolerance_deg);
    current_wp_idx_ = 0;
    goal_reached_ = false;

    publishWaypointMarkers();
    RCLCPP_INFO(
      get_logger(), "Planned path: %zu grid cells -> %zu waypoints.", cell_path.size(), path_.size());
  }

  // Fired on /initialpose (e.g. RViz's "2D Pose Estimate"). This is a
  // statement that the robot's position in the map is being corrected --
  // possibly discontinuously -- so any in-flight plan or smoothing state
  // built up under the old pose estimate is no longer trustworthy.
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // Cache the operator/localization-supplied start pose.
    initial_pose_cache_.x = msg->pose.pose.position.x;
    initial_pose_cache_.y = msg->pose.pose.position.y;
    initial_yaw_cache_ = tf2::getYaw(msg->pose.pose.orientation);
    has_initial_pose_ = true;

    // Reset all stage-2 state: stop, drop the current path so a stale
    // Theta* plan referenced to the old pose can't be driven, and clear
    // the smoothing filter / tie-break / hysteresis state so the next
    // goal starts from a clean slate.
    path_.clear();
    current_wp_idx_ = 0;
    goal_reached_ = true;
    mode_ = Mode::NOMINAL;
    last_side_ = Side::LEFT;
    prev_speed_ = 0.0;
    ma_filter_.reset();
    publishStop();

    RCLCPP_INFO(
      get_logger(),
      "Received /initialpose -- reset navigation state, cached start pose (%.2f, %.2f, yaw=%.2f rad).",
      initial_pose_cache_.x, initial_pose_cache_.y, initial_yaw_cache_);
  }

  bool lookupRobotPose(geometry_msgs::msg::PoseStamped & out)
  {
    try {
      auto tf = tf_buffer_->lookupTransform(
        global_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.5));
      out.header.frame_id = global_frame_;
      out.header.stamp = now();
      out.pose.position.x = tf.transform.translation.x;
      out.pose.position.y = tf.transform.translation.y;
      out.pose.orientation = tf.transform.rotation;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "TF lookup failed: %s", ex.what());
      return false;
    }
  }

  // --- STAGE 2: every control cycle ---
  void controlLoop()
  {
    if (!latest_odom_ || !latest_scan_ || path_.empty() || goal_reached_) {
      // Debug aid: say exactly *why* nothing is happening, not just that
      // nothing is happening -- the four causes look identical from the
      // outside (robot sits still) but need very different fixes.
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Idle: odom=%s scan=%s path=%s goal_reached=%s",
        latest_odom_ ? "OK" : "MISSING", latest_scan_ ? "OK" : "MISSING",
        path_.empty() ? "EMPTY (no goal planned yet)" : "OK",
        goal_reached_ ? "true" : "false");
      publishStop();
      return;
    }

    Vec2 pos{latest_odom_->pose.pose.position.x, latest_odom_->pose.pose.position.y};
    double yaw = tf2::getYaw(latest_odom_->pose.pose.orientation);

    // waypoint advance
    Vec2 target = path_[current_wp_idx_];
    double d_to_wp = std::hypot(target.x - pos.x, target.y - pos.y);
    bool is_last = (current_wp_idx_ + 1 == path_.size());
    if (d_to_wp < params_.waypoint_arrival_radius_m) {
      if (is_last) {
        goal_reached_ = true;
        RCLCPP_INFO(get_logger(), "Final goal reached.");
        publishStop();
        return;
      }
      ++current_wp_idx_;
      target = path_[current_wp_idx_];
      RCLCPP_INFO(
        get_logger(), "Waypoint %zu/%zu reached, advancing to (%.2f, %.2f).",
        current_wp_idx_, path_.size(), target.x, target.y);
    }

    Vec2 f_att = computeAttractiveForce(pos, target, params_);
    ZoneScan zones = computeZoneClusters(*latest_scan_, pos, yaw, params_);
    updateMode(zones, params_.repulsive_influence_radius_m, params_.hysteresis_band_m, mode_);

    Vec2 f_rep{0.0, 0.0}, f_bias{0.0, 0.0};
    if (mode_ == Mode::AVOID) {
      f_rep = computeBoundedRepulsion(zones, pos, params_);
      f_bias = computeTieBreakBias(zones, yaw, params_, last_side_);
    }
    Vec2 f_total = (mode_ == Mode::AVOID) ? (f_att + f_rep + f_bias) : f_att;
    Vec2 f_smooth = ma_filter_.update(f_total);

    DriveCommand cmd = purePursuitCommand(pos, yaw, f_smooth, path_.back(), prev_speed_, params_);
    prev_speed_ = cmd.v;

    // Throttled (not per-cycle) state dump -- the single most useful line
    // when debugging "why is it doing that": mode, distance to target,
    // and the exact command being sent, all in one place.
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "mode=%s wp=%zu/%zu d_to_wp=%.2fm v=%.2f %s=%.3f pos=(%.2f,%.2f) yaw=%.2f",
      mode_ == Mode::AVOID ? "AVOID" : "NOMINAL", current_wp_idx_ + 1, path_.size(),
      d_to_wp, cmd.v, cmd.mode, cmd.w, pos.x, pos.y, yaw);

    publishCmd(cmd.v, cmd.w);
    publishDebug(pos, f_att, f_rep, f_smooth);
  }

  void publishCmd(double v, double w)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
  }

  void publishStop() {publishCmd(0.0, 0.0); prev_speed_ = 0.0;}

  void publishWaypointMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header.frame_id = global_frame_;
    m.header.stamp = now();
    m.ns = "global_waypoints";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = m.scale.y = m.scale.z = 0.15;
    m.color.r = 0.22f; m.color.g = 0.54f; m.color.b = 0.86f; m.color.a = 1.0f;
    m.pose.orientation.w = 1.0;
    for (const auto & wp : path_) {
      geometry_msgs::msg::Point p; p.x = wp.x; p.y = wp.y; p.z = 0.05;
      m.points.push_back(p);
    }
    arr.markers.push_back(m);
    waypoint_markers_pub_->publish(arr);
  }

  void publishDebug(const Vec2 & pos, const Vec2 & f_att, const Vec2 & f_rep, const Vec2 & f_smooth)
  {
    visualization_msgs::msg::MarkerArray arr;
    auto arrow = [&](int id, double dx, double dy, float r, float g, float b) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = global_frame_;
        m.header.stamp = now();
        m.ns = "apf_forces";
        m.id = id;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.scale.x = 0.05; m.scale.y = 0.1; m.scale.z = 0.1;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0f;
        geometry_msgs::msg::Point p0, p1;
        p0.x = pos.x; p0.y = pos.y; p0.z = 0.1;
        p1.x = pos.x + dx; p1.y = pos.y + dy; p1.z = 0.1;
        m.points = {p0, p1};
        return m;
      };
    arr.markers.push_back(arrow(0, f_att.x * 0.3, f_att.y * 0.3, 0.16f, 0.62f, 0.24f));
    arr.markers.push_back(arrow(1, f_rep.x * 0.3, f_rep.y * 0.3, 0.85f, 0.35f, 0.19f));
    arr.markers.push_back(arrow(2, f_smooth.x * 0.3, f_smooth.y * 0.3, 0.1f, 0.1f, 0.1f));
    debug_pub_->publish(arr);
  }

  // --- state ---
  NavParams params_;
  std::string global_frame_, base_frame_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  std::vector<Vec2> path_;
  std::size_t current_wp_idx_{0};
  bool goal_reached_{false};

  Mode mode_{Mode::NOMINAL};
  Side last_side_{Side::LEFT};
  MovingAverageFilter ma_filter_;
  double prev_speed_{0.0};

  // Cached start pose from the most recent /initialpose message.
  Vec2 initial_pose_cache_{0.0, 0.0};
  double initial_yaw_cache_{0.0};
  bool has_initial_pose_{false};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_markers_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApfNavigationNode>());
  rclcpp::shutdown();
  return 0;
}