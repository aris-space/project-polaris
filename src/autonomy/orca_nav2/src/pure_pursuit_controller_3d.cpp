// MIT License
//
// Copyright (c) 2022 Clyde McQueen
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// Inspired by
// https://navigation.ros.org/plugin_tutorials/docs/writing_new_nav2controller_plugin.html
// Regulated Pure Pursuit logic adapted from:
// https://github.com/ros-navigation/navigation2/tree/main/nav2_regulated_pure_pursuit_controller

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "orca_nav2/param_macro.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "pluginlib/class_loader.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace orca_nav2
{

  constexpr bool sign(const double &v) { return v > 0; }

  class Limiter
  {
    double max_a_{};
    double max_dv_{};
    double k_decelerate_{};

  public:
    Limiter() = default;

    Limiter(const double &max_a, const double &dt, const double &k_decelerate)
        : max_a_{max_a}, max_dv_{max_a * dt}, k_decelerate_{k_decelerate}
    {
      assert(max_a > 0);
      assert(dt > 0);
    }

    // Ramp down velocity as the sub approaches the goal
    // Particularly important for linear.x, as momentum will carry the sub a good distance
    // Less important for linear.z, as drag is higher and buoyancy tends to dominate
    void decelerate(double &v, const double &goal_dist) const
    {
      assert(v * goal_dist >= 0);
      auto soft_v = std::abs(goal_dist) * k_decelerate_;
      auto physical_v = std::sqrt(2 * std::abs(goal_dist) * max_a_);
      auto result_v = std::min({std::abs(v), soft_v, physical_v});
      v = std::copysign(result_v, v);
    }

    // Limit acceleration
    void limit(double &v, const double &prev_v) const
    {
      auto dv = v - prev_v;
      if (dv > max_dv_)
      {
        v = prev_v + max_dv_;
      }
      else if (dv < -max_dv_)
      {
        v = prev_v - max_dv_;
      }
    }
  };

  class PurePursuitController3D : public nav2_core::Controller
  {
    rclcpp::Logger logger_{rclcpp::get_logger("placeholder_will_be_set_in_configure")};
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string base_frame_id_;

    // ── Original parameters ────────────────────────────────────────────────────
    double x_vel_{};
    double x_accel_{};
    double z_vel_{};
    double z_accel_{};
    double yaw_vel_{};
    double yaw_accel_{};
    double lookahead_dist_{};
    double transform_tolerance_{};
    double goal_tolerance_{};
    double tick_rate_{};
    double K_descelerate_{};

    Limiter x_limiter_;
    Limiter z_limiter_;
    Limiter yaw_limiter_;

    rclcpp::Duration transform_tolerance_d_{0, 0};

    nav_msgs::msg::Path plan_;
    geometry_msgs::msg::Twist prev_vel_{};

    // Cross-track velocity damping
    double K_cross_vel_{0.0};

    // Velocity-divergence emergency
    double max_velocity_divergence_rad_{0.0};
    double min_speed_divergence_check_{0.02};

    rclcpp::Clock::SharedPtr clock_;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr arm_cmd_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr mode_cmd_pub_;

    bool publish_tracking_error_{true};
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr cross_track_xy_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr vertical_error_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr yaw_error_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr closest_point_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_map_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr robot_twist_pub_;

    // ── Regulated Pure Pursuit parameters ─────────────────────────────────────
    // Rotate-to-heading: stop translating and rotate in place when heading error
    // exceeds rotate_to_heading_min_angle_ (both for sharp turns and goal alignment).
    bool use_rotate_to_heading_{true};
    double rotate_to_heading_angular_vel_{0.3};   // rad/s for in-place rotation
    double rotate_to_heading_min_angle_{M_PI / 4.0}; // 45 deg trigger

    // Velocity-scaled lookahead: lookahead_dist = speed * lookahead_time, clamped.
    // When disabled, lookahead_dist_ (static) is used.
    bool use_velocity_scaled_lookahead_{false};
    double min_lookahead_dist_{1.0};
    double max_lookahead_dist_{5.0};
    double lookahead_time_{1.5};

    // Fixed curvature lookahead: use a separate, fixed-distance point for curvature
    // estimation. Breaks the feedback loop: slow speed → short velocity-scaled
    // lookahead → artificially high curvature → slower still.
    bool use_fixed_curvature_lookahead_{true};
    double curvature_lookahead_dist_{3.0};

    // Curvature-regulated linear velocity: slow down proportionally as turning
    // radius decreases below regulated_linear_scaling_min_radius_.
    bool use_regulated_linear_vel_scaling_{true};
    double regulated_linear_scaling_min_radius_{0.9};
    double regulated_linear_scaling_min_speed_{0.05};

    // Approach velocity: ramp linear.x down as the remaining path length shrinks.
    double min_approach_linear_velocity_{0.05};
    double approach_velocity_scaling_dist_{1.0};

    // Stateful goal heading: once XY tolerance is reached, stay in
    // "rotate to goal heading" mode even if the robot drifts back out slightly.
    bool stateful_{true};
    bool has_reached_xy_tolerance_{false};

    // Hysteresis for rotate-to-path: engage when heading error > rotate_to_heading_min_angle_,
    // stay engaged until error drops below rotate_to_heading_release_angle_.
    // Prevents the AUV releasing rotation too early and then drifting due to residual error.
    double rotate_to_heading_release_angle_{0.2};  // ~11 deg exit threshold
    bool is_rotating_to_path_{false};
    bool was_rotating_to_path_{false};

    // Conservative braking capability used in the sqrt decel formula inside rotateToHeading.
    // Set this to the real AUV angular deceleration (rad/s²), not the command ramp rate.
    // Lower values start braking earlier → less overshoot. Separate from yaw_accel_ so the
    // command ramp rate can stay fast while the braking assumption stays conservative.
    double rotate_to_heading_decel_{0.02};

    // ── Helpers ───────────────────────────────────────────────────────────────

    bool transform_pose(const geometry_msgs::msg::PoseStamped &in_pose,
                        geometry_msgs::msg::PoseStamped &out_pose,
                        const std::string &target_frame) const
    {
      try
      {
        auto transform = tf_->lookupTransform(
            target_frame, in_pose.header.frame_id,
            in_pose.header.stamp, tf2::durationFromSec(transform_tolerance_));
        tf2::doTransform(in_pose, out_pose, transform);
        return true;
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_DEBUG(logger_, "Transform failed: %s", ex.what());
        return false;
      }
    }

    double get_dist_L2_norm(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2) const
    {
      double dx = p1.x - p2.x;
      double dy = p1.y - p2.y;
      double dz = p1.z - p2.z;
      return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Shortest XY cross-track distance, vertical error, and yaw error between
    // the robot and the closest point on the plan polyline (all in map frame).
    void tracking_error_from_plan(
      const geometry_msgs::msg::PoseStamped &pose_f_map,
      double &cross_track_xy_m,
      double &vertical_error_m,
      double &yaw_error_rad,
      geometry_msgs::msg::Point &closest_map) const
    {
      cross_track_xy_m = 0.0;
      vertical_error_m = 0.0;
      yaw_error_rad = 0.0;
      closest_map.x = 0.0;
      closest_map.y = 0.0;
      closest_map.z = 0.0;
      if (plan_.poses.empty()) {
        return;
      }

      const double rx = pose_f_map.pose.position.x;
      const double ry = pose_f_map.pose.position.y;
      const double rz = pose_f_map.pose.position.z;
      const double robot_yaw = tf2::getYaw(pose_f_map.pose.orientation);

      if (plan_.poses.size() == 1) {
        const auto &p = plan_.poses[0].pose.position;
        closest_map = p;
        const double dx = rx - p.x;
        const double dy = ry - p.y;
        const double path_yaw = tf2::getYaw(plan_.poses[0].pose.orientation);
        const double cp = std::cos(path_yaw) * dy - std::sin(path_yaw) * dx;
        const double s = (cp >= 0) ? 1.0 : -1.0;
        cross_track_xy_m = s * std::sqrt(dx * dx + dy * dy);
        vertical_error_m = rz - p.z;
        yaw_error_rad = angles::shortest_angular_distance(path_yaw, robot_yaw);
        return;
      }

      double best_dist_sq = std::numeric_limits<double>::infinity();
      double best_qx = rx;
      double best_qy = ry;
      double best_qz = rz;
      double best_path_yaw = tf2::getYaw(plan_.poses[0].pose.orientation);
      double best_sign = 1.0;

      for (size_t i = 0; i + 1 < plan_.poses.size(); ++i) {
        const auto &p1 = plan_.poses[i].pose.position;
        const auto &p2 = plan_.poses[i + 1].pose.position;
        const double vx = p2.x - p1.x;
        const double vy = p2.y - p1.y;
        const double wx = rx - p1.x;
        const double wy = ry - p1.y;
        const double vv = vx * vx + vy * vy;
        double t = 0.0;
        if (vv > 1e-12) {
          t = (wx * vx + wy * vy) / vv;
          t = std::clamp(t, 0.0, 1.0);
        }
        const double qx = p1.x + t * vx;
        const double qy = p1.y + t * vy;
        const double qz = p1.z + t * (p2.z - p1.z);
        const double ex = rx - qx;
        const double ey = ry - qy;
        const double dist_sq = ex * ex + ey * ey;
        if (dist_sq < best_dist_sq) {
          best_dist_sq = dist_sq;
          best_qx = qx;
          best_qy = qy;
          best_qz = qz;
          if (vv > 1e-12) {
            best_path_yaw = std::atan2(vy, vx);
            const double cp = vx * wy - vy * wx;
            best_sign = (cp >= 0.0) ? 1.0 : -1.0;
          } else {
            best_path_yaw = tf2::getYaw(plan_.poses[i].pose.orientation);
            const double cp = std::cos(best_path_yaw) * wy - std::sin(best_path_yaw) * wx;
            best_sign = (cp >= 0.0) ? 1.0 : -1.0;
          }
        }
      }

      if (std::isfinite(best_dist_sq)) {
        cross_track_xy_m = best_sign * std::sqrt(best_dist_sq);
        vertical_error_m = rz - best_qz;
        yaw_error_rad = angles::shortest_angular_distance(best_path_yaw, robot_yaw);
        closest_map.x = best_qx;
        closest_map.y = best_qy;
        closest_map.z = best_qz;
      }
    }

    // Return velocity-scaled lookahead distance (or static if disabled).
    double getLookaheadDist(double speed) const
    {
      if (use_velocity_scaled_lookahead_) {
        return std::clamp(std::fabs(speed) * lookahead_time_,
                          min_lookahead_dist_, max_lookahead_dist_);
      }
      return lookahead_dist_;
    }

    // Return the first plan pose beyond lookahead_dist from pose_f_map,
    // or the last pose if the plan runs out.
    geometry_msgs::msg::PoseStamped
    find_goal(const geometry_msgs::msg::PoseStamped &pose_f_map, double lookahead_dist) const
    {
      if (plan_.poses.empty()) {
        return geometry_msgs::msg::PoseStamped();
      }

      auto min_dist = std::numeric_limits<double>::max();
      bool dist_decreasing = true;

      for (const auto &item : plan_.poses) {
        auto item_dist = get_dist_L2_norm(item.pose.position, pose_f_map.pose.position);
        if (dist_decreasing) {
          if (item_dist < min_dist) {
            min_dist = item_dist;
          } else {
            dist_decreasing = false;
          }
        }
        if (!dist_decreasing && item_dist > lookahead_dist) {
          return item;
        }
      }
      return plan_.poses.back();
    }

    // True when the heading error to the curvature carrot exceeds the entry threshold.
    // Uses hysteresis: once engaged, stays engaged until error drops below the smaller
    // release threshold — prevents premature release and the cross-track drift that follows.
    bool shouldRotateToPath(double angle_to_carrot)
    {
      if (!use_rotate_to_heading_) {
        is_rotating_to_path_ = false;
        return false;
      }
      const double abs_angle = std::fabs(angle_to_carrot);
      if (is_rotating_to_path_) {
        // Stay in rotation until heading is tight enough to release safely
        is_rotating_to_path_ = abs_angle > rotate_to_heading_release_angle_;
      } else {
        // Engage when error exceeds the entry threshold
        is_rotating_to_path_ = abs_angle > rotate_to_heading_min_angle_;
      }
      return is_rotating_to_path_;
    }

    // True when the AUV is close enough to the goal that it should rotate to
    // the final plan orientation instead of continuing to track the path.
    // With stateful_ = true, persists once XY tolerance is entered.
    bool shouldRotateToGoalHeading(double xy_dist_to_carrot)
    {
      if (!use_rotate_to_heading_) {
        return false;
      }
      if (stateful_) {
        if (!has_reached_xy_tolerance_ && xy_dist_to_carrot < goal_tolerance_) {
          has_reached_xy_tolerance_ = true;
        }
        return has_reached_xy_tolerance_;
      }
      return xy_dist_to_carrot < goal_tolerance_;
    }

    // Rotate in place towards angle_to_target with sqrt-based deceleration
    // (prevents overshoot) and acceleration limiting.
    // Sets linear_vel = 0 and computes angular_vel.
    void rotateToHeading(double &linear_vel, double &angular_vel,
                         double angle_to_target) const
    {
      linear_vel = 0.0;
      const double s = (angle_to_target > 0.0) ? 1.0 : -1.0;
      angular_vel = s * rotate_to_heading_angular_vel_;

      // Acceleration limit: clamp step-change in angular velocity
      const double dt = 1.0 / tick_rate_;
      const double min_w = prev_vel_.angular.z - yaw_accel_ * dt;
      const double max_w = prev_vel_.angular.z + yaw_accel_ * dt;
      angular_vel = std::clamp(angular_vel, min_w, max_w);

      // Decelerate as target approaches: ω_max = sqrt(2 * α * |θ|)
      // Uses rotate_to_heading_decel_ (real braking capability) not yaw_accel_ (ramp rate).
      const double max_w_to_stop = std::sqrt(2.0 * rotate_to_heading_decel_ * std::fabs(angle_to_target));
      if (std::fabs(angular_vel) > max_w_to_stop) {
        angular_vel = s * max_w_to_stop;
      }
    }

    // Apply regulated velocity constraints:
    //   1. Curvature regulation: scale linear_vel down for tight turns.
    //   2. Approach regulation: ramp linear_vel down as remaining path shortens.
    void applyConstraints(double curvature, double path_remaining, double &linear_vel) const
    {
      if (use_regulated_linear_vel_scaling_) {
        const double radius = (std::fabs(curvature) > 1e-6)
                              ? (1.0 / std::fabs(curvature)) : 1e6;
        const double curv_vel = linear_vel *
                                std::min(1.0, radius / regulated_linear_scaling_min_radius_);
        linear_vel = std::max(curv_vel, regulated_linear_scaling_min_speed_);
      }

      if (path_remaining < approach_velocity_scaling_dist_) {
        const double scale = path_remaining / approach_velocity_scaling_dist_;
        linear_vel = std::max(linear_vel * scale, min_approach_linear_velocity_);
      }
    }

    // Regulated Pure Pursuit path tracking — 3D AUV adaptation.
    // linear.z: independent depth control (unchanged from original).
    // linear.x + angular.z: Regulated PP in the XY plane.
    //   - uses a separate fixed-distance lookahead for curvature to decouple
    //     it from the velocity-scaled main lookahead (no feedback loop).
    //   - rotates in place (stops translation) when heading error is large.
    //   - slows for tight curvature and for the final approach.
    geometry_msgs::msg::Twist
    pure_pursuit_3d(const geometry_msgs::msg::PoseStamped &pose_f_odom, double current_speed)
    {
      // Transform pose odom → map
      geometry_msgs::msg::PoseStamped pose_f_map;
      if (!transform_pose(pose_f_odom, pose_f_map, plan_.header.frame_id)) {
        return geometry_msgs::msg::Twist{};
      }

      // ── Lookahead points ───────────────────────────────────────────────────
      const double lookahead = getLookaheadDist(current_speed);

      // Main carrot: drives linear.x and the goal-proximity check
      auto goal_f_map = find_goal(pose_f_map, lookahead);
      goal_f_map.header.stamp = pose_f_map.header.stamp;

      geometry_msgs::msg::PoseStamped goal_f_base;
      if (!transform_pose(goal_f_map, goal_f_base, base_frame_id_)) {
        return geometry_msgs::msg::Twist{};
      }

      // Curvature carrot: drives curvature computation and rotate-to-path trigger.
      // Fixed distance → independent of speed, no feedback loop.
      geometry_msgs::msg::PoseStamped curv_goal_f_base = goal_f_base;
      if (use_fixed_curvature_lookahead_) {
        auto curv_goal_f_map = find_goal(pose_f_map, curvature_lookahead_dist_);
        curv_goal_f_map.header.stamp = pose_f_map.header.stamp;
        geometry_msgs::msg::PoseStamped tmp;
        if (transform_pose(curv_goal_f_map, tmp, base_frame_id_)) {
          curv_goal_f_base = tmp;
        }
      }

      // ── Distances & curvature ──────────────────────────────────────────────
      const double xy_dist_sq =
        goal_f_base.pose.position.x * goal_f_base.pose.position.x +
        goal_f_base.pose.position.y * goal_f_base.pose.position.y;
      const double xy_dist_L2_norm = std::sqrt(xy_dist_sq);
      const double z_dist = std::fabs(goal_f_base.pose.position.z);

      const double cx = curv_goal_f_base.pose.position.x;
      const double cy = curv_goal_f_base.pose.position.y;
      const double curv_dist_sq = cx * cx + cy * cy;
      const double curvature = (curv_dist_sq > 0.001) ? (2.0 * cy / curv_dist_sq) : 0.0;

      // Angle from robot heading to curvature carrot (used for rotate-to-path)
      const double angle_to_curv = std::atan2(cy, cx);

      // Remaining straight-line distance to end of plan (map frame)
      const auto &last_pos = plan_.poses.back().pose.position;
      const double dx_e = last_pos.x - pose_f_map.pose.position.x;
      const double dy_e = last_pos.y - pose_f_map.pose.position.y;
      const double dz_e = last_pos.z - pose_f_map.pose.position.z;
      const double path_remaining = std::sqrt(dx_e * dx_e + dy_e * dy_e + dz_e * dz_e);

      geometry_msgs::msg::Twist cmd_vel;

      // Depth control (linear.z)
      if (z_dist > goal_tolerance_) {
        cmd_vel.linear.z = goal_f_base.pose.position.z > 0 ? z_vel_ : -z_vel_;
        z_limiter_.decelerate(cmd_vel.linear.z, goal_f_base.pose.position.z);
      }

      //  XY control (linear.x + angular.z) — Regulated PP
      //
      // shouldRotateToGoalHeading is checked BEFORE the xy_dist guard so that
      // with stateful_ = true we can still rotate to the final heading even when
      // the robot is already within goal_tolerance_ in XY.
      {
        double linear_vel = 0.0;
        double angular_vel = 0.0;

        if (shouldRotateToGoalHeading(xy_dist_L2_norm)) {
          // Near or at goal position: stop translating and align to final orientation.
          const double goal_yaw = tf2::getYaw(goal_f_base.pose.orientation);
          rotateToHeading(linear_vel, angular_vel, goal_yaw);

        } else if (xy_dist_L2_norm > goal_tolerance_) {
          if (shouldRotateToPath(angle_to_curv)) {
            // Large heading error ahead: rotate in place first.
            // Uses the curvature carrot (further lookahead) so the AUV begins
            // rotating before it physically reaches the turn point.
            rotateToHeading(linear_vel, angular_vel, angle_to_curv);

          } else {
            // Edge: rotate→track handoff. The rotation's last commanded ω is
            // still in prev_vel_.angular.z; with small yaw_accel_·dt the
            // limiter would carry it into tracking and cause overshoot/sway.
            // Zero the history so the curvature-driven yaw rate ramps from 0.
            if (was_rotating_to_path_) {
              prev_vel_.angular.z = 0.0;
              prev_vel_.linear.x = 0.0;
            }

            // Normal tracking: forward motion + curvature-based yaw rate.
            linear_vel = x_vel_;

            // Regulated velocity: scale down for tight curvature and final approach.
            applyConstraints(curvature, path_remaining, linear_vel);

            // Kinematic deceleration near carrot (Limiter sqrt decel)
            x_limiter_.decelerate(linear_vel, xy_dist_L2_norm);

            angular_vel = linear_vel * curvature;
          }
        }
        // else: within XY tolerance, not yet in stateful goal-heading mode → hold still

        cmd_vel.linear.x = linear_vel;
        cmd_vel.angular.z = angular_vel;
      }

      was_rotating_to_path_ = is_rotating_to_path_;

      return cmd_vel;
    }

  public:
    PurePursuitController3D() = default;
    ~PurePursuitController3D() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &weak_parent,
        std::string name,
        const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
    {
      auto parent = weak_parent.lock();

      logger_ = parent->get_logger();
      tf_ = tf;
      base_frame_id_ = costmap_ros->getBaseFrameID();

      // Original parameters
      PARAMETER(parent, name, x_vel, 0.4)
      PARAMETER(parent, name, x_accel, 0.4)
      PARAMETER(parent, name, z_vel, 0.2)
      PARAMETER(parent, name, z_accel, 0.2)
      PARAMETER(parent, name, yaw_vel, 0.4)
      PARAMETER(parent, name, yaw_accel, 0.4)
      PARAMETER(parent, name, lookahead_dist, 1.0)
      PARAMETER(parent, name, transform_tolerance, 1.0)
      PARAMETER(parent, name, goal_tolerance, 0.1)
      PARAMETER(parent, name, tick_rate, 20.0)
      PARAMETER(parent, name, publish_tracking_error, true)
      PARAMETER(parent, name, K_cross_vel, 0.0)
      PARAMETER(parent, name, max_velocity_divergence_rad, 0.0)
      PARAMETER(parent, name, min_speed_divergence_check, 0.02)
      PARAMETER(parent, name, K_descelerate, 0.2)

      // Regulated Pure Pursuit parameters
      PARAMETER(parent, name, use_rotate_to_heading, true)
      PARAMETER(parent, name, rotate_to_heading_angular_vel, 0.3)
      PARAMETER(parent, name, rotate_to_heading_min_angle, 0.785398)
      PARAMETER(parent, name, rotate_to_heading_release_angle, 0.2)
      PARAMETER(parent, name, rotate_to_heading_decel, 0.02)
      PARAMETER(parent, name, use_velocity_scaled_lookahead, false)
      PARAMETER(parent, name, min_lookahead_dist, 1.0)
      PARAMETER(parent, name, max_lookahead_dist, 5.0)
      PARAMETER(parent, name, lookahead_time, 1.5)
      PARAMETER(parent, name, use_fixed_curvature_lookahead, true)
      PARAMETER(parent, name, curvature_lookahead_dist, 3.0)
      PARAMETER(parent, name, use_regulated_linear_vel_scaling, true)
      PARAMETER(parent, name, regulated_linear_scaling_min_radius, 0.9)
      PARAMETER(parent, name, regulated_linear_scaling_min_speed, 0.05)
      PARAMETER(parent, name, min_approach_linear_velocity, 0.05)
      PARAMETER(parent, name, approach_velocity_scaling_dist, 1.0)
      PARAMETER(parent, name, stateful, true)

      clock_ = parent->get_clock();

      arm_cmd_pub_ = parent->create_publisher<std_msgs::msg::Bool>(
        "/pixhawk/arm_cmd", rclcpp::QoS(1).reliable());
      mode_cmd_pub_ = parent->create_publisher<std_msgs::msg::String>(
        "/pixhawk/mode_cmd", rclcpp::QoS(1).reliable());

      if (publish_tracking_error_) {
        cross_track_xy_pub_ = parent->create_publisher<std_msgs::msg::Float64>(
          "pure_pursuit_cross_track_xy", rclcpp::QoS(10));
        vertical_error_pub_ = parent->create_publisher<std_msgs::msg::Float64>(
          "pure_pursuit_vertical_error", rclcpp::QoS(10));
        yaw_error_pub_ = parent->create_publisher<std_msgs::msg::Float64>(
          "pure_pursuit_yaw_error", rclcpp::QoS(10));
        closest_point_pub_ = parent->create_publisher<geometry_msgs::msg::PointStamped>(
          "pure_pursuit_closest_point_map", rclcpp::QoS(10));
        robot_pose_map_pub_ = parent->create_publisher<geometry_msgs::msg::PoseStamped>(
          "pure_pursuit_robot_pose_map", rclcpp::QoS(10));
        robot_twist_pub_ = parent->create_publisher<geometry_msgs::msg::TwistStamped>(
          "pure_pursuit_robot_twist", rclcpp::QoS(10));
      }

      x_limiter_ = Limiter(x_accel_, 1. / tick_rate_, K_descelerate_);
      z_limiter_ = Limiter(z_accel_, 1. / tick_rate_, K_descelerate_);
      yaw_limiter_ = Limiter(yaw_accel_, 1. / tick_rate_, K_descelerate_);

      transform_tolerance_d_ = rclcpp::Duration::from_seconds(transform_tolerance_);

      RCLCPP_INFO(logger_, "PurePursuitController3D (Regulated) configured");
    }

    void cleanup() override
    {
      cross_track_xy_pub_.reset();
      vertical_error_pub_.reset();
      yaw_error_pub_.reset();
      closest_point_pub_.reset();
      robot_pose_map_pub_.reset();
      robot_twist_pub_.reset();
      arm_cmd_pub_.reset();
      mode_cmd_pub_.reset();
    }

    void activate() override
    {
      if (cross_track_xy_pub_) { cross_track_xy_pub_->on_activate(); }
      if (vertical_error_pub_) { vertical_error_pub_->on_activate(); }
      if (yaw_error_pub_) { yaw_error_pub_->on_activate(); }
      if (closest_point_pub_) { closest_point_pub_->on_activate(); }
      if (robot_pose_map_pub_) { robot_pose_map_pub_->on_activate(); }
      if (robot_twist_pub_) { robot_twist_pub_->on_activate(); }
      if (arm_cmd_pub_) { arm_cmd_pub_->on_activate(); }
      if (mode_cmd_pub_) { mode_cmd_pub_->on_activate(); }
    }

    void deactivate() override
    {
      if (cross_track_xy_pub_) { cross_track_xy_pub_->on_deactivate(); }
      if (vertical_error_pub_) { vertical_error_pub_->on_deactivate(); }
      if (yaw_error_pub_) { yaw_error_pub_->on_deactivate(); }
      if (closest_point_pub_) { closest_point_pub_->on_deactivate(); }
      if (robot_pose_map_pub_) { robot_pose_map_pub_->on_deactivate(); }
      if (robot_twist_pub_) { robot_twist_pub_->on_deactivate(); }
      if (arm_cmd_pub_) { arm_cmd_pub_->on_deactivate(); }
      if (mode_cmd_pub_) { mode_cmd_pub_->on_deactivate(); }
      prev_vel_ = geometry_msgs::msg::Twist{};
      is_rotating_to_path_ = false;
      was_rotating_to_path_ = false;
    }

    // Pose is base_f_odom (3D), Twist comes from /odom but is stripped to 2D
    // by nav2_controller::ControllerServer — use prev_vel_ for linear.z history.
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *) override
    {
      geometry_msgs::msg::TwistStamped cmd_vel;
      cmd_vel.header = pose.header;

      // Compute tracking error (needed for cross-track damping and publishing)
      double cross_xy = 0.0, z_err = 0.0, yaw_err = 0.0;
      geometry_msgs::msg::Point closest_map;
      geometry_msgs::msg::PoseStamped pose_f_map;
      bool have_tracking = false;

      if (!plan_.poses.empty() && transform_pose(pose, pose_f_map, plan_.header.frame_id)) {
        tracking_error_from_plan(pose_f_map, cross_xy, z_err, yaw_err, closest_map);
        have_tracking = true;
      }

      if (publish_tracking_error_ && have_tracking &&
          cross_track_xy_pub_ && vertical_error_pub_ && yaw_error_pub_ &&
          closest_point_pub_ && robot_pose_map_pub_ && robot_twist_pub_)
      {
        std_msgs::msg::Float64 cross_msg;  cross_msg.data = cross_xy;
        cross_track_xy_pub_->publish(cross_msg);
        std_msgs::msg::Float64 z_msg;      z_msg.data = z_err;
        vertical_error_pub_->publish(z_msg);
        std_msgs::msg::Float64 yaw_msg;    yaw_msg.data = yaw_err;
        yaw_error_pub_->publish(yaw_msg);

        geometry_msgs::msg::PointStamped cp;
        cp.header.stamp = pose.header.stamp;
        cp.header.frame_id = plan_.header.frame_id;
        cp.point = closest_map;
        closest_point_pub_->publish(cp);

        pose_f_map.header.stamp = pose.header.stamp;
        robot_pose_map_pub_->publish(pose_f_map);

        geometry_msgs::msg::TwistStamped tw;
        tw.header.stamp = pose.header.stamp;
        tw.header.frame_id = pose.header.frame_id;
        tw.twist = velocity;
        robot_twist_pub_->publish(tw);
      }

      // Regulated Pure Pursuit — pass current forward speed for velocity-scaled lookahead
      cmd_vel.twist = pure_pursuit_3d(pose, velocity.linear.x);

      // Cross-track velocity damping: derivative feedback on lateral error.
      if (have_tracking && K_cross_vel_ > 0.0) {
        const double v_cross = velocity.linear.x * std::sin(yaw_err) +
                               velocity.linear.y * std::cos(yaw_err);
        cmd_vel.twist.angular.z -= K_cross_vel_ * v_cross;
      }

      // Acceleration limiting
      x_limiter_.limit(cmd_vel.twist.linear.x, prev_vel_.linear.x);
      z_limiter_.limit(cmd_vel.twist.linear.z, prev_vel_.linear.z);
      yaw_limiter_.limit(cmd_vel.twist.angular.z, prev_vel_.angular.z);

      // Velocity-divergence emergency safety cutoff
      if (have_tracking && max_velocity_divergence_rad_ > 0.0) {
        const double vel_xy = std::hypot(velocity.linear.x, velocity.linear.y);
        if (vel_xy > min_speed_divergence_check_) {
          const double v_along = velocity.linear.x * std::cos(yaw_err) -
                                 velocity.linear.y * std::sin(yaw_err);
          const double v_perp  = velocity.linear.x * std::sin(yaw_err) +
                                 velocity.linear.y * std::cos(yaw_err);
          const double diverge_angle = std::atan2(std::abs(v_perp), v_along);
          if (diverge_angle > max_velocity_divergence_rad_) {
            RCLCPP_ERROR(logger_,
              "EMERGENCY: velocity %.1f deg off path (limit %.1f deg) — disarming + MANUAL",
              diverge_angle * 180.0 / M_PI,
              max_velocity_divergence_rad_ * 180.0 / M_PI);
            if (arm_cmd_pub_) {
              std_msgs::msg::Bool disarm;
              disarm.data = false;
              arm_cmd_pub_->publish(disarm);
            }
            if (mode_cmd_pub_) {
              std_msgs::msg::String mode;
              mode.data = "MANUAL";
              mode_cmd_pub_->publish(mode);
            }
            throw std::runtime_error(
              "Velocity divergence emergency: disarmed and switched to MANUAL");
          }
        }
      }

      // Twist from nav2_controller is Twist2D so linear.z is always 0; keep our own copy.
      prev_vel_ = cmd_vel.twist;

      return cmd_vel;
    }

    void setPlan(const nav_msgs::msg::Path &plan) override
    {
      if (plan.poses.empty()) {
        throw nav2_core::PlannerException("Received plan with zero length");
      }
      plan_ = plan;
      has_reached_xy_tolerance_ = false;
      is_rotating_to_path_ = false;
      was_rotating_to_path_ = false;
      prev_vel_ = geometry_msgs::msg::Twist{};
    }

    void setSpeedLimit(const double &, const bool &) override
    {
      std::cout << "ERROR: speed limit is not supported" << std::endl;
    }
  };

} // namespace orca_nav2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orca_nav2::PurePursuitController3D, nav2_core::Controller)
