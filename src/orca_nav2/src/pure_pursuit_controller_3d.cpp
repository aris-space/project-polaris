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

// for the node and launch file...
// from launch_ros.actions import Node

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "orca_nav2/param_macro.hpp"
// Deleted: #include "orca_shared/util.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "pluginlib/class_loader.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace orca_nav2
{

  constexpr bool sign(const double &v) { return v > 0; }

  class Limiter
  {
    double max_a_{};
    double max_dv_{};

  public:
    Limiter() = default;

    Limiter(const double &max_a, const double &dt)
        : max_a_{max_a}, max_dv_{max_a * dt}
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
      auto decel_v = std::sqrt(2 * std::abs(goal_dist) * max_a_);
      auto result_v = std::min(std::abs(v), decel_v);
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

    // Parameters
    double x_vel_{};
    double x_accel_{};
    double z_vel_{};
    double z_accel_{};
    double yaw_vel_{};
    double yaw_accel_{};
    double lookahead_dist_{};
    double transform_tolerance_{};
    double goal_tolerance_{}; // Stop motion when we're very close to the goal
    double tick_rate_{};      // Tick rate, used to compute dt

    Limiter x_limiter_;
    Limiter z_limiter_;
    Limiter yaw_limiter_;

    rclcpp::Duration transform_tolerance_d_{0, 0};

    // Plan from StraightLinePlanner3D
    nav_msgs::msg::Path plan_;

    // Keep track of the previous cmd_vel to limit acceleration
    geometry_msgs::msg::Twist prev_vel_{};

    // Updated 12-Apr-22: the Orca motion model might be wrong for various
    // reasons, so actual accel/vel might be higher or lower than desired
    // accel/vel. Error factors are introduced so that actual yaw vel might be
    // somewhere in the range:
    //    [(1 - yaw_error_) * yaw_vel_, (1 + yaw_error_) * yaw_vel_]

    double x_error_{};
    double yaw_error_{};

    bool publish_tracking_error_{true};
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr cross_track_xy_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr vertical_error_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>::SharedPtr yaw_error_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PointStamped>::SharedPtr closest_point_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr robot_pose_map_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::TwistStamped>::SharedPtr robot_twist_pub_;

    static constexpr double lower(double v, double e) { return (1.0 - e) * v; }
    static constexpr double upper(double v, double e) { return (1.0 + e) * v; }

    constexpr double x_vel_upper() const { return upper(x_vel_, x_error_); }
    constexpr double yaw_vel_lower() const { return lower(yaw_vel_, yaw_error_); }

    // Return the first pose in the plan > lookahead distance away, or the last pose in the plan
    // ADDED: Built-in safe TF2 Transform helper (Replaces orca_shared math)
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
      double dist_L2_norm = std::sqrt(dx * dx + dy * dy + dz * dz);
      return dist_L2_norm;
    }

    // Shortest distance in the XY plane from the robot to the plan polyline, Z offset at that
    // closest point (robot_z - path_z), and yaw error: shortest angle (rad) from path heading to
    // robot yaw, all in plan_.header.frame_id (typically map). Path heading is atan2 along the
    // winning segment, or the single pose's orientation if one pose.
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

        //For a direction in the cross-tracking error.
        const double cp = std::cos(path_yaw)*dy - std::sin(path_yaw)*dx;
        const double sign = (cp >= 0) ? 1.0 : -1.0;


        cross_track_xy_m = sign * std::sqrt(dx * dx + dy * dy);
        vertical_error_m = rz - p.z;        
        yaw_error_rad = angles::shortest_angular_distance(path_yaw, robot_yaw);
        return;
      }

      double best_dist_sq = std::numeric_limits<double>::infinity();
      double best_qx = rx;
      double best_qy = ry;
      double best_qz = rz;
      double best_path_yaw = tf2::getYaw(plan_.poses[0].pose.orientation);

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

    geometry_msgs::msg::PoseStamped
    find_goal(const geometry_msgs::msg::PoseStamped &pose_f_map) const
    {
      // Walk the plan calculating distance. The plan may be stale, so distances may be
      // decreasing for a while. When they start to increase we've found the closest pose. Then look
      // for the first pose > lookahead_dist_. Return the last pose if we run out of poses.
      if (plan_.poses.empty())
      {
        return geometry_msgs::msg::PoseStamped();
      }

      auto min_dist = std::numeric_limits<double>::max();
      bool dist_decreasing = true;

      for (const auto &item : plan_.poses)
      {
        auto item_dist_L2_norm = get_dist_L2_norm(item.pose.position, pose_f_map.pose.position);

        if (dist_decreasing)
        {
          if (item_dist_L2_norm < min_dist)
          {
            min_dist = item_dist_L2_norm;
          }
          else
          {
            dist_decreasing = false;
          }
        }

        if (!dist_decreasing)
        {
          if (item_dist_L2_norm > lookahead_dist_)
          {
            return item;
          }
        }
      }

      return plan_.poses.back();
    }

    // Modified pure pursuit path tracking algorithm: works in 3D and supports deceleration
    // Reference "Implementation of the Pure Pursuit Path Tracking Algorithm" by R. Craig Coulter
    geometry_msgs::msg::Twist
    pure_pursuit_3d(const geometry_msgs::msg::PoseStamped &pose_f_odom) const
    {
      // Transform pose odom -> map
      geometry_msgs::msg::PoseStamped pose_f_map;
      if (!transform_pose(pose_f_odom, pose_f_map, plan_.header.frame_id))
      {
        return geometry_msgs::msg::Twist{};
      }

      // Find goal
      auto goal_f_map = find_goal(pose_f_map);

      // Plan poses are stale, update the timestamp to get recent map -> odom -> base transforms
      goal_f_map.header.stamp = pose_f_map.header.stamp;

      // Transform goal map -> base
      geometry_msgs::msg::PoseStamped goal_f_base;
      if (!transform_pose(goal_f_map, goal_f_base, base_frame_id_))
      {
        return geometry_msgs::msg::Twist{};
      }

      auto xy_dist_sq = goal_f_base.pose.position.x * goal_f_base.pose.position.x +
                        goal_f_base.pose.position.y * goal_f_base.pose.position.y;
      auto xy_dist_L2_norm = std::sqrt(xy_dist_sq);
      auto z_dist = std::abs(goal_f_base.pose.position.z);

#if 0
    // Useful for debugging, but happens frequently as the sub decelerates
    if (z_dist < goal_tolerance_ && xy_dist_L2_norm < goal_tolerance_) {
      std::cout << "Decelerating / coasting" << std::endl;
      std::cout << "pose_f_odom: " << orca::to_str(pose_f_odom) << std::endl;
      std::cout << "pose_f_map: " << orca::to_str(pose_f_map) << std::endl;
      std::cout << "goal_f_map: " << orca::to_str(goal_f_map) << std::endl;
      std::cout << "goal_f_base: " << orca::to_str(goal_f_base) << std::endl;
      std::cout << "prev_vel: " << orca::to_str(prev_vel_) << std::endl;
    }
#endif

      geometry_msgs::msg::Twist cmd_vel;

      // Calc linear.z
      if (z_dist > goal_tolerance_)
      {
        cmd_vel.linear.z = goal_f_base.pose.position.z > 0 ? z_vel_ : -z_vel_;

        // Decelerate
        z_limiter_.decelerate(cmd_vel.linear.z, goal_f_base.pose.position.z);
      }

      // Calc linear.x and angular.z using pure pursuit algorithm
      if (xy_dist_L2_norm > goal_tolerance_)
      {
        if (goal_f_base.pose.position.x > 0)
        {
          // Goal is ahead of the sub: move forward along the shortest curve
          auto curvature = 2.0 * goal_f_base.pose.position.y / xy_dist_sq;
          if (std::abs(curvature) * x_vel_upper() <= yaw_vel_lower())
          {
            // Move at constant velocity
            cmd_vel.linear.x = x_vel_;
            cmd_vel.angular.z = curvature * x_vel_;
          }
          else
          {
            // Tight curve... don't exceed angular velocity limit
            cmd_vel.linear.x = yaw_vel_ / std::abs(curvature);
            cmd_vel.angular.z = curvature > 0 ? yaw_vel_ : -yaw_vel_;
          }

          // Decelerate
          x_limiter_.decelerate(cmd_vel.linear.x, xy_dist_L2_norm);
        }
        else
        {
          // Goal is behind the sub: rotate to face it
          cmd_vel.angular.z = goal_f_base.pose.position.y > 0 ? yaw_vel_ : -yaw_vel_;
        }
      }

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
      PARAMETER(parent, name, x_error, 0.0)
      PARAMETER(parent, name, yaw_error, 0.0)
      PARAMETER(parent, name, publish_tracking_error, true)

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

      x_limiter_ = Limiter(x_accel_, 1. / tick_rate_);
      z_limiter_ = Limiter(z_accel_, 1. / tick_rate_);
      yaw_limiter_ = Limiter(yaw_accel_, 1. / tick_rate_);

      transform_tolerance_d_ = rclcpp::Duration::from_seconds(transform_tolerance_);

      RCLCPP_INFO(logger_, "PurePursuitController3D configured");
    }

    void cleanup() override
    {
      cross_track_xy_pub_.reset();
      vertical_error_pub_.reset();
      yaw_error_pub_.reset();
      closest_point_pub_.reset();
      robot_pose_map_pub_.reset();
      robot_twist_pub_.reset();
    }

    void activate() override
    {
      if (cross_track_xy_pub_) {
        cross_track_xy_pub_->on_activate();
      }
      if (vertical_error_pub_) {
        vertical_error_pub_->on_activate();
      }
      if (yaw_error_pub_) {
        yaw_error_pub_->on_activate();
      }
      if (closest_point_pub_) {
        closest_point_pub_->on_activate();
      }
      if (robot_pose_map_pub_) {
        robot_pose_map_pub_->on_activate();
      }
      if (robot_twist_pub_) {
        robot_twist_pub_->on_activate();
      }
    }

    void deactivate() override
    {
      if (cross_track_xy_pub_) {
        cross_track_xy_pub_->on_deactivate();
      }
      if (vertical_error_pub_) {
        vertical_error_pub_->on_deactivate();
      }
      if (yaw_error_pub_) {
        yaw_error_pub_->on_deactivate();
      }
      if (closest_point_pub_) {
        closest_point_pub_->on_deactivate();
      }
      if (robot_pose_map_pub_) {
        robot_pose_map_pub_->on_deactivate();
      }
      if (robot_twist_pub_) {
        robot_twist_pub_->on_deactivate();
      }
    }

    // Pose is base_f_odom, it's 3D, and it comes from /tf via:
    //   nav2_controller::ControllerServer::getRobotPose
    //   Using the local_costmap (odom frame), not the global_costmap (map frame)
    //   nav2_costmap_2d::Costmap2DROS::getRobotPose
    //   nav2_util::getCurrentPose
    //
    // Twist comes from /odom, but it's stripped to 2D in nav2_controller::ControllerServer, so
    // ignore it.
    geometry_msgs::msg::TwistStamped computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *) override
    {
      geometry_msgs::msg::TwistStamped cmd_vel;
      cmd_vel.header = pose.header;

      if (publish_tracking_error_ && cross_track_xy_pub_ && vertical_error_pub_ && yaw_error_pub_ &&
        closest_point_pub_ && robot_pose_map_pub_ && robot_twist_pub_ && !plan_.poses.empty())
      {
        geometry_msgs::msg::PoseStamped pose_f_map;
        if (transform_pose(pose, pose_f_map, plan_.header.frame_id)) {
          double cross_xy = 0.0;
          double z_err = 0.0;
          double yaw_err = 0.0;
          geometry_msgs::msg::Point closest_map;
          tracking_error_from_plan(pose_f_map, cross_xy, z_err, yaw_err, closest_map);
          std_msgs::msg::Float64 cross_msg;
          cross_msg.data = cross_xy;
          cross_track_xy_pub_->publish(cross_msg);
          std_msgs::msg::Float64 z_msg;
          z_msg.data = z_err;
          vertical_error_pub_->publish(z_msg);
          std_msgs::msg::Float64 yaw_msg;
          yaw_msg.data = yaw_err;
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
      }

      // Track the plan
      cmd_vel.twist = pure_pursuit_3d(pose);

      // Limit acceleration
      x_limiter_.limit(cmd_vel.twist.linear.x, prev_vel_.linear.x);
      z_limiter_.limit(cmd_vel.twist.linear.z, prev_vel_.linear.z);
      yaw_limiter_.limit(cmd_vel.twist.angular.z, prev_vel_.angular.z);

      // Twist parameter from nav2_controller is generated from a Twist2D, so linear.z is always 0
      // Keep a copy of the previous cmd_vel instead
      prev_vel_ = cmd_vel.twist;

#if 0
    // I'm getting jumps in velocity from max_v to 0 to max_v, but it is not coming from this
    // routine. Somewhere upstream there appears to be a "cmd_vel = {}".

    if (cmd_vel.twist.angular.z < 0.01 && cmd_vel.twist.angular.z > -0.01) {
      std::cout << "sending 0 yaw vel" << std::endl;
    }
#endif

      return cmd_vel;
    }

    void setPlan(const nav_msgs::msg::Path &plan) override
    {
      if (plan.poses.empty())
      {
        throw nav2_core::PlannerException("Received plan with zero length");
      }
      plan_ = plan;
    }

    void setSpeedLimit(const double &, const bool &) override
    {
      // Not supported
      std::cout << "ERROR: speed limit is not supported" << std::endl;
    }
  };

} // namespace orca_nav2

#include "pluginlib/class_list_macros.hpp"

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(orca_nav2::PurePursuitController3D, nav2_core::Controller)
