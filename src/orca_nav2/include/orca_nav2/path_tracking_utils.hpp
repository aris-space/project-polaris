// MIT License
//
// Copyright (c) 2022 Clyde McQueen
//
// Shared path geometry: L2 distance, pure-pursuit lookahead goal, cross-track / vertical / yaw errors.

#ifndef ORCA_NAV2__PATH_TRACKING_UTILS_HPP_
#define ORCA_NAV2__PATH_TRACKING_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <limits>

#include "angles/angles.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/utils.h"

namespace orca_nav2
{

/** Euclidean distance between two 3D points. */
inline double point_distance_l2(
  const geometry_msgs::msg::Point & p1,
  const geometry_msgs::msg::Point & p2)
{
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  const double dz = p1.z - p2.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * Pure-pursuit style goal: walk the plan until distance to the robot stops decreasing, then return
 * the first pose farther than lookahead_dist from the robot (L2 in XYZ), or the last pose.
 */
inline geometry_msgs::msg::PoseStamped find_pure_pursuit_goal(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose_f_map,
  double lookahead_dist)
{
  if (path.poses.empty()) {
    return geometry_msgs::msg::PoseStamped();
  }

  double min_dist = std::numeric_limits<double>::max();
  bool dist_decreasing = true;

  for (const auto & item : path.poses) {
    const double item_dist = point_distance_l2(item.pose.position, pose_f_map.pose.position);

    if (dist_decreasing) {
      if (item_dist < min_dist) {
        min_dist = item_dist;
      } else {
        dist_decreasing = false;
      }
    }

    if (!dist_decreasing) {
      if (item_dist > lookahead_dist) {
        return item;
      }
    }
  }

  return path.poses.back();
}

/**
 * Tracking errors in path.header.frame_id: signed XY cross-track (m), vertical error (m),
 * yaw error (rad), and the closest point on the path polyline (map frame).
 */
inline void tracking_errors_along_path(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & pose_f_map,
  double & cross_track_xy_m,
  double & vertical_error_m,
  double & yaw_error_rad,
  geometry_msgs::msg::Point & closest_map)
{
  cross_track_xy_m = 0.0;
  vertical_error_m = 0.0;
  yaw_error_rad = 0.0;
  closest_map.x = 0.0;
  closest_map.y = 0.0;
  closest_map.z = 0.0;

  if (path.poses.empty()) {
    return;
  }

  const double rx = pose_f_map.pose.position.x;
  const double ry = pose_f_map.pose.position.y;
  const double rz = pose_f_map.pose.position.z;
  const double robot_yaw = tf2::getYaw(pose_f_map.pose.orientation);

  if (path.poses.size() == 1) {
    const auto & p = path.poses[0].pose.position;
    closest_map = p;
    const double dx = rx - p.x;
    const double dy = ry - p.y;
    const double path_yaw = tf2::getYaw(path.poses[0].pose.orientation);

    const double cp = std::cos(path_yaw) * dy - std::sin(path_yaw) * dx;
    const double sign = (cp >= 0.0) ? 1.0 : -1.0;

    cross_track_xy_m = sign * std::sqrt(dx * dx + dy * dy);
    vertical_error_m = rz - p.z;
    yaw_error_rad = angles::shortest_angular_distance(path_yaw, robot_yaw);
    return;
  }

  double best_dist_sq = std::numeric_limits<double>::infinity();
  double best_qx = rx;
  double best_qy = ry;
  double best_qz = rz;
  double best_path_yaw = tf2::getYaw(path.poses[0].pose.orientation);
  double best_sign = 1.0;

  for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & p1 = path.poses[i].pose.position;
    const auto & p2 = path.poses[i + 1].pose.position;
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
        best_path_yaw = tf2::getYaw(path.poses[i].pose.orientation);
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

}  // namespace orca_nav2

#endif  // ORCA_NAV2__PATH_TRACKING_UTILS_HPP_
