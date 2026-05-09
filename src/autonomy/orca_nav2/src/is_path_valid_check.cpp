#include <cmath>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "orca_nav2/path_tracking_utils.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace orca_nav2
{

class IsPathValidCheck : public BT::ConditionNode
{
public:
  IsPathValidCheck(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path", "Path to Check"),
      BT::InputPort<double>("max_dist", 0.5, "Maximum allowed distance from path."),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "If wired, path end must match this goal within xy/z tolerances (avoids stale paths)."),
      BT::InputPort<double>("xy_goal_tolerance", 0.35, "XY distance path end vs goal (same frame)."),
      BT::InputPort<double>("z_goal_tolerance", 0.35, "Z distance path end vs goal (same frame)."),
    };
  }

  BT::NodeStatus tick() override;
};

IsPathValidCheck::IsPathValidCheck(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
}

BT::NodeStatus IsPathValidCheck::tick()
{
  nav_msgs::msg::Path path;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  geometry_msgs::msg::PoseStamped pose;
  geometry_msgs::msg::Point closest_map;
  double max_dist;
  double cross_track_xy_m;
  double vertical_error_m;
  double yaw_error_rad;
  double transform_tolerance = 1.0;

  if (!getInput("path", path) || !getInput("max_dist", max_dist)) {
    return BT::NodeStatus::FAILURE;
  }
  if (path.poses.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  if (!config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer)) {
    return BT::NodeStatus::FAILURE;
  }

  if (!nav2_util::getCurrentPose(pose, *tf_buffer, "map", "base_link", transform_tolerance)) {
    return BT::NodeStatus::FAILURE;
  }

  tracking_errors_along_path(
    path, pose, cross_track_xy_m, vertical_error_m, yaw_error_rad, closest_map);

  if (std::abs(cross_track_xy_m) > max_dist || std::abs(vertical_error_m) > max_dist) {
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::msg::PoseStamped goal;
  if (getInput("goal", goal)) {
    double xy_tol = 0.35;
    double z_tol = 0.35;
    getInput("xy_goal_tolerance", xy_tol);
    getInput("z_goal_tolerance", z_tol);

    geometry_msgs::msg::PoseStamped path_end = path.poses.back();
    if (path_end.header.frame_id.empty()) {
      path_end.header = path.header;
    }

    geometry_msgs::msg::PoseStamped goal_in_path_frame;
    if (goal.header.frame_id == path.header.frame_id) {
      goal_in_path_frame = goal;
    } else {
      try {
        auto transform = tf_buffer->lookupTransform(
          path.header.frame_id, goal.header.frame_id, tf2::TimePointZero,
          tf2::durationFromSec(transform_tolerance));
        tf2::doTransform(goal, goal_in_path_frame, transform);
        goal_in_path_frame.header.frame_id = path.header.frame_id;
      } catch (const tf2::TransformException &) {
        return BT::NodeStatus::FAILURE;
      }
    }

    const auto & pe = path_end.pose.position;
    const auto & ge = goal_in_path_frame.pose.position;
    const double dx = pe.x - ge.x;
    const double dy = pe.y - ge.y;
    const double dz = pe.z - ge.z;
    const double xy_err = std::hypot(dx, dy);
    if (xy_err > xy_tol || std::abs(dz) > z_tol) {
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace orca_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::IsPathValidCheck>("IsStraightPathValid");
}

