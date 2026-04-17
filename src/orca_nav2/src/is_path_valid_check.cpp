#include <cmath>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "orca_nav2/path_tracking_utils.hpp"
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
  return BT::NodeStatus::SUCCESS;
}

}  // namespace orca_nav2

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<orca_nav2::IsPathValidCheck>("IsStraightPathValid");
}

