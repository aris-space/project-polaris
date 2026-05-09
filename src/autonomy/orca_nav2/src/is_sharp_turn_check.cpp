#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "orca_nav2/path_tracking_utils.hpp"
#include "tf2_ros/buffer.h"

namespace
{
rclcpp::Clock g_log_throttle_clock(RCL_STEADY_TIME);
}

namespace orca_nav2
{

class IsSharpTurnCheck : public BT::ConditionNode
{
public:
    IsSharpTurnCheck(
        const std::string & condition_name,
        const BT::NodeConfiguration & conf);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<nav_msgs::msg::Path>("path", "Path to Check"),
            BT::InputPort<double>("min_angle_rad", "Angle to trigger the Yes SharpTurn"),
            BT::InputPort<double>("release_angle_rad", "Angle to release the SharpTurn"),
        };
    }

    BT::NodeStatus tick() override;

private:
    bool is_replaning_;
    bool prev_is_replaning_;
};

IsSharpTurnCheck::IsSharpTurnCheck(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), is_replaning_(false), prev_is_replaning_(false)
{
}

BT::NodeStatus IsSharpTurnCheck::tick()
{
    nav_msgs::msg::Path path;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::Point closest_map;

    double min_angle_rad;
    double release_angle_rad;

    double cross_track_xy_m;
    double vertical_error_m;
    double yaw_error_rad;
    double transform_tolerance = 1.0;

    auto logger = rclcpp::get_logger("bt_is_sharp_turn");

    if (!getInput("path", path) || !getInput("min_angle_rad", min_angle_rad) ||
        !getInput("release_angle_rad", release_angle_rad))
    {
        RCLCPP_WARN_THROTTLE(
            logger, g_log_throttle_clock, 5000,
            "IsSharpTurn: missing path or angle ports (check BT XML blackboard wiring)");
        return BT::NodeStatus::FAILURE;
    }
    if (path.poses.empty()) {
        RCLCPP_WARN_THROTTLE(
            logger, g_log_throttle_clock, 5000, "IsSharpTurn: path has no poses yet");
        return BT::NodeStatus::FAILURE;
    }
    if (!config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer)) {
        RCLCPP_WARN_THROTTLE(
            logger, g_log_throttle_clock, 5000,
            "IsSharpTurn: blackboard has no tf_buffer (Nav2 should inject this)");
        return BT::NodeStatus::FAILURE;
    }

    if (!nav2_util::getCurrentPose(pose, *tf_buffer, "map", "base_link", transform_tolerance)) {
        RCLCPP_WARN_THROTTLE(
            logger, g_log_throttle_clock, 5000,
            "IsSharpTurn: getCurrentPose(map -> base_link) failed; "
            "condition fails and BT falls back to LongStraightLine");
        return BT::NodeStatus::FAILURE;
    }

    tracking_errors_along_path(
        path, pose, cross_track_xy_m, vertical_error_m, yaw_error_rad, closest_map);
    double abs_yaw_error_rad = std::abs(yaw_error_rad);

    // Schmitt-trigger: hysteresis between sharp-turn and pure-pursuit branches.

    if (!is_replaning_) {
        if (abs_yaw_error_rad > min_angle_rad) {
            is_replaning_ = true;
        }
    } else {
        if (abs_yaw_error_rad < release_angle_rad) {
            is_replaning_ = false;
        }
    }

    if (is_replaning_ != prev_is_replaning_) {
        if (is_replaning_) {
            RCLCPP_INFO(
                logger,
                "SharpTurn branch: ON (yaw_err=%.3f rad, enter > %.3f)",
                yaw_error_rad, min_angle_rad);
        } else {
            RCLCPP_INFO(
                logger,
                "PurePursuit branch: ON (yaw_err=%.3f rad, release < %.3f)",
                yaw_error_rad, release_angle_rad);
        }
        prev_is_replaning_ = is_replaning_;
    }

    return is_replaning_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace orca_nav2


BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<orca_nav2::IsSharpTurnCheck>("IsSharpTurn");
}