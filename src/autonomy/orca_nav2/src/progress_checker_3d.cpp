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

#include <string>
#include <cmath>

#include "nav2_core/progress_checker.hpp"
#include "orca_nav2/param_macro.hpp"
//#include "orca_shared/util.hpp" // DONE: Delete this dependency!

namespace orca_nav2
{

class ProgressChecker3D : public nav2_core::ProgressChecker
{
  rclcpp::Clock::SharedPtr clock_;

  // Progress is defined as moving by more than radius_ w/in the time_allowance_
  double radius_{};
  double time_allowance_{};
  rclcpp::Duration time_allowance_d_{0, 0};

  geometry_msgs::msg::Pose baseline_;
  rclcpp::Time baseline_time_;
  bool baseline_set_{false};

  double get_dist_L2_norm(const geometry_msgs::msg::Point & p1, const geometry_msgs::msg::Point & p2) const
  {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    double dist_L2_norm = std::sqrt(dx * dx + dy * dy + dz * dz);
    return dist_L2_norm;
  }

  void set_baseline(const geometry_msgs::msg::Pose & pose)
  {
    baseline_ = pose;
    baseline_time_ = clock_->now();
    baseline_set_ = true;
  }

public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & weak_parent,
    const std::string & plugin_name) override
  {
    auto parent = weak_parent.lock();

    clock_ = parent->get_clock();

    PARAMETER(parent, plugin_name, radius, 0.5) // TODO: Is this usefulwith this values?
    PARAMETER(parent, plugin_name, time_allowance, 10.0) // yes v_r >= 0.05 m/s

    time_allowance_d_ = rclcpp::Duration::from_seconds(time_allowance_);

    RCLCPP_INFO(parent->get_logger(), "Nav2: ProgressChecker3D configured");
  }

  bool check(geometry_msgs::msg::PoseStamped & pose) override
  {   

    if (!baseline_set_ || get_dist_L2_norm(pose.pose.position, baseline_.position) > radius_) {
      set_baseline(pose.pose);
      return true;
    }

    return (clock_->now() - baseline_time_) < time_allowance_d_;
  }

  void reset() override { baseline_set_ = false; }
};

}  // namespace orca_nav2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orca_nav2::ProgressChecker3D, nav2_core::ProgressChecker)
