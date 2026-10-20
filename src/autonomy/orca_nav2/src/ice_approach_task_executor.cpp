// Nav2 WaypointTaskExecutor plugin: rise to the ice ceiling, hold firm
// contact, descend, per-waypoint (gated by the mission CSV's up_down flag).
//
// processAtWaypoint() runs synchronously on nav2_waypoint_follower's own
// dedicated callback_group_executor_ thread (same mechanism that already
// lets the stock "wait_at_waypoint" plugin block with rclcpp::sleep_for
// without freezing the rest of Nav2). Our own subscriptions below are left
// on the node's default callback group, which keeps being serviced by the
// node's main spin() loop on a different thread the whole time we block
// here -- no extra executor/thread needed.

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/waypoint_task_executor.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "orca_nav2/param_macro.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

namespace orca_nav2
{

class IceApproachTaskExecutor : public nav2_core::WaypointTaskExecutor
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & weak_parent,
    const std::string & plugin_name) override
  {
    auto node = weak_parent.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    PARAMETER(node, plugin_name, rise_vel, 0.05)
    PARAMETER(node, plugin_name, press_vel, 0.03)
    PARAMETER(node, plugin_name, descend_vel, 0.05)
    PARAMETER(node, plugin_name, crawl_dist_m, 0.15)
    PARAMETER(node, plugin_name, crawl_vel, 0.02)
    PARAMETER(node, plugin_name, depth_tolerance_m, 0.10)
    PARAMETER(node, plugin_name, rise_timeout_s, 60.0)
    PARAMETER(node, plugin_name, descend_timeout_s, 60.0)
    PARAMETER(node, plugin_name, contact_lost_grace_s, 2.0)
    PARAMETER(node, plugin_name, plain_waypoint_pause_ms, 1250.0)
    PARAMETER(node, plugin_name, cmd_rate_hz, 10.0)
    PARAMETER(node, plugin_name, touching_topic, std::string("/ice_touch_detection/touching"))
    PARAMETER(node, plugin_name, ultrasonic_topic, std::string("/top/ultrasonic/distance"))
    PARAMETER(node, plugin_name, odom_topic, std::string("/odometry/filtered/local"))
    PARAMETER(node, plugin_name, mode_topic, std::string("/mode_control/current_mode"))
    PARAMETER(node, plugin_name, meta_topic, std::string("/mission_waypoints_meta"))
    PARAMETER(node, plugin_name, cmd_vel_topic, std::string("/pixhawk/cmd_vel"))

    touching_sub_ = node->create_subscription<std_msgs::msg::Bool>(
      touching_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {touching_.store(msg->data);});

    ultrasonic_sub_ = node->create_subscription<std_msgs::msg::Float32>(
      ultrasonic_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::Float32::ConstSharedPtr msg) {ultrasonic_m_.store(msg->data);});

    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(10),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        current_z_.store(msg->pose.pose.position.z);
      });

    mode_sub_ = node->create_subscription<std_msgs::msg::String>(
      mode_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lk(mode_mutex_);
        current_mode_ = msg->data;
      });

    // TRANSIENT_LOCAL to match mission_waypoint_loader's latched publisher --
    // a QoS mismatch here would silently drop the metadata.
    rclcpp::QoS meta_qos(1);
    meta_qos.transient_local().reliable();
    meta_sub_ = node->create_subscription<std_msgs::msg::Float32MultiArray>(
      meta_topic_, meta_qos,
      std::bind(&IceApproachTaskExecutor::onMeta, this, std::placeholders::_1));

    cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(1).reliable());

    RCLCPP_INFO(logger_, "IceApproachTaskExecutor configured");
  }

  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose, const int & curr_waypoint_index) override
  {
    if (!meta_received_ || curr_waypoint_index < 0 ||
      static_cast<size_t>(curr_waypoint_index) >= up_down_.size())
    {
      RCLCPP_WARN(
        logger_, "No waypoint metadata for index %d; treating as plain waypoint.",
        curr_waypoint_index);
      sleepPlainPause();
      return true;
    }
    if (!up_down_[curr_waypoint_index]) {
      sleepPlainPause();
      return true;
    }

    const double delta_z_cap = delta_z_m_[curr_waypoint_index];
    const double hold_s = hold_s_[curr_waypoint_index];
    const double pre_approach_z = curr_pose.pose.position.z;

    RCLCPP_INFO(
      logger_,
      "Waypoint %d: ice approach requested (delta_z_cap=%.2fm, hold_s=%.2fs, pre_approach_z=%.2fm)",
      curr_waypoint_index, delta_z_cap, hold_s, pre_approach_z);

    if (!isGuided()) {
      RCLCPP_ERROR(
        logger_, "Not in GUIDED mode at waypoint %d; aborting ice approach.", curr_waypoint_index);
      publishZeroTwist();
      return false;
    }

    auto depth_reached = [this, pre_approach_z]() {
        return std::abs(current_z_.load() - pre_approach_z) <= depth_tolerance_m_;
      };

    // --- RISE: ascend, ramping down near the ceiling, until firm contact or abort. ---
    bool rise_ok = true;
    const double rise_start_z = current_z_.load();
    const rclcpp::Time rise_start_t = clock_->now();
    while (rclcpp::ok()) {
      if (!isGuided()) {
        RCLCPP_ERROR(logger_, "GUIDED lost during RISE at waypoint %d.", curr_waypoint_index);
        rise_ok = false;
        break;
      }
      if (touching_.load()) {
        break;
      }
      const double risen = current_z_.load() - rise_start_z;
      if (risen >= delta_z_cap) {
        RCLCPP_WARN(
          logger_,
          "RISE exceeded delta_z_m safety cap (%.2fm) without contact at waypoint %d; "
          "no ice present or sensor fault. Aborting.", delta_z_cap, curr_waypoint_index);
        rise_ok = false;
        break;
      }
      if ((clock_->now() - rise_start_t).seconds() > rise_timeout_s_) {
        RCLCPP_WARN(
          logger_, "RISE timed out (%.1fs) without contact at waypoint %d.",
          rise_timeout_s_, curr_waypoint_index);
        rise_ok = false;
        break;
      }
      // If the ultrasonic reading is invalid (stuck at 0.0) this ramp simply never
      // engages and RISE proceeds at rise_vel_ -- /ice_touch_detection/touching is the
      // authoritative contact signal (it has its own pressure/IMU fallback), this is
      // purely a "slow down before bumping" refinement.
      const float u = ultrasonic_m_.load();
      const double vz = (u > 0.0f && u <= crawl_dist_m_) ? crawl_vel_ : rise_vel_;
      publishTwist(vz);
      sleepTick();
    }

    if (!rise_ok) {
      publishZeroTwist();
      if (isGuided()) {
        waitPhase(depth_reached, descend_timeout_s_, -std::abs(descend_vel_), "RISE-ABORT-DESCEND");
      }
      publishZeroTwist();
      return false;
    }

    // --- HOLD_CONTACT: light constant upward thrust for hold_s, one re-approach if lost. ---
    rclcpp::Time hold_start_t = clock_->now();
    bool contact_lost = false;
    rclcpp::Time contact_lost_t = clock_->now();
    bool reapproach_attempted = false;
    bool guided_lost_in_hold = false;

    while (rclcpp::ok()) {
      if (!isGuided()) {
        RCLCPP_ERROR(logger_, "GUIDED lost during HOLD at waypoint %d.", curr_waypoint_index);
        guided_lost_in_hold = true;
        break;
      }
      const double elapsed = (clock_->now() - hold_start_t).seconds();
      if (elapsed >= hold_s) {
        break;
      }
      if (!touching_.load()) {
        if (!contact_lost) {
          contact_lost = true;
          contact_lost_t = clock_->now();
        }
        const double lost_for = (clock_->now() - contact_lost_t).seconds();
        if (lost_for > contact_lost_grace_s_ && !reapproach_attempted) {
          reapproach_attempted = true;
          RCLCPP_WARN(
            logger_, "Contact lost %.1fs during HOLD at waypoint %d; attempting one re-approach.",
            lost_for, curr_waypoint_index);
          const bool regained = waitPhase(
            [this]() {return touching_.load();}, rise_timeout_s_, rise_vel_, "HOLD-REAPPROACH");
          if (!regained) {
            RCLCPP_WARN(
              logger_,
              "Re-approach failed to regain contact at waypoint %d; proceeding to DESCEND.",
              curr_waypoint_index);
            break;
          }
          contact_lost = false;
          // Re-base so the remaining loop credits time already spent holding.
          hold_start_t = clock_->now() - rclcpp::Duration::from_seconds(elapsed);
          continue;
        }
      } else {
        contact_lost = false;
      }
      publishTwist(press_vel_);
      sleepTick();
    }

    // --- DESCEND: back to the pre-approach navigation depth. ---
    publishZeroTwist();
    bool success = false;
    if (!guided_lost_in_hold) {
      success = waitPhase(depth_reached, descend_timeout_s_, -std::abs(descend_vel_), "DESCEND");
      if (!success) {
        RCLCPP_WARN(
          logger_,
          "DESCEND did not reach pre-approach depth within %.1fs at waypoint %d; "
          "continuing anyway.", descend_timeout_s_, curr_waypoint_index);
      }
    }
    publishZeroTwist();
    return success;
  }

private:
  rclcpp::Logger logger_{rclcpp::get_logger("ice_approach_task_executor")};
  rclcpp::Clock::SharedPtr clock_;

  double rise_vel_{};
  double press_vel_{};
  double descend_vel_{};
  double crawl_dist_m_{};
  double crawl_vel_{};
  double depth_tolerance_m_{};
  double rise_timeout_s_{};
  double descend_timeout_s_{};
  double contact_lost_grace_s_{};
  double plain_waypoint_pause_ms_{};
  double cmd_rate_hz_{};
  std::string touching_topic_;
  std::string ultrasonic_topic_;
  std::string odom_topic_;
  std::string mode_topic_;
  std::string meta_topic_;
  std::string cmd_vel_topic_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr touching_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ultrasonic_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr meta_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  std::atomic<bool> touching_{false};
  std::atomic<float> ultrasonic_m_{0.0f};
  std::atomic<double> current_z_{0.0};
  mutable std::mutex mode_mutex_;
  std::string current_mode_{"UNKNOWN"};

  // Per-waypoint metadata cache, index-aligned with /mission_waypoints_enu.
  bool meta_received_{false};
  std::vector<bool> up_down_;
  std::vector<float> delta_z_m_;
  std::vector<float> hold_s_;

  void onMeta(std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
  {
    constexpr size_t stride = 3;
    if (msg->data.size() % stride != 0) {
      RCLCPP_ERROR(
        logger_, "Malformed %s: size %zu is not a multiple of %zu",
        meta_topic_.c_str(), msg->data.size(), stride);
      return;
    }
    const size_t n = msg->data.size() / stride;
    up_down_.assign(n, false);
    delta_z_m_.assign(n, 0.0f);
    hold_s_.assign(n, 0.0f);
    for (size_t i = 0; i < n; ++i) {
      up_down_[i] = msg->data[i * stride + 0] > 0.5f;
      delta_z_m_[i] = msg->data[i * stride + 1];
      hold_s_[i] = msg->data[i * stride + 2];
    }
    meta_received_ = true;
    RCLCPP_INFO(logger_, "Received waypoint metadata for %zu waypoints", n);
  }

  bool isGuided() const
  {
    std::lock_guard<std::mutex> lk(mode_mutex_);
    return current_mode_ == "GUIDED";
  }

  void publishTwist(double vz)
  {
    geometry_msgs::msg::Twist msg;
    msg.linear.z = vz;
    cmd_vel_pub_->publish(msg);
  }

  void publishZeroTwist() {publishTwist(0.0);}

  void sleepTick()
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(1.0 / cmd_rate_hz_));
  }

  void sleepPlainPause()
  {
    std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(plain_waypoint_pause_ms_));
  }

  // Publish vz_cmd at cmd_rate_hz_ until done_cond() is true, timeout_s elapses, or
  // GUIDED is lost. Returns whether done_cond() was satisfied.
  bool waitPhase(
    const std::function<bool()> & done_cond,
    double timeout_s,
    double vz_cmd,
    const char * phase_name)
  {
    const rclcpp::Time start = clock_->now();
    while (rclcpp::ok()) {
      if (!isGuided()) {
        RCLCPP_ERROR(logger_, "GUIDED lost during %s", phase_name);
        return false;
      }
      if (done_cond()) {
        return true;
      }
      if ((clock_->now() - start).seconds() > timeout_s) {
        RCLCPP_WARN(logger_, "%s timed out after %.1fs", phase_name, timeout_s);
        return false;
      }
      publishTwist(vz_cmd);
      sleepTick();
    }
    return false;
  }
};

}  // namespace orca_nav2

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(orca_nav2::IceApproachTaskExecutor, nav2_core::WaypointTaskExecutor)
