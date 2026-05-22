#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <Eigen/Dense>

class AdaptiveThrusterEstimator : public rclcpp::Node
{
public:
  explicit AdaptiveThrusterEstimator(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

private:
  // ── Callbacks ─────────────────────────────────────────────────────────────
  void onServo(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
  void onDvl(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onTimer();
  void onDiagTimer();

  // ── Core RLS update ───────────────────────────────────────────────────────
  // ln_vx_abs = ln(|vx|),  ln_u_abs = ln(|u|)
  void rlsUpdate(double ln_vx_abs, double ln_u_abs);

  // ── Output ────────────────────────────────────────────────────────────────
  void publishEstimate(double vx_hat, const rclcpp::Time & stamp);

  // ── Pub / Sub / Timer ─────────────────────────────────────────────────────
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr           dvl_sub_;
  rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr    servo_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr              pub_;
  // Diagnostics: [k, b, P00, P11, trace(P), dvl_age_s] at 1 Hz.
  // Plot in Foxglove to watch k/b converge and P shrink during learning.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     diag_pub_;
  rclcpp::TimerBase::SharedPtr                                       timer_;
  rclcpp::TimerBase::SharedPtr                                       diag_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr  param_cb_handle_;

  // ── Parameter callback ────────────────────────────────────────────────────
  rcl_interfaces::msg::SetParametersResult onParamChange(
    const std::vector<rclcpp::Parameter> & params);

  // ── Parameters (set once at startup) ─────────────────────────────────────
  int    surge_channel_;
  int    pwm_neutral_;
  int    pwm_range_;
  double lambda_;           // RLS forgetting factor  (≈0.995)
  double u_min_;            // min |u| to engage learning / estimation
  double dvl_cov_thresh_;   // max twist.cov[0] to declare DVL locked
  double accel_thresh_;     // max |Δvx/Δt| (m/s²) for steady-state gate
  double dvl_timeout_s_;    // wall-clock age (s) that triggers fallback
  double fallback_cov_vx_;
  double fallback_cov_vy_;
  double fallback_cov_vz_;

  // ── RLS state ─────────────────────────────────────────────────────────────
  // θ = [ln(k), b]ᵀ  —  the two power-law parameters in log space
  Eigen::Vector2d theta_;
  // P is the 2×2 parameter-error covariance.  Large P → high uncertainty
  // → large Kalman gain → new observations dominate.  The forgetting factor
  // inflates P each step so old data loses influence over time.
  Eigen::Matrix2d P_;

  // ── Runtime bookkeeping ───────────────────────────────────────────────────
  double last_u_{0.0};              // last normalised surge command [-1, 1]
  double last_vx_{0.0};            // vx from last accepted DVL message
  double last_dvl_wall_s_{-1.0};   // steady_clock seconds of last DVL msg
  bool   fallback_active_{false};
  bool   force_publish_{false};     // publish even while DVL is present (testing)
  rclcpp::Time last_dvl_stamp_{0, 0, RCL_ROS_TIME};
};
