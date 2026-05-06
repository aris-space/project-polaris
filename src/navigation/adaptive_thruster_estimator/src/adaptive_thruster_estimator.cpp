/**
 * adaptive_thruster_estimator — online RLS identification of AUV thruster model
 *
 * Physics model:  vx = k · sign(u) · |u|^b
 *
 * Linearisation via log transform:
 *   y  = ln|vx|  =  ln(k)  +  b · ln|u|  =  φᵀ θ
 *   φ  = [1,  ln|u|]ᵀ
 *   θ  = [ln(k),  b]ᵀ   ← estimated online
 *
 * Sign handling for reverse thrust:
 *   The log domain works on magnitudes only.  Sign is stripped before the
 *   update and re-applied during estimation:
 *     v̂x = sign(u) · exp(φᵀ θ)
 *   The learning gate additionally requires sign(u) == sign(vx) so that
 *   deceleration transients (where drag sign conflicts with velocity sign)
 *   do not pollute the steady-state thrust map.
 *
 * P-matrix update (see rlsUpdate for code):
 *   K  = P φ / (λ + φᵀ P φ)          Kalman gain (2×1)
 *   θ ← θ + K · (y − φᵀ θ)           parameter update
 *   P ← (1/λ)(I − K φᵀ) P            covariance update with forgetting
 *
 *   The (1/λ) factor inflates P each step, making the effective memory
 *   window ≈ 1/(1−λ) samples.  At λ=0.995 and 9 Hz DVL this is ~22 s.
 *   After the K·φᵀ correction shrinks P along the excited direction, the
 *   net effect is that recent data dominates while old data fades.
 *   Symmetry is enforced after each step to prevent floating-point drift.
 */

#include "adaptive_thruster_estimator/adaptive_thruster_estimator.hpp"

#include <chrono>
#include <cmath>
#include <vector>

using std::placeholders::_1;
using namespace std::chrono_literals;

// ── helpers ──────────────────────────────────────────────────────────────────
static double wallNow()
{
  using namespace std::chrono;
  return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ── Constructor ───────────────────────────────────────────────────────────────
AdaptiveThrusterEstimator::AdaptiveThrusterEstimator(
  const rclcpp::NodeOptions & options)
: Node("adaptive_thruster_estimator", options)
{
  // ── Parameter declarations ─────────────────────────────────────────────────
  declare_parameter("surge_channel",   0);
  declare_parameter("pwm_neutral",     1500);
  declare_parameter("pwm_range",       500);
  declare_parameter("lambda",          0.995);
  declare_parameter("u_min",           0.10);
  // DVL lock gate: twist.covariance[0] below this → DVL is bottom-locked.
  // WaterLinked A50 publishes ~1e-8 when locked; 1e-4 gives comfortable margin.
  declare_parameter("dvl_cov_thresh",  1e-4);
  // Steady-state gate: skip learning when |Δvx/Δt| exceeds this (m/s²).
  // Prevents capturing inertial lag (added mass) in the drag model.
  declare_parameter("accel_thresh",    0.05);
  declare_parameter("dvl_timeout_s",   3.0);
  declare_parameter("fallback_cov_vx", 0.0057);
  declare_parameter("fallback_cov_vy", 0.0027);
  declare_parameter("fallback_cov_vz", 0.0009);
  // Prior from Zermatt 2026-04-29 recordings (zero-current, depth-hold survey)
  declare_parameter("init_k",          0.780);
  declare_parameter("init_b",          0.964);

  surge_channel_   = get_parameter("surge_channel").as_int();
  pwm_neutral_     = get_parameter("pwm_neutral").as_int();
  pwm_range_       = get_parameter("pwm_range").as_int();
  lambda_          = get_parameter("lambda").as_double();
  u_min_           = get_parameter("u_min").as_double();
  dvl_cov_thresh_  = get_parameter("dvl_cov_thresh").as_double();
  accel_thresh_    = get_parameter("accel_thresh").as_double();
  dvl_timeout_s_   = get_parameter("dvl_timeout_s").as_double();
  fallback_cov_vx_ = get_parameter("fallback_cov_vx").as_double();
  fallback_cov_vy_ = get_parameter("fallback_cov_vy").as_double();
  fallback_cov_vz_ = get_parameter("fallback_cov_vz").as_double();

  const double init_k = get_parameter("init_k").as_double();
  const double init_b = get_parameter("init_b").as_double();

  theta_ << std::log(init_k), init_b;

  // High initial P → high uncertainty → fast convergence on first data.
  // Off-diagonals zero: no prior correlation between ln(k) and b.
  P_ = Eigen::Matrix2d::Identity() * 1e4;

  // ── Pub / Sub / Timer ──────────────────────────────────────────────────────
  dvl_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/sensors/dvl/odometry_cov", 10,
    std::bind(&AdaptiveThrusterEstimator::onDvl, this, _1));

  servo_sub_ = create_subscription<std_msgs::msg::Int16MultiArray>(
    "/pixhawk/servo_output_raw", 10,
    std::bind(&AdaptiveThrusterEstimator::onServo, this, _1));

  pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "/sensors/thruster/odometry_cov", 10);

  diag_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
    "~/diagnostics", 10);

  // Fallback publishing at 10 Hz — matches servo update rate so ZOH lag is
  // at most one servo period (100 ms).
  timer_ = create_wall_timer(100ms,
    std::bind(&AdaptiveThrusterEstimator::onTimer, this));

  // Diagnostics at 1 Hz — fields: [k, b, P00, P11, trace(P), dvl_age_s]
  diag_timer_ = create_wall_timer(1000ms,
    std::bind(&AdaptiveThrusterEstimator::onDiagTimer, this));

  RCLCPP_INFO(get_logger(),
    "init k=%.3f b=%.3f  lambda=%.4f  u_min=%.2f  accel_thresh=%.3f m/s²",
    init_k, init_b, lambda_, u_min_, accel_thresh_);
}

// ── Servo callback ────────────────────────────────────────────────────────────
void AdaptiveThrusterEstimator::onServo(
  const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
  if (surge_channel_ >= static_cast<int>(msg->data.size())) {
    return;
  }
  last_u_ = static_cast<double>(msg->data[surge_channel_] - pwm_neutral_)
            / static_cast<double>(pwm_range_);
}

// ── DVL callback (learning only — fallback publishing is timer-driven) ────────
void AdaptiveThrusterEstimator::onDvl(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  last_dvl_wall_s_ = wallNow();  // update age regardless of lock status

  const double cov_vx   = msg->twist.covariance[0];
  const bool   dvl_locked = (cov_vx > 0.0 && cov_vx < dvl_cov_thresh_);
  if (!dvl_locked) {
    return;
  }

  const double vx = msg->twist.twist.linear.x;

  // ── Acceleration gate ──────────────────────────────────────────────────────
  bool steady = true;
  if (last_dvl_stamp_.nanoseconds() > 0) {
    const double dt = (rclcpp::Time(msg->header.stamp) - last_dvl_stamp_).seconds();
    if (dt > 0.0 && dt < 1.0) {
      steady = std::abs(vx - last_vx_) / dt < accel_thresh_;
    }
  }
  last_dvl_stamp_ = msg->header.stamp;
  last_vx_        = vx;

  // ── Learning gate: excitation + sign consistency + steady state ────────────
  const double abs_u  = std::abs(last_u_);
  const double abs_vx = std::abs(vx);
  const bool   same_sign = (last_u_ * vx > 0.0);  // avoids deceleration data

  if (steady && abs_u > u_min_ && abs_vx > u_min_ && same_sign) {
    rlsUpdate(std::log(abs_vx), std::log(abs_u));
  }
}

// ── Timer: fallback publishing ────────────────────────────────────────────────
void AdaptiveThrusterEstimator::onTimer()
{
  const double dvl_age_s  = wallNow() - last_dvl_wall_s_;
  const bool   dvl_present = (last_dvl_wall_s_ > 0.0) && (dvl_age_s < dvl_timeout_s_);

  if (dvl_present) {
    if (fallback_active_) {
      const double k = std::exp(theta_(0));
      RCLCPP_INFO(get_logger(),
        "DVL restored — thruster fallback off (k=%.3f b=%.3f)", k, theta_(1));
      fallback_active_ = false;
    }
    return;  // DVL present: nothing to publish
  }

  if (!fallback_active_) {
    const double k = std::exp(theta_(0));
    RCLCPP_INFO(get_logger(),
      "DVL absent — thruster fallback active (k=%.3f b=%.3f)", k, theta_(1));
    fallback_active_ = true;
  }

  const double abs_u  = std::abs(last_u_);
  double vx_hat = 0.0;
  if (abs_u > u_min_) {
    // v̂x = sign(u) · exp(ln k) · |u|^b  =  sign(u) · k · |u|^b
    vx_hat = std::exp(theta_(0)) * std::pow(abs_u, theta_(1))
             * (last_u_ > 0.0 ? 1.0 : -1.0);
  }

  publishEstimate(vx_hat, this->now());
}

// ── RLS update ────────────────────────────────────────────────────────────────
void AdaptiveThrusterEstimator::rlsUpdate(double ln_vx_abs, double ln_u_abs)
{
  // Regressor for this sample: φ = [1, ln|u|]ᵀ
  const Eigen::Vector2d phi{1.0, ln_u_abs};

  // Kalman gain.  Denominator λ + φᵀPφ is a positive scalar.
  // Large P (uncertain) → large K → new observation dominates.
  const double           denom = lambda_ + phi.dot(P_ * phi);
  const Eigen::Vector2d  K     = (P_ * phi) / denom;

  // Innovation in log space
  const double error = ln_vx_abs - phi.dot(theta_);

  // Parameter update
  theta_ += K * error;

  // Clamp θ to physically meaningful range to prevent RLS runaway on bad data.
  // k ∈ [0.05, 10]  →  ln(k) ∈ [ln(0.05), ln(10)]
  // b ∈ [0.1, 5.0]
  theta_(0) = std::clamp(theta_(0), std::log(0.05), std::log(10.0));
  theta_(1) = std::clamp(theta_(1), 0.1, 5.0);

  // Covariance update with forgetting factor.
  // P ← (1/λ)(I − Kφᵀ)P
  // The (1/λ) inflation counteracts the (I − Kφᵀ) shrinkage so that the
  // effective memory stays at ~1/(1−λ) samples even in steady state.
  P_ = (1.0 / lambda_) * (Eigen::Matrix2d::Identity() - K * phi.transpose()) * P_;

  // Enforce symmetry: floating-point accumulation can make P slightly asymmetric.
  P_ = 0.5 * (P_ + P_.transpose());
}

// ── Diagnostics (1 Hz) ───────────────────────────────────────────────────────
// Publishes [k, b, P00, P11, trace(P), dvl_age_s] on ~/diagnostics.
// Plot in Foxglove (raw message panel or time-series) to watch the model
// converge: P should shrink as data accumulates, k and b should stabilise.
void AdaptiveThrusterEstimator::onDiagTimer()
{
  const double dvl_age_s = (last_dvl_wall_s_ > 0.0)
                           ? wallNow() - last_dvl_wall_s_
                           : -1.0;

  auto msg  = std_msgs::msg::Float64MultiArray{};
  msg.data  = std::vector<double>{
    std::exp(theta_(0)),   // k  (physical units, m/s)
    theta_(1),             // b  (exponent, dimensionless)
    P_(0, 0),              // var(ln k)
    P_(1, 1),              // var(b)
    P_.trace(),            // total parameter uncertainty
    dvl_age_s              // seconds since last DVL message (-1 = never seen)
  };
  diag_pub_->publish(msg);
}

// ── Publish odometry estimate ─────────────────────────────────────────────────
void AdaptiveThrusterEstimator::publishEstimate(
  double vx_hat, const rclcpp::Time & stamp)
{
  auto odom = nav_msgs::msg::Odometry{};
  odom.header.stamp    = stamp;
  odom.header.frame_id = "odom";
  odom.child_frame_id  = "base_link";

  odom.twist.twist.linear.x = vx_hat;
  // vy and vz remain zero: depth-hold survey assumption (vz ≈ 0, vy modelled as 0)

  // Diagonal twist covariance (row-major 6×6, angular block set large)
  odom.twist.covariance[0]  = fallback_cov_vx_;
  odom.twist.covariance[7]  = fallback_cov_vy_;
  odom.twist.covariance[14] = fallback_cov_vz_;
  odom.twist.covariance[21] = 99999.0;
  odom.twist.covariance[28] = 99999.0;
  odom.twist.covariance[35] = 99999.0;

  pub_->publish(odom);
}

// ── Entry point ───────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdaptiveThrusterEstimator>());
  rclcpp::shutdown();
  return 0;
}
