/**
 * visual_localization_node.cpp
 *
 * Redundant visual localization node that runs alongside LiDAR SLAM.
 * Uses RTAB-Map visual odometry to maintain a camera-based pose estimate.
 *
 * Operation modes:
 *   RELATIVE_ONLY  - Accumulates VO displacements, publishes vo_odom->camera_link TF
 *   ABSOLUTE_MODE  - After receiving absolute pose via service, also publishes
 *                    map->vo_odom TF; monitors LiDAR SLAM with dual-layer
 *                    jump/drift detection and VO health-gated auto-correction
 *
 * ADR-001 features:
 *   - VO health check (covariance + anchor age/distance + data freshness)
 *   - Dual-layer thresholds (jump detection → auto-correct, drift → alert only)
 *   - Diagnostic publishing (~/vo_health, ~/slam_deviation)
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <mutex>
#include <cmath>
#include <sstream>

#include <visual_localization/SetAbsolutePose.h>
#include <visual_localization/SetInitialPose.h>
#include <visual_localization/SlamDeviation.h>

class VisualLocalizationNode
{
public:
  VisualLocalizationNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , mode_(RELATIVE_ONLY)
    , vo_received_(false)
    , slam_pose_received_(false)
    , jump_consecutive_count_(0)
    , drift_consecutive_count_(0)
    , T_vo_current_(Eigen::Isometry3d::Identity())
    , T_absolute_anchor_(Eigen::Isometry3d::Identity())
    , T_vo_at_anchor_(Eigen::Isometry3d::Identity())
    , T_map_to_odom_(Eigen::Isometry3d::Identity())
    , accumulated_distance_(0.0)
    , T_vo_previous_(Eigen::Isometry3d::Identity())
    , vo_position_variance_(0.0)
    , vo_rotation_variance_(0.0)
  {
    loadParameters();
    setupROS();
    ROS_INFO("[VisualLoc] Node initialized in RELATIVE_ONLY mode");
    ROS_INFO("[VisualLoc] Waiting for VO data on: %s", vo_odom_topic_.c_str());
  }

private:
  enum Mode { RELATIVE_ONLY, ABSOLUTE_MODE };

  // =========================================================================
  //  Parameter Loading
  // =========================================================================
  void loadParameters()
  {
    // Topics & frames
    pnh_.param<std::string>("vo_odom_topic", vo_odom_topic_, "/vo/odom");
    pnh_.param<std::string>("slam_pose_topic", slam_pose_topic_, "/slam_pose");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("odom_frame", odom_frame_, "vo_odom");
    pnh_.param<std::string>("camera_frame", camera_frame_, "camera_link");

    // SLAM correction
    pnh_.param<std::string>("slam_set_pose_service", slam_set_pose_service_, "/set_pose");
    pnh_.param<double>("cooldown_duration", cooldown_duration_, 30.0);
    pnh_.param<double>("comparison_rate", comparison_rate_, 2.0);

    // Dual-layer thresholds
    pnh_.param<double>("jump_translation_threshold", jump_translation_threshold_, 3.0);
    pnh_.param<double>("jump_rotation_threshold", jump_rotation_threshold_, 1.0);
    pnh_.param<int>("jump_consecutive_threshold", jump_consecutive_threshold_, 2);
    pnh_.param<double>("drift_translation_threshold", drift_translation_threshold_, 1.0);
    pnh_.param<double>("drift_rotation_threshold", drift_rotation_threshold_, 0.5);
    pnh_.param<int>("drift_consecutive_threshold", drift_consecutive_threshold_, 10);

    // VO health check
    pnh_.param<double>("vo_max_covariance_position", vo_max_cov_pos_, 0.5);
    pnh_.param<double>("vo_max_covariance_rotation", vo_max_cov_rot_, 0.3);
    pnh_.param<double>("vo_max_age", vo_max_age_, 0.5);
    pnh_.param<double>("vo_max_trust_duration", vo_max_trust_duration_, 120.0);
    pnh_.param<double>("vo_max_trust_distance", vo_max_trust_distance_, 50.0);

    // Publishing rates
    pnh_.param<double>("tf_publish_rate", tf_publish_rate_, 30.0);
    pnh_.param<double>("pose_publish_rate", pose_publish_rate_, 10.0);
  }

  // =========================================================================
  //  ROS Setup
  // =========================================================================
  void setupROS()
  {
    // Subscribers
    vo_sub_ = nh_.subscribe(vo_odom_topic_, 10,
                            &VisualLocalizationNode::voOdomCallback, this);
    slam_sub_ = nh_.subscribe(slam_pose_topic_, 10,
                              &VisualLocalizationNode::slamPoseCallback, this);

    // Service server
    set_absolute_pose_srv_ = pnh_.advertiseService(
        "set_absolute_pose", &VisualLocalizationNode::setAbsolutePoseCallback, this);

    // Service client
    slam_set_pose_client_ = nh_.serviceClient<visual_localization::SetInitialPose>(
        slam_set_pose_service_);

    // Publishers
    pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("pose", 10);
    vo_health_pub_ = pnh_.advertise<diagnostic_msgs::DiagnosticStatus>("vo_health", 10);
    slam_deviation_pub_ = pnh_.advertise<visual_localization::SlamDeviation>("slam_deviation", 10);

    // Timers
    tf_timer_ = nh_.createTimer(
        ros::Duration(1.0 / tf_publish_rate_),
        &VisualLocalizationNode::tfPublishCallback, this);
    comparison_timer_ = nh_.createTimer(
        ros::Duration(1.0 / comparison_rate_),
        &VisualLocalizationNode::comparisonCallback, this);

    last_correction_time_ = ros::Time(0);
  }

  // =========================================================================
  //  VO Odometry Callback
  // =========================================================================
  void voOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(msg->pose.pose, pose);

    // Accumulate travel distance from anchor for health check
    if (vo_received_ && mode_ == ABSOLUTE_MODE)
    {
      double step = (pose.translation() - T_vo_previous_.translation()).norm();
      accumulated_distance_ += step;
    }
    T_vo_previous_ = pose;

    T_vo_current_ = pose;
    vo_received_ = true;
    last_vo_stamp_ = msg->header.stamp;

    // Extract covariance for health check (diagonal: x, y, z, roll, pitch, yaw)
    const auto& cov = msg->pose.covariance;
    vo_position_variance_ = std::max({cov[0], cov[7], cov[14]});
    vo_rotation_variance_ = std::max({cov[21], cov[28], cov[35]});
  }

  // =========================================================================
  //  SLAM Pose Callback
  // =========================================================================
  void slamPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(slam_mutex_);
    latest_slam_pose_ = *msg;
    slam_pose_received_ = true;
  }

  // =========================================================================
  //  Set Absolute Pose Service
  // =========================================================================
  bool setAbsolutePoseCallback(visual_localization::SetAbsolutePose::Request& req,
                               visual_localization::SetAbsolutePose::Response& res)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!vo_received_)
    {
      res.success = false;
      res.message = "Cannot set absolute pose: no VO data received yet";
      ROS_WARN("[VisualLoc] %s", res.message.c_str());
      return true;
    }

    // Store anchor
    tf::poseMsgToEigen(req.pose.pose, T_absolute_anchor_);
    T_vo_at_anchor_ = T_vo_current_;
    T_map_to_odom_ = T_absolute_anchor_ * T_vo_at_anchor_.inverse();

    // Reset health tracking
    anchor_time_ = ros::Time::now();
    accumulated_distance_ = 0.0;

    mode_ = ABSOLUTE_MODE;
    jump_consecutive_count_ = 0;
    drift_consecutive_count_ = 0;

    res.success = true;
    res.message = "Absolute pose set. Now in ABSOLUTE_MODE.";
    ROS_INFO("[VisualLoc] Anchor set at (%.3f, %.3f, %.3f). ABSOLUTE_MODE active.",
             T_absolute_anchor_.translation().x(),
             T_absolute_anchor_.translation().y(),
             T_absolute_anchor_.translation().z());
    return true;
  }

  // =========================================================================
  //  VO Health Evaluation
  // =========================================================================
  struct VoHealthResult
  {
    bool healthy;
    bool covariance_ok;
    bool anchor_time_ok;
    bool anchor_distance_ok;
    bool data_fresh;
    double position_variance;
    double rotation_variance;
    double anchor_elapsed;
    double travel_distance;
    double data_age;
  };

  VoHealthResult evaluateVoHealth(const ros::Time& now) const
  {
    VoHealthResult h;

    // 1. Covariance check
    h.position_variance = vo_position_variance_;
    h.rotation_variance = vo_rotation_variance_;
    h.covariance_ok = (h.position_variance <= vo_max_cov_pos_) &&
                      (h.rotation_variance <= vo_max_cov_rot_);

    // 2. Anchor time check
    h.anchor_elapsed = (now - anchor_time_).toSec();
    h.anchor_time_ok = (h.anchor_elapsed <= vo_max_trust_duration_);

    // 3. Anchor distance check
    h.travel_distance = accumulated_distance_;
    h.anchor_distance_ok = (h.travel_distance <= vo_max_trust_distance_);

    // 4. Data freshness check
    h.data_age = (now - last_vo_stamp_).toSec();
    h.data_fresh = (h.data_age <= vo_max_age_);

    h.healthy = h.covariance_ok && h.anchor_time_ok &&
                h.anchor_distance_ok && h.data_fresh;
    return h;
  }

  void publishVoHealth(const VoHealthResult& h)
  {
    diagnostic_msgs::DiagnosticStatus status;
    status.name = "visual_localization/vo_health";
    status.hardware_id = "visual_odometry";

    if (!vo_received_)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::STALE;
      status.message = "No VO data received";
    }
    else if (mode_ == RELATIVE_ONLY)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "RELATIVE_ONLY mode - health check not applicable";
    }
    else if (h.healthy)
    {
      status.level = diagnostic_msgs::DiagnosticStatus::OK;
      status.message = "VO healthy";
    }
    else
    {
      status.level = diagnostic_msgs::DiagnosticStatus::WARN;
      std::string reason;
      if (!h.covariance_ok)
        reason += "covariance_exceeded ";
      if (!h.anchor_time_ok)
        reason += "anchor_expired ";
      if (!h.anchor_distance_ok)
        reason += "distance_exceeded ";
      if (!h.data_fresh)
        reason += "data_stale ";
      status.message = "VO degraded: " + reason;
    }

    auto kv = [](const std::string& key, double val) {
      diagnostic_msgs::KeyValue kvp;
      kvp.key = key;
      std::ostringstream oss;
      oss << std::fixed << std::setprecision(3) << val;
      kvp.value = oss.str();
      return kvp;
    };

    status.values.push_back(kv("position_variance", h.position_variance));
    status.values.push_back(kv("rotation_variance", h.rotation_variance));
    status.values.push_back(kv("anchor_elapsed_s", h.anchor_elapsed));
    status.values.push_back(kv("travel_distance_m", h.travel_distance));
    status.values.push_back(kv("data_age_s", h.data_age));

    vo_health_pub_.publish(status);
  }

  // =========================================================================
  //  TF & Pose Publishing
  // =========================================================================
  void tfPublishCallback(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!vo_received_)
      return;

    ros::Time stamp = last_vo_stamp_;

    // Always: vo_odom -> camera_link
    publishTransform(T_vo_current_, odom_frame_, camera_frame_, stamp);

    // ABSOLUTE_MODE: map -> vo_odom
    if (mode_ == ABSOLUTE_MODE)
    {
      publishTransform(T_map_to_odom_, map_frame_, odom_frame_, stamp);

      Eigen::Isometry3d T_absolute = T_map_to_odom_ * T_vo_current_;
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.stamp = stamp;
      pose_msg.header.frame_id = map_frame_;
      tf::poseEigenToMsg(T_absolute, pose_msg.pose);
      pose_pub_.publish(pose_msg);
    }
  }

  void publishTransform(const Eigen::Isometry3d& transform,
                         const std::string& parent_frame,
                         const std::string& child_frame,
                         const ros::Time& stamp)
  {
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform = tf2::eigenToTransform(transform).transform;
    tf_broadcaster_.sendTransform(tf_msg);
  }

  // =========================================================================
  //  Dual-Layer SLAM Comparison & Correction
  // =========================================================================
  void comparisonCallback(const ros::TimerEvent&)
  {
    ros::Time now = ros::Time::now();
    Eigen::Isometry3d visual_absolute;
    VoHealthResult health;

    // --- Gather VO state ---
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mode_ != ABSOLUTE_MODE || !vo_received_)
      {
        // Still publish health even if not in absolute mode
        if (vo_received_)
        {
          health = evaluateVoHealth(now);
          publishVoHealth(health);
        }
        return;
      }
      visual_absolute = T_map_to_odom_ * T_vo_current_;
      health = evaluateVoHealth(now);
    }

    publishVoHealth(health);

    // --- Gather SLAM state ---
    geometry_msgs::PoseStamped slam_pose;
    {
      std::lock_guard<std::mutex> lock(slam_mutex_);
      if (!slam_pose_received_)
        return;
      slam_pose = latest_slam_pose_;
    }

    // --- Compute difference ---
    Eigen::Isometry3d T_slam;
    tf::poseMsgToEigen(slam_pose.pose, T_slam);
    Eigen::Isometry3d diff = T_slam.inverse() * visual_absolute;

    double trans_error = diff.translation().norm();
    Eigen::AngleAxisd angle_axis(diff.rotation());
    double rot_error = std::abs(angle_axis.angle());
    if (rot_error > M_PI)
      rot_error = 2.0 * M_PI - rot_error;

    // --- Layer 1: Jump detection ---
    bool is_jump = (trans_error > jump_translation_threshold_) ||
                   (rot_error > jump_rotation_threshold_);

    // --- Layer 2: Drift detection ---
    bool is_drift = !is_jump &&
                    ((trans_error > drift_translation_threshold_) ||
                     (rot_error > drift_rotation_threshold_));

    // --- Update consecutive counters ---
    if (is_jump)
    {
      jump_consecutive_count_++;
      drift_consecutive_count_ = 0;
    }
    else if (is_drift)
    {
      drift_consecutive_count_++;
      jump_consecutive_count_ = 0;
    }
    else
    {
      // Within normal range — reset both
      if (jump_consecutive_count_ > 0 || drift_consecutive_count_ > 0)
      {
        ROS_INFO("[VisualLoc] Deviation returned to normal. Counters reset.");
      }
      jump_consecutive_count_ = 0;
      drift_consecutive_count_ = 0;
      publishSlamDeviation(trans_error, rot_error, health.healthy,
                           visual_localization::SlamDeviation::LEVEL_NORMAL, false);
      return;
    }

    // --- Layer 1: Jump — attempt auto-correction if VO healthy ---
    if (is_jump && jump_consecutive_count_ >= jump_consecutive_threshold_)
    {
      ROS_WARN("[VisualLoc] JUMP detected: trans=%.3fm rot=%.3frad (consecutive=%d)",
               trans_error, rot_error, jump_consecutive_count_);

      if (health.healthy)
      {
        bool corrected = attemptSlamCorrection(visual_absolute, now);
        publishSlamDeviation(trans_error, rot_error, true,
                             corrected ? visual_localization::SlamDeviation::LEVEL_CORRECTED
                                       : visual_localization::SlamDeviation::LEVEL_JUMP_DETECTED,
                             corrected);
      }
      else
      {
        ROS_WARN("[VisualLoc] JUMP detected but VO unhealthy — correction blocked. "
                 "pos_var=%.3f rot_var=%.3f anchor_age=%.1fs travel=%.1fm data_age=%.3fs",
                 health.position_variance, health.rotation_variance,
                 health.anchor_elapsed, health.travel_distance, health.data_age);
        publishSlamDeviation(trans_error, rot_error, false,
                             visual_localization::SlamDeviation::LEVEL_JUMP_DETECTED, false);
      }
      jump_consecutive_count_ = 0;
      return;
    }

    // --- Layer 2: Drift — alert only, never auto-correct ---
    if (is_drift && drift_consecutive_count_ >= drift_consecutive_threshold_)
    {
      ROS_WARN("[VisualLoc] DRIFT warning: trans=%.3fm rot=%.3frad (consecutive=%d). "
               "No auto-correction — external decision required.",
               trans_error, rot_error, drift_consecutive_count_);
      publishSlamDeviation(trans_error, rot_error, health.healthy,
                           visual_localization::SlamDeviation::LEVEL_DRIFT_WARNING, false);
      drift_consecutive_count_ = 0;
      return;
    }

    // --- Thresholds exceeded but consecutive count not yet met ---
    uint8_t pending_level = is_jump
        ? visual_localization::SlamDeviation::LEVEL_JUMP_DETECTED
        : visual_localization::SlamDeviation::LEVEL_DRIFT_WARNING;
    publishSlamDeviation(trans_error, rot_error, health.healthy, pending_level, false);

    if (is_jump)
    {
      ROS_WARN_THROTTLE(5.0, "[VisualLoc] Jump pending: trans=%.3fm rot=%.3frad "
                              "consecutive=%d/%d",
                        trans_error, rot_error,
                        jump_consecutive_count_, jump_consecutive_threshold_);
    }
    else
    {
      ROS_WARN_THROTTLE(5.0, "[VisualLoc] Drift pending: trans=%.3fm rot=%.3frad "
                              "consecutive=%d/%d",
                        trans_error, rot_error,
                        drift_consecutive_count_, drift_consecutive_threshold_);
    }
  }

  // =========================================================================
  //  SLAM Correction
  // =========================================================================
  bool attemptSlamCorrection(const Eigen::Isometry3d& corrected_pose,
                              const ros::Time& now)
  {
    // Cooldown check
    double elapsed = (now - last_correction_time_).toSec();
    if (last_correction_time_ != ros::Time(0) && elapsed < cooldown_duration_)
    {
      ROS_WARN("[VisualLoc] Correction skipped: cooldown active (%.1fs remaining)",
               cooldown_duration_ - elapsed);
      return false;
    }

    // Build service request
    visual_localization::SetInitialPose srv;
    srv.request.pose.header.stamp = now;
    srv.request.pose.header.frame_id = map_frame_;
    tf::poseEigenToMsg(corrected_pose, srv.request.pose.pose.pose);

    // Default covariance
    for (int i = 0; i < 36; ++i)
      srv.request.pose.pose.covariance[i] = 0.0;
    srv.request.pose.pose.covariance[0]  = 0.25;  // x
    srv.request.pose.pose.covariance[7]  = 0.25;  // y
    srv.request.pose.pose.covariance[14] = 0.25;  // z
    srv.request.pose.pose.covariance[21] = 0.07;  // roll
    srv.request.pose.pose.covariance[28] = 0.07;  // pitch
    srv.request.pose.pose.covariance[35] = 0.07;  // yaw

    if (slam_set_pose_client_.call(srv))
    {
      if (srv.response.success)
      {
        ROS_INFO("[VisualLoc] SLAM correction sent successfully: %s",
                 srv.response.message.c_str());
        last_correction_time_ = now;
        return true;
      }
      ROS_WARN("[VisualLoc] SLAM correction service returned failure: %s",
               srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("[VisualLoc] Failed to call SLAM set_pose service: %s",
                slam_set_pose_service_.c_str());
    }

    last_correction_time_ = now;
    return false;
  }

  // =========================================================================
  //  Diagnostic Publishing
  // =========================================================================
  void publishSlamDeviation(double trans_error, double rot_error,
                             bool vo_healthy, uint8_t level, bool correction_attempted)
  {
    visual_localization::SlamDeviation msg;
    msg.header.stamp = ros::Time::now();
    msg.translation_error = trans_error;
    msg.rotation_error = rot_error;
    msg.level = level;
    msg.vo_healthy = vo_healthy;
    msg.correction_attempted = correction_attempted;
    slam_deviation_pub_.publish(msg);
  }

  // =========================================================================
  //  Member Variables
  // =========================================================================
  ros::NodeHandle nh_, pnh_;
  Mode mode_;

  // VO state (protected by mutex_)
  bool vo_received_;
  Eigen::Isometry3d T_vo_current_;
  Eigen::Isometry3d T_vo_previous_;
  ros::Time last_vo_stamp_;
  double vo_position_variance_;
  double vo_rotation_variance_;

  // Absolute reference state (protected by mutex_)
  Eigen::Isometry3d T_absolute_anchor_;
  Eigen::Isometry3d T_vo_at_anchor_;
  Eigen::Isometry3d T_map_to_odom_;
  ros::Time anchor_time_;
  double accumulated_distance_;

  // SLAM comparison state
  bool slam_pose_received_;
  geometry_msgs::PoseStamped latest_slam_pose_;
  int jump_consecutive_count_;
  int drift_consecutive_count_;
  ros::Time last_correction_time_;

  // Parameters: topics & frames
  std::string vo_odom_topic_;
  std::string slam_pose_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string camera_frame_;

  // Parameters: SLAM correction
  std::string slam_set_pose_service_;
  double cooldown_duration_;
  double comparison_rate_;

  // Parameters: dual-layer thresholds
  double jump_translation_threshold_;
  double jump_rotation_threshold_;
  int jump_consecutive_threshold_;
  double drift_translation_threshold_;
  double drift_rotation_threshold_;
  int drift_consecutive_threshold_;

  // Parameters: VO health
  double vo_max_cov_pos_;
  double vo_max_cov_rot_;
  double vo_max_age_;
  double vo_max_trust_duration_;
  double vo_max_trust_distance_;

  // Parameters: publishing
  double tf_publish_rate_;
  double pose_publish_rate_;

  // ROS interfaces
  ros::Subscriber vo_sub_;
  ros::Subscriber slam_sub_;
  ros::ServiceServer set_absolute_pose_srv_;
  ros::ServiceClient slam_set_pose_client_;
  ros::Publisher pose_pub_;
  ros::Publisher vo_health_pub_;
  ros::Publisher slam_deviation_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Timer tf_timer_;
  ros::Timer comparison_timer_;

  // Thread safety
  std::mutex mutex_;       // protects VO state, mode, absolute reference
  std::mutex slam_mutex_;  // protects SLAM pose state
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_localization_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  VisualLocalizationNode node(nh, pnh);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
