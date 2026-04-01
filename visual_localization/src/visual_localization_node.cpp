/**
 * visual_localization_node.cpp
 *
 * Redundant visual localization node that runs alongside LiDAR SLAM.
 * Uses RTAB-Map visual odometry to maintain a camera-based pose estimate.
 *
 * Operation modes:
 *   RELATIVE_ONLY  - Accumulates VO displacements from startup, publishes odom->camera_link
 *   ABSOLUTE_MODE  - After receiving absolute pose via service, also publishes map->odom
 *                    and monitors LiDAR SLAM for drift correction
 */

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Geometry>
#include <mutex>
#include <cmath>

#include <visual_localization/SetAbsolutePose.h>
#include <visual_localization/SetInitialPose.h>

class VisualLocalizationNode
{
public:
  VisualLocalizationNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh)
    , pnh_(pnh)
    , mode_(RELATIVE_ONLY)
    , vo_received_(false)
    , slam_pose_received_(false)
    , consecutive_exceed_count_(0)
    , T_vo_current_(Eigen::Isometry3d::Identity())
    , T_absolute_anchor_(Eigen::Isometry3d::Identity())
    , T_vo_at_anchor_(Eigen::Isometry3d::Identity())
    , T_map_to_odom_(Eigen::Isometry3d::Identity())
  {
    loadParameters();
    setupROS();
    ROS_INFO("[VisualLoc] Node initialized in RELATIVE_ONLY mode");
    ROS_INFO("[VisualLoc] Waiting for VO data on: %s", vo_odom_topic_.c_str());
  }

private:
  enum Mode { RELATIVE_ONLY, ABSOLUTE_MODE };

  // --- Parameter Loading ---
  void loadParameters()
  {
    pnh_.param<std::string>("vo_odom_topic", vo_odom_topic_, "/vo/odom");
    pnh_.param<std::string>("slam_pose_topic", slam_pose_topic_, "/slam_pose");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("odom_frame", odom_frame_, "vo_odom");
    pnh_.param<std::string>("camera_frame", camera_frame_, "camera_link");
    pnh_.param<std::string>("slam_set_pose_service", slam_set_pose_service_, "/set_pose");
    pnh_.param<double>("translation_threshold", translation_threshold_, 0.5);
    pnh_.param<double>("rotation_threshold", rotation_threshold_, 0.3);
    pnh_.param<int>("consecutive_threshold", consecutive_threshold_, 5);
    pnh_.param<double>("comparison_rate", comparison_rate_, 2.0);
    pnh_.param<double>("cooldown_duration", cooldown_duration_, 30.0);
    pnh_.param<double>("tf_publish_rate", tf_publish_rate_, 30.0);
    pnh_.param<double>("pose_publish_rate", pose_publish_rate_, 10.0);
  }

  // --- ROS Setup ---
  void setupROS()
  {
    // Subscribers
    vo_sub_ = nh_.subscribe(vo_odom_topic_, 10,
                            &VisualLocalizationNode::voOdomCallback, this);
    slam_sub_ = nh_.subscribe(slam_pose_topic_, 10,
                              &VisualLocalizationNode::slamPoseCallback, this);

    // Service server: receive absolute pose from external
    set_absolute_pose_srv_ = pnh_.advertiseService(
        "set_absolute_pose", &VisualLocalizationNode::setAbsolutePoseCallback, this);

    // Service client: send initialpose to LiDAR SLAM
    slam_set_pose_client_ = nh_.serviceClient<visual_localization::SetInitialPose>(
        slam_set_pose_service_);

    // Publishers
    pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("pose", 10);

    // Timers
    tf_timer_ = nh_.createTimer(
        ros::Duration(1.0 / tf_publish_rate_),
        &VisualLocalizationNode::tfPublishCallback, this);
    comparison_timer_ = nh_.createTimer(
        ros::Duration(1.0 / comparison_rate_),
        &VisualLocalizationNode::comparisonCallback, this);

    // Initialize cooldown time to allow immediate first correction
    last_correction_time_ = ros::Time(0);
  }

  // --- VO Odometry Callback ---
  void voOdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    Eigen::Isometry3d pose;
    tf::poseMsgToEigen(msg->pose.pose, pose);
    T_vo_current_ = pose;
    vo_received_ = true;
    last_vo_stamp_ = msg->header.stamp;
  }

  // --- SLAM Pose Callback ---
  void slamPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(slam_mutex_);
    latest_slam_pose_ = *msg;
    slam_pose_received_ = true;
  }

  // --- Set Absolute Pose Service ---
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

    // Store the absolute anchor and the VO pose at this moment
    tf::poseMsgToEigen(req.pose.pose, T_absolute_anchor_);
    T_vo_at_anchor_ = T_vo_current_;

    // Compute map -> odom transform: T_map_odom = T_abs_anchor * inv(T_vo_anchor)
    T_map_to_odom_ = T_absolute_anchor_ * T_vo_at_anchor_.inverse();

    mode_ = ABSOLUTE_MODE;
    consecutive_exceed_count_ = 0;

    res.success = true;
    res.message = "Absolute pose set successfully. Now in ABSOLUTE_MODE.";
    ROS_INFO("[VisualLoc] Absolute pose anchor set. Transitioning to ABSOLUTE_MODE.");
    ROS_INFO("[VisualLoc]   Anchor position: (%.3f, %.3f, %.3f)",
             T_absolute_anchor_.translation().x(),
             T_absolute_anchor_.translation().y(),
             T_absolute_anchor_.translation().z());
    return true;
  }

  // --- TF & Pose Publishing ---
  void tfPublishCallback(const ros::TimerEvent&)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!vo_received_)
      return;

    ros::Time stamp = last_vo_stamp_;

    // Always publish: odom_frame -> camera_frame (relative VO)
    publishTransform(T_vo_current_, odom_frame_, camera_frame_, stamp);

    // In ABSOLUTE_MODE: also publish map_frame -> odom_frame
    if (mode_ == ABSOLUTE_MODE)
    {
      publishTransform(T_map_to_odom_, map_frame_, odom_frame_, stamp);

      // Publish absolute pose topic
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

  // --- SLAM Pose Comparison & Correction ---
  void comparisonCallback(const ros::TimerEvent&)
  {
    Eigen::Isometry3d visual_absolute;
    geometry_msgs::PoseStamped slam_pose;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (mode_ != ABSOLUTE_MODE || !vo_received_)
        return;
      visual_absolute = T_map_to_odom_ * T_vo_current_;
    }

    {
      std::lock_guard<std::mutex> lock(slam_mutex_);
      if (!slam_pose_received_)
        return;
      slam_pose = latest_slam_pose_;
    }

    // Convert SLAM pose to Eigen
    Eigen::Isometry3d T_slam;
    tf::poseMsgToEigen(slam_pose.pose, T_slam);

    // Compute difference
    Eigen::Isometry3d diff = T_slam.inverse() * visual_absolute;
    double trans_error = diff.translation().norm();

    // Extract rotation angle difference
    Eigen::AngleAxisd angle_axis(diff.rotation());
    double rot_error = std::abs(angle_axis.angle());
    // Normalize to [0, pi]
    if (rot_error > M_PI)
      rot_error = 2.0 * M_PI - rot_error;

    bool exceeded = (trans_error > translation_threshold_) ||
                    (rot_error > rotation_threshold_);

    if (exceeded)
    {
      consecutive_exceed_count_++;
      ROS_WARN_THROTTLE(5.0, "[VisualLoc] Pose deviation detected: "
                              "trans=%.3fm (thresh=%.3f), rot=%.3frad (thresh=%.3f), "
                              "consecutive=%d/%d",
                        trans_error, translation_threshold_,
                        rot_error, rotation_threshold_,
                        consecutive_exceed_count_, consecutive_threshold_);

      if (consecutive_exceed_count_ >= consecutive_threshold_)
      {
        attemptSlamCorrection(visual_absolute);
      }
    }
    else
    {
      if (consecutive_exceed_count_ > 0)
      {
        ROS_INFO("[VisualLoc] Pose deviation returned within threshold. Reset counter.");
      }
      consecutive_exceed_count_ = 0;
    }
  }

  void attemptSlamCorrection(const Eigen::Isometry3d& corrected_pose)
  {
    // Check cooldown
    ros::Time now = ros::Time::now();
    double elapsed = (now - last_correction_time_).toSec();
    if (elapsed < cooldown_duration_)
    {
      ROS_WARN("[VisualLoc] Correction skipped: cooldown active (%.1fs remaining)",
               cooldown_duration_ - elapsed);
      consecutive_exceed_count_ = 0;
      return;
    }

    // Build service request
    visual_localization::SetInitialPose srv;
    srv.request.pose.header.stamp = ros::Time::now();
    srv.request.pose.header.frame_id = map_frame_;
    tf::poseEigenToMsg(corrected_pose, srv.request.pose.pose.pose);
    // Set a reasonable default covariance (diagonal)
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
      }
      else
      {
        ROS_WARN("[VisualLoc] SLAM correction service returned failure: %s",
                 srv.response.message.c_str());
      }
    }
    else
    {
      ROS_ERROR("[VisualLoc] Failed to call SLAM set_pose service: %s",
                slam_set_pose_service_.c_str());
    }

    last_correction_time_ = now;
    consecutive_exceed_count_ = 0;
  }

  // --- Member Variables ---
  ros::NodeHandle nh_, pnh_;
  Mode mode_;

  // VO state
  bool vo_received_;
  Eigen::Isometry3d T_vo_current_;
  ros::Time last_vo_stamp_;

  // Absolute reference state
  Eigen::Isometry3d T_absolute_anchor_;
  Eigen::Isometry3d T_vo_at_anchor_;
  Eigen::Isometry3d T_map_to_odom_;

  // SLAM comparison state
  bool slam_pose_received_;
  geometry_msgs::PoseStamped latest_slam_pose_;
  int consecutive_exceed_count_;
  ros::Time last_correction_time_;

  // Parameters
  std::string vo_odom_topic_;
  std::string slam_pose_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string camera_frame_;
  std::string slam_set_pose_service_;
  double translation_threshold_;
  double rotation_threshold_;
  int consecutive_threshold_;
  double comparison_rate_;
  double cooldown_duration_;
  double tf_publish_rate_;
  double pose_publish_rate_;

  // ROS interfaces
  ros::Subscriber vo_sub_;
  ros::Subscriber slam_sub_;
  ros::ServiceServer set_absolute_pose_srv_;
  ros::ServiceClient slam_set_pose_client_;
  ros::Publisher pose_pub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  ros::Timer tf_timer_;
  ros::Timer comparison_timer_;

  // Thread safety
  std::mutex mutex_;       // protects VO and mode state
  std::mutex slam_mutex_;  // protects SLAM pose state
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "visual_localization_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  VisualLocalizationNode node(nh, pnh);

  ros::AsyncSpinner spinner(2);  // 2 threads for callbacks
  spinner.start();
  ros::waitForShutdown();

  return 0;
}
