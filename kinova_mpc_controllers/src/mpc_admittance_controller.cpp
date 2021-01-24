//
// Created by giuseppe on 24.01.21.
//

#include "kinova_mpc_controllers/mpc_admittance_controller.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace kinova_controllers;

MPC_AdmittanceController::MPC_AdmittanceController(const ros::NodeHandle& nh)
    : MPC_Controller(nh), tf_listener_(tf_buffer_) {
  std::string wrench_topic;
  if (!nh.param<std::string>("wrench_topic", wrench_topic, "/wrench")) {
    ROS_WARN_STREAM("Failed to parse wrench topic, defaulting to /wrench");
  }

  if (!nh.param<std::string>("sensor_frame", sensor_frame_, "/sensor_frame")) {
    ROS_WARN_STREAM("Failed to parse sensor_frame, defaulting to /ft_sensor0");
  }

  std::vector<double> kp_linear_gains;
  if (!nh.param<std::vector<double>>("kp_linear_gains", kp_linear_gains, {0.0, 0.0, 0.0})) {
    ROS_WARN_STREAM("Failed to parse wrench topic, defaulting to [0 0 0]");
  }
  if (kp_linear_gains.size() != 3) {
    ROS_ERROR_STREAM("Gains have the wrong size! Defaulting to [0 0 0]");
    kp_linear_gains.clear();
    kp_linear_gains.resize(3, 0.0);
  }
  Kp_linear_.x() = kp_linear_gains[0];
  Kp_linear_.y() = kp_linear_gains[1];
  Kp_linear_.z() = kp_linear_gains[2];

  wrench_callback_queue_ = std::unique_ptr<ros::CallbackQueue>(new ros::CallbackQueue());
  ros::SubscribeOptions so;
  so.init<geometry_msgs::WrenchStamped>(
      wrench_topic, 1, boost::bind(&MPC_AdmittanceController::wrench_callback, this, _1));
  so.callback_queue = wrench_callback_queue_.get();
  wrench_subscriber_ = nh_.subscribe(so);

  wrench_offset_.setZero();
  reset_wrench_offset_service_ = nh_.advertiseService(
      "/reset_wrench_offset", &MPC_AdmittanceController::reset_wrench_offset_callback, this);

  desired_wrench_service_ = nh_.advertiseService(
      "/set_desired_wrench", &MPC_AdmittanceController::set_desired_wrench, this);
  active_ = false;
}

void MPC_AdmittanceController::adjustPath(nav_msgs::Path& desiredPath) const {
  wrench_callback_queue_->callAvailable();
  Eigen::Vector3d force(wrench_.wrench.force.x, wrench_.wrench.force.y, wrench_.wrench.force.z);
  force -= wrench_offset_.head<3>();
  Eigen::Vector3d delta_position = Kp_linear_.cwiseProduct(-force);

  // Transform delta in poses frame
  geometry_msgs::TransformStamped transform;
  try {
    // target_frame, source_frame ...
    transform =
        tf_buffer_.lookupTransform(desiredPath.header.frame_id, sensor_frame_, ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN_STREAM_THROTTLE(2.0, ex.what());
    return;
  }

  Eigen::Quaterniond q(transform.transform.rotation.w, transform.transform.rotation.x,
                       transform.transform.rotation.y, transform.transform.rotation.z);
  Eigen::Matrix3d R(q);
  Eigen::Vector3d delta_position_transformed = R * delta_position;
  if (active_) {
    ROS_INFO_STREAM_THROTTLE(2.0, "Adjusting path: " << std::endl
                                                     << "delta position (sensor frame) = "
                                                     << delta_position.transpose() << std::endl
                                                     << "delta position (base frame) = "
                                                     << delta_position_transformed.transpose());
    for (auto& pose : desiredPath.poses) {
      pose.pose.position.x += delta_position_transformed.x();
      pose.pose.position.y += delta_position_transformed.y();
      pose.pose.position.z += delta_position_transformed.z();
    }
  }
}

void MPC_AdmittanceController::wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg) {
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  wrench_ = *msg;
}

bool MPC_AdmittanceController::reset_wrench_offset_callback(std_srvs::EmptyRequest&,
                                                            std_srvs::EmptyResponse&) {
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  wrench_offset_(0) = wrench_.wrench.force.x;
  wrench_offset_(1) = wrench_.wrench.force.y;
  wrench_offset_(2) = wrench_.wrench.force.z;
  wrench_offset_(3) = wrench_.wrench.torque.x;
  wrench_offset_(4) = wrench_.wrench.torque.y;
  wrench_offset_(5) = wrench_.wrench.torque.z;

  ROS_INFO_STREAM("Wrench offset reset to: " << wrench_offset_.transpose());
  return true;
}

bool MPC_AdmittanceController::set_desired_wrench(std_srvs::EmptyRequest&,
                                                  std_srvs::EmptyResponse&) {
  ROS_WARN("This service is just triggering admittance mode with 0 reference wrench.");
  active_ = true;
  return true;
}