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

  parse_vector<3>(nh_, "kp_linear_gains", Kp_linear_);
  parse_vector<3>(nh_, "kp_angular_gains", Kp_angular_);
  parse_vector<3>(nh_, "ki_linear_gains", Ki_linear_);
  parse_vector<3>(nh_, "ki_angular_gains", Ki_angular_);
  parse_vector<3>(nh_, "force_integral_max", force_integral_max_);
  parse_vector<3>(nh_, "torque_integral_max", torque_integral_max_);

  if (!nh.param<double>("payload_mass", payload_mass_, 0.0)) {
    ROS_WARN_STREAM("Failed to parse payload_mass, defaulting to 0");
  }
  parse_vector<3>(nh_, "payload_offset", payload_offset_);

  wrench_callback_queue_ = std::unique_ptr<ros::CallbackQueue>(new ros::CallbackQueue());
  ros::SubscribeOptions so;
  so.init<geometry_msgs::WrenchStamped>(
      wrench_topic, 1, boost::bind(&MPC_AdmittanceController::wrench_callback, this, _1));
  so.callback_queue = wrench_callback_queue_.get();
  wrench_subscriber_ = nh_.subscribe(so);

  reset_wrench_offset_service_ = nh_.advertiseService(
      "/reset_wrench_offset", &MPC_AdmittanceController::reset_wrench_offset_callback, this);

  desired_wrench_service_ = nh_.advertiseService(
      "/set_desired_wrench", &MPC_AdmittanceController::set_desired_wrench, this);
  active_ = false;
  force_integral_.setZero();
  torque_integral_.setZero();
  last_time_ = -1;
}

void MPC_AdmittanceController::adjustPath(nav_msgs::Path& desiredPath) const {
  // Update time interval for integral computation
  double dt;
  if (last_time_ < 0) {
    dt = 0;
    last_time_ = ros::Time::now().toSec();
  } else {
    dt = ros::Time::now().toSec() - last_time_;
    last_time_ = ros::Time::now().toSec();
  }

  // Update sensor pose in a base frame
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
  Eigen::Vector3d gravity_sensor_frame = R.transpose() * Eigen::Vector3d::UnitX() * -9.81;

  // Update bias
  bias_ = ft_sensor_utils::get_bias(payload_mass_, payload_offset_, gravity_sensor_frame);

  // Update measured wrench and tracking errors
  wrench_callback_queue_->callAvailable();
  force_error_ =
      -(Eigen::Vector3d(wrench_.wrench.force.x, wrench_.wrench.force.y, wrench_.wrench.force.z) -
        bias_.get_force());
  force_integral_ += force_error_ * dt;
  force_integral_ = force_integral_.cwiseMax(-force_integral_max_);
  force_integral_ = force_integral_.cwiseMin(force_integral_max_);
  Eigen::Vector3d delta_position =
      Kp_linear_.cwiseProduct(force_error_) + Ki_linear_.cwiseProduct(force_integral_);
  Eigen::Vector3d delta_position_transformed = R * delta_position;

  torque_error_ =
      -(Eigen::Vector3d(wrench_.wrench.torque.x, wrench_.wrench.torque.y, wrench_.wrench.torque.z) -
        bias_.get_torque());
  torque_integral_ += torque_error_ * dt;
  torque_integral_ = torque_integral_.cwiseMax(-torque_integral_max_);
  torque_integral_ = torque_integral_.cwiseMin(torque_integral_max_);
  Eigen::Vector3d rotationImg =
      Kp_angular_.cwiseProduct(torque_error_) + Ki_angular_.cwiseProduct(torque_integral_);
  Eigen::Quaterniond delta_rotation;
  double r = rotationImg.norm();
  if (r > 0) {
    delta_rotation.w() = std::cos(r);
    Eigen::Vector3d quatImg = std::sin(r) / r * rotationImg;
    delta_rotation.x() = quatImg[0];
    delta_rotation.y() = quatImg[1];
    delta_rotation.z() = quatImg[2];
  }

  if (active_) {
    ROS_INFO_STREAM_THROTTLE(
        2.0, "Adjusting path: " << std::endl
                                << "delta position (sensor frame) = " << delta_position.transpose()
                                << std::endl
                                << "delta position (base frame) = "
                                << delta_position_transformed.transpose()
                                << "delta rotation norm (deg): = " << r * 180.0 / M_PI);
    for (auto& pose : desiredPath.poses) {
      pose.pose.position.x += delta_position_transformed.x();
      pose.pose.position.y += delta_position_transformed.y();
      pose.pose.position.z += delta_position_transformed.z();
      Eigen::Quaterniond new_orientation(pose.pose.orientation.w, pose.pose.orientation.x,
                                         pose.pose.orientation.y, pose.pose.orientation.z);
      new_orientation = new_orientation * delta_rotation;
      pose.pose.orientation.x = new_orientation.x();
      pose.pose.orientation.y = new_orientation.y();
      pose.pose.orientation.z = new_orientation.z();
      pose.pose.orientation.w = new_orientation.w();
    }
  }
}

void MPC_AdmittanceController::wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg) {
  std::lock_guard<std::mutex> lock(wrench_mutex_);
  wrench_ = *msg;
}

bool MPC_AdmittanceController::reset_wrench_offset_callback(std_srvs::EmptyRequest&,
                                                            std_srvs::EmptyResponse&) {
  wrench_callback_queue_->callAvailable();
  bias_.get_force().x() = wrench_.wrench.force.x;
  bias_.get_force().y() = wrench_.wrench.force.y;
  bias_.get_force().z() = wrench_.wrench.force.z;
  bias_.get_torque().x() = wrench_.wrench.torque.x;
  bias_.get_torque().y() = wrench_.wrench.torque.y;
  bias_.get_torque().z() = wrench_.wrench.torque.z;

  ROS_INFO_STREAM("Wrench offset reset to: " << bias_);
  return true;
}

bool MPC_AdmittanceController::set_desired_wrench(std_srvs::EmptyRequest&,
                                                  std_srvs::EmptyResponse&) {
  ROS_WARN("This service is just triggering admittance mode with 0 reference wrench.");
  wrench_callback_queue_->callAvailable();
  bias_.get_force().x() = wrench_.wrench.force.x;
  bias_.get_force().y() = wrench_.wrench.force.y;
  bias_.get_force().z() = wrench_.wrench.force.z;
  bias_.get_torque().x() = wrench_.wrench.torque.x;
  bias_.get_torque().y() = wrench_.wrench.torque.y;
  bias_.get_torque().z() = wrench_.wrench.torque.z;

  ROS_INFO_STREAM("Wrench offset reset to: " << bias_);
  active_ = true;
  return true;
}

template <int N>
void MPC_AdmittanceController::parse_vector(ros::NodeHandle& nh, const std::string& name,
                                            Eigen::Matrix<double, N, 1>& gains) {
  std::vector<double> gains_vector;
  if (!nh.param<std::vector<double>>(name, gains_vector, {})) {
    ROS_WARN_STREAM("Failed to parse " << name << ", defaulting to zero vector");
  }
  if (gains_vector.size() != N) {
    ROS_ERROR_STREAM("Gains have the wrong size! Defaulting to zero");
    gains_vector.clear();
    gains_vector.resize(N, 0.0);
  }

  for (size_t i = 0; i < N; i++) gains(i) = gains_vector[i];
}