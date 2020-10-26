/*!
 * @file     task_space_controller.cpp
 * @author   Giuseppe Rizzi
 * @date     24.10.2020
 * @version  1.0
 * @brief    description
 */

#include "kinova_task_space_controller/task_space_controller.h"

#include <cmath>
#include <ros/ros.h>
#include <Eigen/Dense>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <pinocchio/spatial/se3.hpp>

namespace kinova_controllers {

template<class I, class H>
KinovaTaskSpaceController<I, H>::~KinovaTaskSpaceController(){
  if (publish_thread_.joinable()) publish_thread_.join();
}

template<class I, class H>
bool KinovaTaskSpaceController<I, H>::init(
    I* robot_hw,
    ros::NodeHandle& node_handle,
    ros::NodeHandle& ctrl_handle) {
  std::string robot_description;
  if (!node_handle.getParam("/kinova_no_gripper", robot_description)) {
    ROS_ERROR_STREAM("Can't read robot description.");
    return false;
  }
  if (!ctrl_handle.getParam("controlled_frame", controlled_frame_)) {
    ROS_ERROR_STREAM("Set the controlled_frame parameter.");
    return false;
  }

  if (!ctrl_handle.getParam("publish_ros", publish_ros_)) {
    ROS_ERROR_STREAM("Set the publish_ros parameter.");
    return false;
  }

  if (!ctrl_handle.getParam("publish_ros", publish_ros_)) {
    ROS_ERROR_STREAM("Set the publish_ros parameter.");
    return false;
  }

  if (!ctrl_handle.getParam("publish_ros_rate", publish_ros_rate_)) {
    ROS_ERROR_STREAM("Set the publish_ros_rate parameter.");
    return false;
  }

  robot_wrapper = std::make_shared<rc::RobotWrapper>();
  robot_wrapper->initFromXml(robot_description);
  controller = std::make_shared<rc::TaskSpaceController>(robot_wrapper, controlled_frame_);

  for (auto& joint_name : joint_names_) {
    joint_handles_.push_back(robot_hw->getHandle(joint_name));
  }

  std::string current_pose_topic = ctrl_handle.param<std::string>("current_pose_topic", "/current_pose");
  pose_publisher_ = std::make_unique<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>>(node_handle, current_pose_topic, 10);

  std::string target_pose_topic = ctrl_handle.param<std::string>("target_pose_topic", "/target_pose");
  target_subscriber_ = node_handle.subscribe("/target_pose", 10, &KinovaTaskSpaceController::newTargetCallback, this);

  if (publish_ros_)
    publish_thread_ = std::thread(&KinovaTaskSpaceController::publish_thread, this);
  return true;
}

template<class I, class H>
void KinovaTaskSpaceController<I, H>::newTargetCallback(const geometry_msgs::PoseStamped& msg) {
  Eigen::Vector3d translation(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
  Eigen::Quaterniond rotation(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
  target_pose_ = pin::SE3(rotation, translation);
  controller->setTaskTarget(target_pose_);
}

template<class I, class H>
void KinovaTaskSpaceController<I, H>::starting(const ros::Time& time) {}

template<class I, class H>
void KinovaTaskSpaceController<I, H>::getJointVelocities(Eigen::VectorXd& q) const {
  for(size_t i=0; i < dof; i++){
    q(i) = joint_handles_[i].getVelocity();
  }
}

template<class I, class H>
void KinovaTaskSpaceController<I, H>::getJointPositions(Eigen::VectorXd& qd) const {
  for(size_t i=0; i< dof; i++) {
    qd(i) = joint_handles_[i].getPosition();
  }
}

template<class I, class H>
void KinovaTaskSpaceController<I, H>::publish_thread() {
  ros::Rate rate(publish_ros_rate_);
  while (ros::ok()){
    if (pose_publisher_->trylock()){
      current_pose_ = robot_wrapper->getFramePlacement(controlled_frame_);
      pose_publisher_->msg_.header.stamp = ros::Time::now();
      pose_publisher_->msg_.header.frame_id = frame_id_;
      pose_publisher_->msg_.pose.position.x = current_pose_.translation()(0);
      pose_publisher_->msg_.pose.position.y = current_pose_.translation()(1);
      pose_publisher_->msg_.pose.position.z = current_pose_.translation()(2);
      Eigen::Quaterniond q(current_pose_.rotation());
      pose_publisher_->msg_.pose.orientation.x = q.x();
      pose_publisher_->msg_.pose.orientation.y = q.y();
      pose_publisher_->msg_.pose.orientation.z = q.z();
      pose_publisher_->msg_.pose.orientation.w = q.w();
      pose_publisher_->unlockAndPublish();
    }
    rate.sleep();
  }
}

void KinovaTaskSpaceControllerRobot::update(const ros::Time& time, const ros::Duration& period) {
  getJointPositions(q_);
  getJointVelocities(qd_);
  cmd_ = controller->advance(q_, qd_);
  for (size_t i=0; i < dof; i++) {
    joint_handles_[i].setEffortCommand(cmd_[i]);
    joint_handles_[i].setMode(hardware_interface::KortexControlMode::EFFORT);
  }
}

void KinovaTaskSpaceControllerSim::update(const ros::Time& time, const ros::Duration& period) {
  getJointPositions(q_);
  getJointVelocities(qd_);
  cmd_ = controller->advance(q_, qd_);
  for (size_t i=0; i < dof; i++) {
    joint_handles_[i].setCommand(cmd_[i]);
  }
}



}

PLUGINLIB_EXPORT_CLASS(kinova_controllers::KinovaTaskSpaceControllerRobot, controller_interface::ControllerBase)
PLUGINLIB_EXPORT_CLASS(kinova_controllers::KinovaTaskSpaceControllerSim, controller_interface::ControllerBase)

