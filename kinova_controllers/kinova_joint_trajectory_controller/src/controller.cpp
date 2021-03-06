#include "kinova_joint_trajectory_controller/controller.h"
#include "kinova_joint_trajectory_controller/JointResult.h"

#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>

namespace kinova_controllers {

bool KinovaJointTrajectoryController::init(hardware_interface::RobotHW* hw,
                                           ros::NodeHandle& root_nh,
                                           ros::NodeHandle& controller_nh) {
  controller_nh.param<std::vector<std::string>>("joint_names", joint_names_, {});
  if (joint_names_.size() != 7) {
    ROS_ERROR("Joint names must be 7.");
    return false;
  }

  if (!controller_nh.getParam("lower_limit", lower_limit_) || lower_limit_.size() != 7) {
    ROS_ERROR_STREAM("Failed to get lower_limit or invalid param");
    return false;
  }

  if (!controller_nh.getParam("upper_limit", upper_limit_) || upper_limit_.size() != 7) {
    ROS_ERROR_STREAM("Failed to get upper_limit or invalid param");
    return false;
  }

  if (!controller_nh.getParam("max_velocity", max_velocity_) || max_velocity_ < 0) {
    ROS_ERROR_STREAM("Failed to get max_velocity or invalid param");
    return false;
  }

  double max_acceleration;
  if (!controller_nh.getParam("max_acceleration", max_acceleration) || max_acceleration < 0) {
    ROS_ERROR_STREAM("Failed to get max_acceleration or invalid param");
    return false;
  }

  if (!controller_nh.getParam("gain", gain_) || gain_ < 0) {
    ROS_ERROR_STREAM("Failed to get gain or invalid param");
    return false;
  }

  if (!controller_nh.getParam("tolerance", tolerance_) || tolerance_ < 0) {
    ROS_ERROR_STREAM("Failed to get tolerance or invalid param");
    return false;
  }

  auto command_interface = hw->get<hardware_interface::KinovaCommandInterface>();
  if (command_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get command interface");
    return false;
  }

  for (size_t i = 0; i < 7; i++) {
    command_handles_[i] = command_interface->getHandle(joint_names_[i]);
  }

  generator_ = std::make_unique<TrajectoryGenerator>(max_velocity_, max_acceleration,
                                                     7);  // max vel, max acc, size
  joint_desired_ = Eigen::VectorXd::Zero(7);
  joint_current_ = Eigen::VectorXd::Zero(7);

  trajectory_subscriber_ =
      controller_nh.subscribe("/kinova_joint_trajectory_controller/goal", 1,
                              &KinovaJointTrajectoryController::joint_callback, this);

  action_server_ = std::make_unique<ActionServer>(
      controller_nh, "/kinova_joint_trajectory_server",
      boost::bind(&KinovaJointTrajectoryController::execute_callback, this, _1), false);
  return true;
}

void KinovaJointTrajectoryController::starting(const ros::Time& time) {
  for (size_t i = 0; i < 7; i++) {
    command_handles_[i].setMode(hardware_interface::KinovaControlMode::VELOCITY);
    command_handles_[i].setCommand(0.0);
  }
  trajectory_available_ = false;
  action_server_->start();
}

void KinovaJointTrajectoryController::update(const ros::Time& time, const ros::Duration& period) {
  if (trajectory_available_) {
    Eigen::VectorXd joint_desired_now;
    {
      std::unique_lock<std::mutex> lock(generator_mutex_);
      joint_desired_now = generator_->get_next_point(ros::Time::now().toSec());
    }

    bool all_reached = true;
    for (size_t i = 0; i < 7; i++) {
      double joint_error, velocity_command;
      angles::shortest_angular_distance_with_large_limits(command_handles_[i].getPosition(),
                                                          joint_desired_now(i), lower_limit_[i],
                                                          upper_limit_[i], joint_error);
      std::stringstream ss;
      ss << "Joint current=" << command_handles_[i].getPosition() << std::endl;
      ss << "joint desired=" << joint_desired_now(i) << std::endl;
      ss << "lower limit=" << lower_limit_[i] << std::endl;
      ss << "upper limit=" << upper_limit_[i] << std::endl;
      ss << "error=" << joint_error << std::endl;
      ROS_DEBUG_STREAM_THROTTLE(1.0, ss.str());

      if (abs(joint_error) < tolerance_) {
        velocity_command = 0.0;
      } else {
        velocity_command = std::min(std::max(gain_ * joint_error, -max_velocity_), max_velocity_);
        all_reached = false;
      }

      success_ = all_reached;
      command_handles_[i].setCommand(velocity_command);
    }
  }
}

void KinovaJointTrajectoryController::stopping(const ros::Time& time) {
  for (size_t i = 0; i < 7; i++) {
    command_handles_[i].setCommand(0.0);
  }
  action_server_->shutdown();
}

void KinovaJointTrajectoryController::compute_profile(const Eigen::VectorXd& goal) {
  // the next desired point is the closest within the limits
  double delta = 0;
  for (size_t i = 0; i < 7; i++) {
    angles::shortest_angular_distance_with_large_limits(command_handles_[i].getPosition(), goal[i],
                                                        lower_limit_[i], upper_limit_[i], delta);

    joint_desired_[i] = command_handles_[i].getPosition() + delta;
    joint_current_[i] = command_handles_[i].getPosition();
  }
  ROS_INFO_STREAM("Received new joint target configuration."
                  << std::endl
                  << "Current is: " << joint_current_.transpose() << std::endl
                  << "Desired is: " << joint_desired_.transpose());

  {
    std::unique_lock<std::mutex> lock(generator_mutex_);
    generator_->compute(joint_current_, joint_desired_, ros::Time::now().toSec());
  }

  ROS_INFO("Computed new velocity profile.");
  trajectory_available_ = true;
}

void KinovaJointTrajectoryController::execute_callback(
    const kinova_joint_trajectory_controller::JointGoalConstPtr& goal) {
  if (goal->position.size() != 7) {
    ROS_ERROR_STREAM(
        "Target joint configuration message has the wrong size: " << goal->position.size());
    return;
  }

  Eigen::VectorXd joint_desired(7);
  for (size_t i = 0; i < 7; i++) joint_desired[i] = goal->position[i];
  compute_profile(joint_desired);

  ros::Rate r(1);
  kinova_joint_trajectory_controller::JointResult result;
  result.success = false;
  success_ = false;

  // start executing the action
  while (ros::ok()) {
    // check that preempt has not been requested by the client
    if (action_server_->isPreemptRequested()) {
      ROS_INFO("KinovaJointTrajectoryController preempted");
      action_server_->setPreempted();
      trajectory_available_ = false;
      return;
    }

    if (success_) {
      result.success = true;
      action_server_->setSucceeded(result);
      ROS_INFO("[KinovaJointTrajectoryController::execute_callback]: Goal reached.");
      trajectory_available_ = false;
      return;
    }
    r.sleep();
  }
}

void KinovaJointTrajectoryController::joint_callback(const sensor_msgs::JointStateConstPtr& msg) {
  if (msg->position.size() != 7) {
    ROS_ERROR_STREAM(
        "Target joint configuration message has the wrong size: " << msg->position.size());
    return;
  }

  Eigen::VectorXd joint_desired(7);
  for (size_t i = 0; i < 7; i++) joint_desired[i] = msg->position[i];
  compute_profile(joint_desired);
}
}

PLUGINLIB_EXPORT_CLASS(kinova_controllers::KinovaJointTrajectoryController,
                       controller_interface::ControllerBase)
