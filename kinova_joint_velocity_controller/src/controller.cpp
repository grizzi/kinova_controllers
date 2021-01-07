#include "kinova_joint_velocity_controller/controller.h"
#include <pluginlib/class_list_macros.h>


namespace kinova_controllers{

bool KinovaJointVelocityController::init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  controller_nh.param<std::vector<std::string>>("joint_names", joint_names_, {});
  if (joint_names_.size() != 7){
    ROS_ERROR("Joint names must be 7.");
    return false;
  }

  auto command_interface = hw->get<hardware_interface::KinovaCommandInterface>();
  if (command_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get state interface");
    return false;
  }
  for (size_t i = 0; i < 7; i++) {
    command_handles_[i] = command_interface->getHandle(joint_names_[i]);
  }
  return true;
}

void KinovaJointVelocityController::starting(const ros::Time& time) {
  for (size_t i=0; i<7; i++){
    command_handles_[i].setMode(hardware_interface::KinovaControlMode::VELOCITY);
    command_handles_[i].setCommand(0.0);
  }
  command_handles_[6].setCommand(0.1); // TODO(giuseppe) this is for debugging
}

void KinovaJointVelocityController::update(const ros::Time& time, const ros::Duration& period) {
  ROS_WARN_THROTTLE(5.0, "KinovaJointVelocityController only to test velocity control mode.");
}

void KinovaJointVelocityController::stopping(const ros::Time& time) {
  command_handles_[6].setCommand(0.0);
}
}

PLUGINLIB_EXPORT_CLASS(kinova_controllers::KinovaJointVelocityController,
                       controller_interface::ControllerBase)
