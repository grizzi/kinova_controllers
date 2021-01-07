//
// Created by giuseppe on 06.01.21.
//

#include "kinova_mpc_controllers/kinova_mpc_velocity_controller.h"
#include <pluginlib/class_list_macros.h>

using namespace hardware_interface;

namespace kinova_controllers {

bool KinovaMpcVelocityController::addStateHandles(hardware_interface::RobotHW *hw) {
  // add fake handles for the base joints
  for (size_t i = 0; i < 3; i++) {
    stateHandles_.emplace_back("x_base", &fake_base_pos[0], &fake_base_vel[0], &fake_base_eff[0]);
    stateHandles_.emplace_back("y_base", &fake_base_pos[1], &fake_base_vel[1], &fake_base_eff[1]);
    stateHandles_.emplace_back("w_base", &fake_base_pos[2], &fake_base_vel[2], &fake_base_eff[2]);
  }
  // only arm joints
  auto state_interface = hw->get<JointStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get state interface");
    return false;
  }
  for (auto &jointName : jointNames_) {
    stateHandles_.push_back(state_interface->getHandle(jointName));
  }
  return true;
}

bool KinovaMpcVelocityController::addCommandHandles(hardware_interface::RobotHW *hw) {
  // setup robot model
  std::string robotDescription;
  if (!controllerNh_.param<std::string>("/robot_description", robotDescription, "")) {
    ROS_ERROR("Failed to find is_real_robot param");
    return false;
  };

  model_ = std::unique_ptr<rc::RobotWrapper>(new rc::RobotWrapper());
  model_->initFromXml(robotDescription);

  if (!controllerNh_.param<bool>("is_real_robot", isRealRobot_, "false")) {
    ROS_ERROR("Failed to find is_real_robot param");
    return false;
  };

  // only arm joints
  if (isRealRobot_) {
    auto command_interface = hw->get<KortexCommandInterface>();
    if (command_interface == nullptr) {
      ROS_ERROR_STREAM("Can't get state interface");
      return false;
    }
    for (size_t i = 0; i < jointNames_.size(); i++) {
      robotCommandHandles_[i] = command_interface->getHandle(jointNames_[i]);
    }
    return true;
  } else {
    auto command_interface = hw->get<EffortJointInterface>();
    if (command_interface == nullptr) {
      ROS_ERROR_STREAM("Can't get state interface");
      return false;
    }
    for (size_t i = 0; i < jointNames_.size(); i++) {
      commandHandles_.push_back(std::unique_ptr<JointHandle>(
          new JointHandle(command_interface->getHandle(jointNames_[i]))));
      if (!pid_controllers_[i].init(ros::NodeHandle(controllerNh_, jointNames_[i] + "/pid"))) {
        ROS_ERROR_STREAM("Failed to load PID parameters from " << jointNames_[i] + "/pid");
        return false;
      }
    }
    return true;
  }
}

void KinovaMpcVelocityController::writeCommand(const ros::Duration &period) {
  if (isRealRobot_) {
    for (size_t i = 0; i < 7; i++) {
      robotCommandHandles_[i].setMode(KortexControlMode::VELOCITY);
      robotCommandHandles_[i].setVelocityCommand(velocityCommand_(2 + i));
    }
  } else {
    // discard the base 2D twist command and the base 3D position
    Eigen::VectorXd position_command = positionCommand_.segment(3, positionCommand_.size() - 3);
    Eigen::VectorXd velocity_command = velocityCommand_.segment(2, velocityCommand_.size() - 2);

    // update current state
    joint_vector position_current;
    joint_vector velocity_current;
    for (size_t i = 0; i < 7; i++) {
      position_current(i) = stateHandles_[3 + i].getPosition();
      velocity_current(i) = stateHandles_[3 + i].getVelocity();
    }
    model_->updateState(position_current, velocity_current);
    model_->computeAllTerms();
    joint_vector gravity_and_coriolis = model_->getNonLinearTerms();

    // compute tau with pd gains
    joint_vector tau;
    joint_vector position_error = position_command - position_current;
    joint_vector velocity_error = velocity_command - velocity_current;
    for (size_t i = 0; i < 7; i++) {
      commandHandles_[i]->setCommand(
          pid_controllers_[i].computeCommand(position_error(i), velocity_error(i), period) +
          gravity_and_coriolis(i));
    }
  }
}

}  // namespace kinova_controllers

PLUGINLIB_EXPORT_CLASS(kinova_controllers::KinovaMpcVelocityController,
                       controller_interface::ControllerBase)