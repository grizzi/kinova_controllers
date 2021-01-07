//
// Created by giuseppe on 06.01.21.
//

#include "kinova_mpc_controllers/kinova_mpc_velocity_controller.h"
#include <pluginlib/class_list_macros.h>

using namespace hardware_interface;

namespace kinova_controllers {

bool KinovaMpcVelocityController::init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
                                       ros::NodeHandle& controller_nh) {
  controller_nh.param<std::vector<std::string>>("joint_names", joint_names_, {});
  controller_nh.param<bool>("is_real_robot", is_real_robot_, {});

  mpc_controller_ = std::unique_ptr<MPC_Controller>(new MPC_Controller(controller_nh));
  mpc_controller_->init();

  addStateHandles(hw);
  addCommandHandles(hw);

}

void KinovaMpcVelocityController::starting(const ros::Time& time) {
  readState();
  mpc_controller_->start(joint_state_);
}

void KinovaMpcVelocityController::update(const ros::Time& time, const ros::Duration& period){
  readState();
  mpc_controller_->update(time, joint_state_);
  writeCommand(period);
}

void KinovaMpcVelocityController::stopping(const ros::Time& time){
  mpc_controller_->stop();
}

bool KinovaMpcVelocityController::addStateHandles(hardware_interface::RobotHW* hw) {
  // only arm joints
  auto state_interface = hw->get<JointStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get state interface");
    return false;
  }
  for (size_t i=0; i < 7; i++) {
    state_handles_[i] = state_interface->getHandle(joint_names_[i]);
  }
  return true;
}

bool KinovaMpcVelocityController::addCommandHandles(hardware_interface::RobotHW* hw) {
  std::string robot_description = mpc_controller_->get_description();
  model_ = std::unique_ptr<rc::RobotWrapper>(new rc::RobotWrapper());
  model_->initFromXml(robot_description);

  // only arm joints
  if (is_real_robot_) {
    auto command_interface = hw->get<KortexCommandInterface>();
    if (command_interface == nullptr) {
      ROS_ERROR_STREAM("Can't get state interface");
      return false;
    }
    for (size_t i = 0; i < 7; i++) {
      robot_command_handles_[i] = command_interface->getHandle(joint_names_[i]);
    }
    return true;
  } else {
    auto command_interface = hw->get<EffortJointInterface>();
    if (command_interface == nullptr) {
      ROS_ERROR_STREAM("Can't get state interface");
      return false;
    }
    for (size_t i = 0; i < 7; i++) {
      sim_command_handles_[i] = command_interface->getHandle(joint_names_[i]);
      if (!pid_controllers_[i].init(ros::NodeHandle(nh_, joint_names_[i] + "/pid"))) {
        ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
        return false;
      }
    }
    return true;
  }
}

void KinovaMpcVelocityController::readState(){
  for(size_t i=0; i < 7; i++){
    joint_state_(i) = state_handles_[i].getPosition();
  }
}
void KinovaMpcVelocityController::writeCommand(const ros::Duration& period) {
  if (is_real_robot_) {
    for (size_t i = 0; i < 7; i++) {
      robot_command_handles_[i].setMode(KortexControlMode::VELOCITY);
      robot_command_handles_[i].setVelocityCommand(mpc_controller_->get_velocity_command()(i));
    }
  } else {
    // discard the base 2D twist command and the base 3D position
    Eigen::VectorXd position_command = mpc_controller_->get_position_command();
    Eigen::VectorXd velocity_command = mpc_controller_->get_velocity_command();

    // update current state
    joint_vector position_current;
    joint_vector velocity_current;
    for (size_t i = 0; i < 7; i++) {
      position_current(i) = state_handles_[i].getPosition();
      velocity_current(i) = state_handles_[i].getVelocity();
    }
    model_->updateState(position_current, velocity_current);
    model_->computeAllTerms();
    joint_vector gravity_and_coriolis = model_->getNonLinearTerms();

    // compute tau with pd gains
    joint_vector tau;
    joint_vector position_error = position_command - position_current;
    joint_vector velocity_error = velocity_command - velocity_current;
    for (size_t i = 0; i < 7; i++) {
      sim_command_handles_[i].setCommand(
          pid_controllers_[i].computeCommand(position_error(i), velocity_error(i), period) +
          gravity_and_coriolis(i));
    }
  }
}

}  // namespace kinova_controllers

PLUGINLIB_EXPORT_CLASS(kinova_controllers::KinovaMpcVelocityController,
                       controller_interface::ControllerBase)