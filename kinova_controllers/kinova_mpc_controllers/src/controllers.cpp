//
// Created by giuseppe on 08.03.21.
//

#include "kinova_mpc_controllers/ros/kinova_mpc_controllers.h"
#include <controller_interface/controller.h>
#include <pluginlib/class_list_macros.h>

using namespace hardware_interface;

namespace kinova_controllers {

bool KinovaMpcControllerSim::addCommandHandles(hardware_interface::RobotHW* hw) {
  auto command_interface = hw->get<EffortJointInterface>();
  if (command_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get state interface");
    return false;
  }
  for (size_t i = 0; i < 7; i++) {
    command_handles_[i] = command_interface->getHandle(joint_names_[i]);
  }
  return true;
}

void KinovaMpcControllerSim::writeCommand(const ros::Duration& period) {
  computeTorqueCommands(this->tau_, period);
  for (size_t i = 0; i < 7; i++) {
    command_handles_[i].setCommand(this->tau_(i));
  }
}

template <typename Controller>
void KinovaMpcControllerRobot<Controller>::stopping(const ros::Time& time) {
  ROS_INFO("Stopping Mpc Controller!");
  this->mpc_controller_->stop();
  for (auto& handle : command_handles_) {
    handle.setMode(KinovaControlMode::NO_MODE);
  }
}

template <typename Controller>
bool KinovaMpcControllerRobot<Controller>::addCommandHandles(hardware_interface::RobotHW* hw) {
  // only arm joints
  auto command_interface = hw->get<KinovaCommandInterface>();
  if (command_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get state interface");
    return false;
  }
  for (size_t i = 0; i < 7; i++) {
    command_handles_[i] = command_interface->getHandle(this->joint_names_[i]);
  }
  ROS_INFO("Successfully added robot command handles.");
  return true;
}

template <typename Controller>
void KinovaMpcControllerRobot<Controller>::writeCommand(const ros::Duration& period) {
  for (size_t i = 0; i < 7; i++) {
    KinovaControlMode mode = KinovaControlMode::VELOCITY;
    command_handles_[i].setMode(mode);
    //command_handles_[i].setVelocityCommand(this->mpc_controller_->get_velocity_command()(i)); // TODO(giuseppe) bug here
  }
}

template <typename Controller>
void KinovaMpcControllerRobotTorque<Controller>::writeCommand(const ros::Duration& period) {
  this->computeTorqueCommands(this->tau_, period);
  for (size_t i = 0; i < 7; i++) {
    this->command_handles_[i].setMode(KinovaControlMode::EFFORT);
    this->command_handles_[i].setEffortCommand(this->tau_(i));
  }
}

}  // namespace kinova_controllers

// Explicit instantiations
namespace kinova_controllers {

template class KinovaMpcControllerRos<MPC_VelocityController>;
template class KinovaMpcControllerRobot<MPC_VelocityController>;
template class KinovaMpcControllerRobotTorque<MPC_VelocityController>;
template class KinovaMpcControllerRobot<MPC_AdmittanceController>;
template class KinovaMpcControllerRobotTorque<MPC_AdmittanceController>;

}  // namespace kinova_controllers

// Export plugins
PLUGINLIB_EXPORT_CLASS(kinova_controllers::KinovaMpcControllerSim,
                       controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(kinova_controllers::KinovaMpcControllerRobotPositionVelocity,
                       controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(kinova_controllers::KinovaMpcControllerRobotPositionTorque,
                       controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(kinova_controllers::KinovaMpcControllerRobotAdmittanceVelocity,
                       controller_interface::ControllerBase)

PLUGINLIB_EXPORT_CLASS(kinova_controllers::KinovaMpcControllerRobotAdmittanceTorque,
                       controller_interface::ControllerBase)
