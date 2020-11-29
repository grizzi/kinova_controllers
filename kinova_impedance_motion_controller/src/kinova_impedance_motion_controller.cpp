/*!
 * @file     kinova_impedance_motion_controller.cpp
 * @author   Giuseppe Rizzi
 * @date     28.11.2020
 * @version  1.0
 * @brief    description
 */

#include <pluginlib/class_list_macros.h>
#include "kinova_impedance_motion_controller/kinova_impedance_motion_controller.h"

using namespace kinova_controllers;

// explicit instantiation
template class rc_ros::ImpedanceFTController<hardware_interface::JointStateInterface,
                                             hardware_interface::JointStateHandle,
                                             hardware_interface::KortexCommandInterface,
                                             hardware_interface::KortexCommandHandle>;

void ImpedanceControllerKinova::update(const ros::Time& time, const ros::Duration& period) {
  updateCommand();
  for (size_t i=0; i < nr_chain_joints_; i++) {
    joint_handles_[i].setEffortCommand(eff_command_[i]);
    joint_handles_[i].setPositionCommand(pos_command_[i]);
    joint_handles_[i].setMode(hardware_interface::KortexControlMode::EFFORT);
  }
}

void ImpedanceControllerKinova::stopping(const ros::Time& time) {
  ImpedanceMotionControllerBase::stopping(time);
  for (size_t i=0; i < nr_chain_joints_; i++) {
    joint_handles_[i].setMode(hardware_interface::KortexControlMode::VELOCITY);
  }
}



PLUGINLIB_EXPORT_CLASS(kinova_controllers::ImpedanceControllerKinova, controller_interface::ControllerBase)