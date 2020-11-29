/*!
 * @file     kinova_impedance_motion_controller.h
 * @author   Giuseppe Rizzi
 * @date     28.11.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <robot_control_ros/impedance_motion_controller.h>
#include <kortex_driver/non-generated/kortex_command_interface.h>
#include <controller_interface/multi_interface_controller.h>

namespace kinova_controllers {
class ImpedanceControllerKinova :
    public rc_ros::ImpedanceFTController<hardware_interface::JointStateInterface,
                                         hardware_interface::JointStateHandle,
                                         hardware_interface::KortexCommandInterface,
                                         hardware_interface::KortexCommandHandle> {

  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& /*time*/) override;

};

}
