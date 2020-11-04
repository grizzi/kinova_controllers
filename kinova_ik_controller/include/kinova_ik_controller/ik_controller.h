/*!
 * @file     ik_controller.h
 * @author   Giuseppe Rizzi
 * @date     28.10.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <robot_control_ros/ik_controller.h>
#include <kortex_driver/non-generated/kortex_command_interface.h>
#include <controller_interface/multi_interface_controller.h>

namespace kinova_controllers {
 class IKControllerKinova : public rc_ros::IKControllerBase<hardware_interface::JointStateInterface,
                                                            hardware_interface::JointStateHandle,
                                                            hardware_interface::KortexCommandInterface,
                                                            hardware_interface::KortexCommandHandle> {

 void update(const ros::Time& time, const ros::Duration& period) override;
 void stopping(const ros::Time& /*time*/) override;

 };

}

