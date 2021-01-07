//
// Created by giuseppe on 06.01.21.
//

#pragma once

#include <robot_control/modeling/robot_wrapper.h>
#include <kortex_driver/non-generated/kortex_command_interface.h>
#include <ocs2_mobile_manipulator_example/MobileManipulatorController.h>
#include <control_toolbox/pid.h>

namespace kinova_controllers {

class KinovaMpcVelocityController : public mobile_manipulator::MobileManipulatorController {
 public:
  using BASE = mobile_manipulator::MobileManipulatorController;
  using joint_vector = Eigen::Matrix<double, 7, 1>;

  KinovaMpcVelocityController() : BASE(){};
  ~KinovaMpcVelocityController() = default;

 protected:
  bool addStateHandles(hardware_interface::RobotHW* hw) override;
  bool addCommandHandles(hardware_interface::RobotHW* hw) override;
  void writeCommand(const ros::Duration& period) override;

 private:
  bool isRealRobot_;
  std::array<double, 3> fake_base_pos;
  std::array<double, 3> fake_base_vel;
  std::array<double, 3> fake_base_eff;

  std::array<hardware_interface::KortexCommandHandle, 7> robotCommandHandles_;

  // model for the simulation
  std::unique_ptr<rc::RobotWrapper> model_;
  std::vector<control_toolbox::Pid> pid_controllers_;
};
}  // namespace kinova_controllers
