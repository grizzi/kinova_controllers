//
// Created by giuseppe on 08.03.21.
//

#pragma once
#include "kinova_mpc_controllers/ros/kinova_mpc_ros.h"
#include "kinova_mpc_controllers/mpc_admittance_controller.h"
#include "kinova_mpc_controllers/mpc_velocity_controller.h"

namespace kinova_controllers {


class KinovaMpcControllerSim : public KinovaMpcControllerRos<kinova_controllers::MPC_VelocityController> {
 protected:
  bool addCommandHandles(hardware_interface::RobotHW* hw) override;
  void writeCommand(const ros::Duration& period) override;

 private:
  std::array<hardware_interface::JointHandle, 7> command_handles_;
};

template <typename Controller>
class KinovaMpcControllerRobot : public KinovaMpcControllerRos<Controller> {
 protected:
  void stopping(const ros::Time& time) override;
  bool addCommandHandles(hardware_interface::RobotHW* hw) override;
  virtual void writeCommand(const ros::Duration& period);

 protected:
  std::array<hardware_interface::KinovaCommandHandle, 7> command_handles_;
};

template <typename Controller>
class KinovaMpcControllerRobotTorque : public KinovaMpcControllerRobot<Controller> {
 protected:
  void writeCommand(const ros::Duration& period) override;
};

class KinovaMpcControllerRobotPositionVelocity
    : public KinovaMpcControllerRobot<MPC_VelocityController> {};
class KinovaMpcControllerRobotPositionTorque
    : public KinovaMpcControllerRobotTorque<MPC_VelocityController> {};
class KinovaMpcControllerRobotAdmittanceVelocity
    : public KinovaMpcControllerRobot<MPC_AdmittanceController> {};
class KinovaMpcControllerRobotAdmittanceTorque
    : public KinovaMpcControllerRobotTorque<MPC_AdmittanceController> {};

}  // namespace kinova_controllers