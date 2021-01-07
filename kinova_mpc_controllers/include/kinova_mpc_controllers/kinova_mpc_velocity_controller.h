//
// Created by giuseppe on 06.01.21.
//

#pragma once
#include <robot_control/modeling/robot_wrapper.h>

#include <control_toolbox/pid.h>
#include <kinova_robot/command_interface.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <kinova_mpc_controllers/mpc_controller.h>

namespace kinova_controllers {

class KinovaMpcVelocityController : public controller_interface::MultiInterfaceController<
                                        hardware_interface::JointStateInterface> {
 public:
  using joint_vector_t = Eigen::Matrix<double, 7, 1>;

  KinovaMpcVelocityController() = default;
  ~KinovaMpcVelocityController() = default;

 private:
  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 protected:
  bool addStateHandles(hardware_interface::RobotHW* hw);
  bool addCommandHandles(hardware_interface::RobotHW* hw);
  void writeCommand(const ros::Duration& period);
  void readState();

 private:
  bool is_real_robot_;
  joint_vector_t joint_state_;
  std::vector<std::string> joint_names_;

  // model for the simulation
  std::unique_ptr<rc::RobotWrapper> model_;
  std::vector<control_toolbox::Pid> pid_controllers_;

  // mpc controller
  std::unique_ptr<MPC_Controller> mpc_controller_;

  // joint handles
  std::array<hardware_interface::JointStateHandle, 7> state_handles_;
  std::array<hardware_interface::JointHandle, 7> sim_command_handles_;
  std::array<hardware_interface::KinovaCommandHandle, 7> robot_command_handles_;
};
}  // namespace kinova_controllers
