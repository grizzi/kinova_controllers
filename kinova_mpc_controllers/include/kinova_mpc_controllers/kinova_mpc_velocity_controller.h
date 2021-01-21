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
#include <sensor_msgs/JointState.h>

namespace kinova_controllers {

class KinovaMpcVelocityController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::JointStateInterface, hardware_interface::EffortJointInterface,
          hardware_interface::KinovaCommandInterface> {
 public:
  using joint_vector_t = Eigen::Matrix<double, 7, 1>;

  using BASE =
      controller_interface::MultiInterfaceController<hardware_interface::JointStateInterface,
                                                     hardware_interface::EffortJointInterface,
                                                     hardware_interface::KinovaCommandInterface>;

  // optional interfaces
  KinovaMpcVelocityController() : BASE(true){};
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
  void computeTorqueCommands(joint_vector_t& tau, const ros::Duration& period);
  void readState();

 private:
  std::string tool_frame_;
  bool is_real_robot_;
  std::string robot_description_;
  std::vector<std::string> joint_names_;
  ros::Publisher reset_imarker_pose_pub_;

  sensor_msgs::JointState joint_state_des_;
  sensor_msgs::JointState joint_state_cur_;
  ros::Publisher joint_state_des_pub_;
  ros::Publisher joint_state_cur_pub_;

  // model for the simulation
  std::unique_ptr<rc::RobotWrapper> model_;
  std::array<control_toolbox::Pid, 7> pid_controllers_;
  joint_vector_t position_command_;
  joint_vector_t velocity_command_;
  joint_vector_t position_error_;
  joint_vector_t velocity_error_;
  joint_vector_t gravity_and_coriolis_;
  Eigen::VectorXd position_current_;    // compute PD error: dynamic vector in the gripper case
  Eigen::VectorXd velocity_current_;    // compute PD error: dynamic vector in the gripper case

  // mpc controller
  std::unique_ptr<MPC_Controller> mpc_controller_;

  // joint handles
  std::array<hardware_interface::JointStateHandle, 7> state_handles_;
  std::array<hardware_interface::JointHandle, 7> sim_command_handles_;
  std::array<hardware_interface::KinovaCommandHandle, 7> robot_command_handles_;
};
}  // namespace kinova_controllers
