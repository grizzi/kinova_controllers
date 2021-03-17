//
// Created by giuseppe on 06.01.21.
//

#pragma once
#include <robot_control/modeling/robot_wrapper.h>

#include <control_toolbox/pid.h>
#include <kinova_robot/command_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/multi_interface_controller.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

namespace kinova_controllers {

template<typename Controller>
class KinovaMpcControllerRos
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
  KinovaMpcControllerRos() : BASE(true){};
  ~KinovaMpcControllerRos() = default;

 private:
  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  virtual void stopping(const ros::Time& time) {};

 protected:
  bool initRos(ros::NodeHandle& nh);
  bool addStateHandles(hardware_interface::RobotHW* hw);
  virtual bool addCommandHandles(hardware_interface::RobotHW* hw) = 0;
  virtual void writeCommand(const ros::Duration& period) = 0;
  void computeTorqueCommands(joint_vector_t& tau, const ros::Duration& period);
  void readState();

 protected:
  std::vector<std::string> joint_names_;
  std::unique_ptr<Controller> mpc_controller_;
  joint_vector_t tau_;

 private:
  std::string tool_frame_;
  bool is_real_robot_;
  std::string robot_description_;

  // Ros
  sensor_msgs::JointState joint_state_des_;
  sensor_msgs::JointState joint_state_cur_;
  ros::Publisher joint_state_des_pub_;
  ros::Publisher joint_state_cur_pub_;
  ros::Publisher reset_imarker_pose_pub_;

  // dynamic model
  std::unique_ptr<rc::RobotWrapper> model_;
  std::array<control_toolbox::Pid, 7> pid_controllers_;
  joint_vector_t position_command_;
  joint_vector_t velocity_command_;
  joint_vector_t position_error_;
  joint_vector_t velocity_error_;
  joint_vector_t gravity_and_coriolis_;

  // State dynamic vector to account for eventual gripper case
  Eigen::VectorXd position_current_; 
  Eigen::VectorXd velocity_current_;
  Eigen::VectorXd torque_current_;
  Eigen::VectorXd torque_offset_;
  Eigen::VectorXd position_integral_;

  std::array<hardware_interface::JointStateHandle, 7> state_handles_;
};
}  // namespace kinova_controllers

#include "kinova_mpc_controllers/ros/kinova_mpc_ros.tpp"
