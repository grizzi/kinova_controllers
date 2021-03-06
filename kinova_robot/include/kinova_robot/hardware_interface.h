#pragma once

/*!
 * @file     hardware_interface.h
 * @author   Giuseppe Rizzi
 * @date     21.10.2020
 * @version  1.0
 * @brief    description
 */

#pragma once

// std
#include <cmath>
#include <memory>
#include <map>

// Ros
#include <ros/ros.h>
#include <angles/angles.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

// ROS Control
#include <controller_manager/controller_manager.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_mode_interface.h>
#include <hardware_interface/joint_state_interface.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

// ROS tools
#include <control_toolbox/pid.h>
#include <realtime_tools/realtime_publisher.h>

// Kinova
#include <kortex_driver/non-generated/kortex_arm_driver.h>
#include <kinova_robot/command_interface.h>

using namespace Kinova::Api;
using namespace Kinova::Api::BaseCyclic;
using namespace Kinova::Api::Base;

namespace hardware_interface
{

class KinovaHardwareInterface : public hardware_interface::RobotHW, KortexArmDriver
{
 public:
  KinovaHardwareInterface() = delete;
  KinovaHardwareInterface(ros::NodeHandle& nh);
  ~KinovaHardwareInterface();
  void run();

  ros::Time get_time();
  ros::Duration get_period();

 private:
  void read_loop(const double rate);
  void write_loop();

  void read();
  void write(const double dt);
  void update();

  /**
   * Set the servoing mode. There are two servoing modes:
   * -
   * -
   */
  bool set_servoing_mode(Kinova::Api::Base::ServoingMode servoing_mode);

  /**
   * @brief Set the actuator low level control mode
   * @param mode
   * @return
   */
  bool set_actuators_control_mode(KinovaControlMode new_mode);

  /**
   * @brief Send lowlevel command to the hardware
   * @return
   */
  bool send_lowlevel_command();

  /**
   * @brief Send a highlevel joint velocity command
   * @return
   */
  bool send_joint_velocity_command();

  /**
   * @brief Set the hardware command from the command copy and adds feedforward pid torque
   * @param dt the time elapsed since last call (sec)
   */
  void set_hardware_command(const double dt, bool same_as_readings=false);


  /**
   * @brief Copy ros commands to hardware commands variable
   */
  void copy_commands();

  /**
   * @brief Dedicated function to switch between modes
   */
  void switch_mode();

  /**
   * @brief Set the joint limits from parameter file
   * @return false if unable to set the joint limits
   */
  bool set_joint_limits();

  /**
   * @brief Enforce limits and additional postprocessing
   * @return
   */
  void enforce_limits();

  /**
   * @brief Make sure the state is within the limits. If not, triggers the estop
   */
  void check_state();

  /**
   * @brief Publish commands as sensor_msgs/JointState in a realtime safe manner
   */
  void publish_commands();

  /**
   * @brief Publish state as sensor_msgs/JointState in a realtime safe manner
   */
  void publish_state();

  /**
   * @brief Returns the angle for a continuous joint when a_next is in the range [-PI, PI]
   * @param a_prev the previous continuous joint angle [-inf, inf]
   * @param a_next the next reading for the joint angle [-PI, PI]
   * @return the wrapped angle
   */
  double wrap_angle(const double a_prev, const double a_next) const;

  /**
   * @brief Init low level pid controllers
   * @return false if failed to initialize the lowlevel controller
   */
  bool init_pid();

 private:
  ros::Time last_time_;
  controller_manager::ControllerManager* cm_;

  hardware_interface::KinovaCommandInterface jnt_cmd_interface_;
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface gripper_cmd_interface_;

  std::vector<std::string> joint_names_;

  double pos_[7];                         // [-PI, PI] converted readings from actuators
  double pos_cmd_[7];                     // [-PI, PI] converted commands to actuators
  double pos_cmd_copy_[7];                // intermediate variable to reduce locking time
  double pos_wrapped_[7];                 // position reading with continuous revolution for continuous joints

  double vel_[7];
  double vel_cmd_[7];
  double vel_cmd_copy_[7];

  double eff_[7];
  double eff_cmd_[7];
  double eff_cmd_copy_[7];

  double pos_error_[7];
  double vel_error_[7];
  std::vector<control_toolbox::Pid> pid_;

  // gripper state and command
  double gripper_position_;
  double gripper_velocity_;
  double gripper_force_;

  double gripper_position_error_;
  double gripper_position_command_;
  double gripper_velocity_command_;
  double gripper_force_command_;

  std::vector<joint_limits_interface::JointLimits> limits_;

  hardware_interface::KinovaControlMode mode_;
  hardware_interface::KinovaControlMode current_mode_;

  // multi-threading
  std::atomic_bool stop_writing_;    // when switching back to highlevel we need to stop the concurrent write
  std::mutex cmd_mutex_;

  std::thread write_thread_;
  std::thread read_update_thread_;

  Feedback current_state_;
  Kinova::Api::BaseCyclic::Command kortex_cmd_;
  Kinova::Api::GripperCyclic::MotorCommand* kortex_gripper_cmd_;
  Kinova::Api::Base::ServoingMode current_servoing_mode_;

  // publish state and command
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> realtime_state_pub_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> realtime_command_pub_;

  // IMU
  sensor_msgs::Imu imu_;
  realtime_tools::RealtimePublisher<sensor_msgs::Imu> realtime_imu_pub_;

  ros::Time last_publish_time_;
  double publish_rate_;

  bool initialized_;
  bool estopped_ = false;
  ros::ServiceClient estop_client_;

  Kinova::Api::Base::JointSpeeds kortex_joint_speeds_cmd_;
};
}

std::ostream& operator<<(std::ostream& os, const joint_limits_interface::JointLimits& limits);
