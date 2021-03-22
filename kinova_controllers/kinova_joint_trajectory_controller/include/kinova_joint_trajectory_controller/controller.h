#pragma once

#include <kinova_robot/command_interface.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <sensor_msgs/JointState.h>

#include "kinova_joint_trajectory_controller/trajectory_generator.h"

#include <mutex>

namespace kinova_controllers {

class KinovaJointTrajectoryController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::JointStateInterface, hardware_interface::KinovaCommandInterface> {
 public:
  using BASE = controller_interface::MultiInterfaceController<hardware_interface::JointStateInterface, hardware_interface::KinovaCommandInterface>;
  
  // not all interfaces are mandatory
  KinovaJointTrajectoryController() : BASE(true) {};
  ~KinovaJointTrajectoryController() = default;

 private:
  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

  void joint_callback(const sensor_msgs::JointStateConstPtr& msg);

 private:
  std::vector<std::string> joint_names_;
  std::array<hardware_interface::KinovaCommandHandle, 7> command_handles_;
  
  std::mutex generator_mutex_;
  TrajectoryGenerator generator_;

  std::atomic_bool trajectory_available_;
  ros::Subscriber trajectory_subscriber_;
  
  double gain_;
  Eigen::VectorXd joint_desired_;
  Eigen::VectorXd joint_current_;

  double tolerance_;
  double max_velocity_;
  std::vector<double> lower_limit_;
  std::vector<double> upper_limit_;
};
}  // namespace kinova_controllers
