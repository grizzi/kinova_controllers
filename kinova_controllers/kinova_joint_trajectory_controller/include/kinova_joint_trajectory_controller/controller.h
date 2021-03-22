#pragma once

#include <kinova_robot/command_interface.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>

#include "kinova_joint_trajectory_controller/trajectory_generator.h"
#include "kinova_joint_trajectory_controller/JointAction.h"

#include <mutex>

namespace kinova_controllers {

class KinovaJointTrajectoryController
    : public controller_interface::MultiInterfaceController<
          hardware_interface::JointStateInterface, hardware_interface::KinovaCommandInterface> {
 public:
  using BASE = controller_interface::MultiInterfaceController<hardware_interface::JointStateInterface, hardware_interface::KinovaCommandInterface>;
  using ActionServer = actionlib::SimpleActionServer<kinova_joint_trajectory_controller::JointAction>;

  // not all interfaces are mandatory
  KinovaJointTrajectoryController() : BASE(true) {};
  ~KinovaJointTrajectoryController() = default;

 private:
  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

  void compute_profile(const Eigen::VectorXd& goal);
  void joint_callback(const sensor_msgs::JointStateConstPtr& msg);
  void execute_callback(const kinova_joint_trajectory_controller::JointGoalConstPtr& goal);

 private:
  std::vector<std::string> joint_names_;
  std::array<hardware_interface::KinovaCommandHandle, 7> command_handles_;
  
  std::mutex generator_mutex_;
  std::unique_ptr<TrajectoryGenerator> generator_;

  std::atomic_bool trajectory_available_;
  ros::Subscriber trajectory_subscriber_;
  
  double gain_;
  Eigen::VectorXd joint_desired_;
  Eigen::VectorXd joint_current_;

  double tolerance_;
  double max_velocity_;
  std::vector<double> lower_limit_;
  std::vector<double> upper_limit_;

  std::atomic_bool success_ = false;
  std::unique_ptr<ActionServer> action_server_;
};
}  // namespace kinova_controllers
