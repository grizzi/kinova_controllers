/*!
 * @file     task_space_controller.h
 * @author   Giuseppe Rizzi
 * @date     24.10.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <robot_control/modeling/robot_wrapper.h>
#include <robot_control/controllers/end_effector_controllers/task_space_controller.h>

#include <geometry_msgs/PoseStamped.h>
#include <realtime_tools/realtime_publisher.h>

#include <kortex_driver/non-generated/kortex_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>

#include <interactive_markers/interactive_marker_server.h>

namespace kinova_controllers {

template<class CommandInterface, class CommandHandle>
class KinovaTaskSpaceController : public controller_interface::Controller<CommandInterface> {
 public:
  ~KinovaTaskSpaceController();

  bool init(CommandInterface* robot_hw,
      ros::NodeHandle& node_handle,
      ros::NodeHandle& ctrl_handle) override;

  void newTargetCallback(const geometry_msgs::PoseStamped&);

  void publishRos();
  void publish_thread();

  void getJointPositions(Eigen::VectorXd& q) const;
  void getJointVelocities(Eigen::VectorXd& qd) const;

 protected:
  const int dof = 7;
  Eigen::VectorXd q_;
  Eigen::VectorXd qd_;
  Eigen::VectorXd cmd_;

  std::shared_ptr<rc::RobotWrapper> robot_wrapper;
  std::shared_ptr<rc::TaskSpaceController> controller;

  std::vector<hardware_interface::JointStateHandle> state_handles_;
  std::vector<CommandHandle> joint_handles_;

  std::string joint_names_[7] = {"arm_joint_1",
                                 "arm_joint_2",
                                 "arm_joint_3",
                                 "arm_joint_4",
                                 "arm_joint_5",
                                 "arm_joint_6",
                                 "arm_joint_7"};

  std::string frame_id_ = "world";
  std::string controlled_frame_;
  pin::SE3 target_pose_;
  pin::SE3 current_pose_;

  ros::Subscriber target_subscriber_;

  std::atomic_bool started_;
  bool publish_ros_;
  double publish_ros_rate_;
  std::thread publish_thread_;
  std::unique_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped>> pose_publisher_;

};

 class KinovaTaskSpaceControllerRobot : public KinovaTaskSpaceController<hardware_interface::KortexCommandInterface, hardware_interface::KortexCommandHandle> {
  public:
   void update(const ros::Time&, const ros::Duration& period) override;
   void starting(const ros::Time& time) override;
};

 class KinovaTaskSpaceControllerSim : public KinovaTaskSpaceController<hardware_interface::EffortJointInterface, hardware_interface::JointHandle> {
  public:
   void update(const ros::Time&, const ros::Duration& period) override;
   void starting(const ros::Time& time) override;
};
}
