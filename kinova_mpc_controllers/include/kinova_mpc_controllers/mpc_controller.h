//
// Created by giuseppe on 31.12.20.
//

#pragma once

#include <mutex>

#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_mobile_manipulator_example/MobileManipulatorInterface.h>

#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_publisher.h>

namespace kinova_controllers {

class MPC_Controller{
 public:
  using joint_vector_t = Eigen::Matrix<double, 7, 1>;

  MPC_Controller() = delete;
  explicit MPC_Controller(const ros::NodeHandle& nh);
  ~MPC_Controller() = default;

  bool init();
  void start(const joint_vector_t& initial_observation);
  void update(const ros::Time& time, const joint_vector_t& observation);
  void stop();
  /**
   * Updates the desired cost trajectories from path message.
   * @param desiredPath
   */
  void pathCallback(const nav_msgs::PathConstPtr& desiredPath);
  inline Eigen::VectorXd get_position_command() const { return positionCommand_; }
  inline Eigen::VectorXd get_velocity_command() const { return velocityCommand_; }
  inline const ros::NodeHandle& get_node_handle() { return nh_; }
  inline std::string get_description() { return robot_description_; }

 protected:

  /**
   * Optional preprocessing step for the tracked path.
   * @param desiredPath
   */
  virtual void adjustPath(nav_msgs::Path& desiredPath) const {};

 private:
  void advanceMpc();
  void setObservation(const joint_vector_t& q);
  void updateCommand();
  void publishObservation();

  void writeDesiredPath(const nav_msgs::Path& path);

  // path processing
  static bool sanityCheck(const nav_msgs::Path& path);
  void transformPath(nav_msgs::Path& desiredPath) const;
  void adjustPathTime(nav_msgs::Path& desiredPath) const;


 protected:
  double start_time_;
  std::string base_link_;
  std::string robot_description_;
  joint_vector_t jointInitialState_;
  joint_vector_t positionCommand_;
  joint_vector_t velocityCommand_;
  std::vector<std::string> jointNames_;

  ros::NodeHandle nh_;
  ros::Publisher observationPublisher_;
  ros::Publisher commandPublisher_;
  ros::Subscriber targetPathSubscriber_;

 private:
  std::atomic_bool stopped_;
  std::unique_ptr<ocs2::MPC_DDP> mpcPtr_;
  double mpcFrequency_;
  std::string taskFile_;

  std::atomic_bool policyReady_;
  std::atomic_bool referenceEverReceived_;
  std::atomic_bool observationEverReceived_;

  std::thread mpcThread_;
  ocs2::benchmark::RepeatedTimer mpcTimer_;

  std::mutex observationMutex_;
  ocs2::SystemObservation observation_;

  std::mutex desiredPathMutex_;
  nav_msgs::Path desiredPath_;

  std::unique_ptr<mobile_manipulator::MobileManipulatorInterface> mm_interface_;
  std::unique_ptr<ocs2::MPC_MRT_Interface> mpc_mrt_interface_;

  realtime_tools::RealtimePublisher<nav_msgs::Path> command_path_publisher_;
};
}  // namespace mobile_manipulator