//
// Created by giuseppe on 24.01.21.
//

#include <ros/subscribe_options.h>
#include <ros/callback_queue.h>
#include "kinova_mpc_controllers/mpc_controller.h"
#include <geometry_msgs/WrenchStamped.h>
#include <std_srvs/Empty.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <realtime_tools/realtime_publisher.h>

#pragma once

namespace kinova_controllers {

class MPC_AdmittanceController : public MPC_Controller{
 public:
  MPC_AdmittanceController() = delete;
  explicit MPC_AdmittanceController(const ros::NodeHandle& nh);
  ~MPC_AdmittanceController() = default;

 private:
  void adjustPath(nav_msgs::Path& desiredPath) const override;
  void wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg);
  bool reset_wrench_offset_callback(std_srvs::EmptyRequest&, std_srvs::EmptyResponse& );

  // TODO(giuseppe) for now this just trigger 0 desired wrench
  bool set_desired_wrench(std_srvs::EmptyRequest&, std_srvs::EmptyResponse& );

 private:
  // ROS
  std::unique_ptr<ros::CallbackQueue> wrench_callback_queue_;
  ros::Subscriber wrench_subscriber_;
  ros::ServiceServer reset_wrench_offset_service_;
  ros::ServiceServer desired_wrench_service_;

  std::mutex wrench_mutex_;
  Eigen::Matrix<double, 6, 1> wrench_offset_;
  geometry_msgs::WrenchStamped wrench_;

  // Gains
  Eigen::Vector3d Kp_linear_;
  Eigen::Vector3d kp_angular_;

  // TF
  std::string sensor_frame_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;

  bool active_;
};


}
