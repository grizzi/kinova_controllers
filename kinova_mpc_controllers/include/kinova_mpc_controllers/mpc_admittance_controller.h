//
// Created by giuseppe on 24.01.21.
//

#include <geometry_msgs/WrenchStamped.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_srvs/Empty.h>
#include "kinova_mpc_controllers/mpc_controller.h"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <realtime_tools/realtime_publisher.h>

#pragma once

namespace kinova_controllers {

class MPC_AdmittanceController : public MPC_Controller {
 public:
  MPC_AdmittanceController() = delete;
  explicit MPC_AdmittanceController(const ros::NodeHandle& nh);
  ~MPC_AdmittanceController() = default;

 private:
  void adjustPath(nav_msgs::Path& desiredPath) const override;
  void wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg);
  bool reset_wrench_offset_callback(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);

  // TODO(giuseppe) for now this just trigger 0 desired wrench
  bool set_desired_wrench(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);

  /**
   * Utility function to parse a vector of 3 gains
   * @param nh: ros node handle
   * @param name: name of the gains on the parameter server
   * @param gains: the vector object
   */
  template <int N>
  void parse_vector(ros::NodeHandle& nh, const std::string& name,
                    Eigen::Matrix<double, N, 1>& gains);

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
  Eigen::Vector3d Kp_angular_;
  Eigen::Vector3d Ki_linear_;
  Eigen::Vector3d Ki_angular_;

  // Errors
  mutable double last_time_;
  mutable Eigen::Vector3d force_error_;
  mutable Eigen::Vector3d force_integral_;
  mutable Eigen::Vector3d torque_error_;
  mutable Eigen::Vector3d torque_integral_;
  Eigen::Vector3d force_integral_max_;
  Eigen::Vector3d torque_integral_max_;

  // TF
  std::string sensor_frame_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::Buffer tf_buffer_;

  bool active_;
};

}  // namespace kinova_controllers
