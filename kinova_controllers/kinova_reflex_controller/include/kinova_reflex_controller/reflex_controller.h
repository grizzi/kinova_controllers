//
// Created by giuseppe on 27.01.21.
//

#pragma once
#include <controller_interface/controller.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/force_torque_sensor_interface.h>
#include <realtime_tools/realtime_publisher.h>
#include <memory>

namespace kinova_controllers {

// Controller using FT feedback to stop the robot motion triggering the emergency stop
class ReflexController
    : public controller_interface::Controller<hardware_interface::ForceTorqueSensorInterface> {
 public:
  ReflexController() : force_max_(0.0), torque_max_(0.0){};

  bool init(hardware_interface::ForceTorqueSensorInterface* hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& /*period*/) override;

  void starting(const ros::Time& time) override {};
  void stopping(const ros::Time& /*time*/) override {};

 private:
  hardware_interface::ForceTorqueSensorHandle sensor_;
  ros::ServiceClient estop_client_;

  double force_max_;
  double torque_max_;

  bool estopped_;
};

}  // namespace kinova_controllers