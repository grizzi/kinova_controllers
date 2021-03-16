//
// Created by giuseppe on 27.01.21.
//

#include "kinova_reflex_controller/reflex_controller.h"
#include <kortex_driver/ApplyEmergencyStop.h>
#include <Eigen/Core>
#include <pluginlib/class_list_macros.hpp>

using namespace kinova_controllers;

bool ReflexController::init(hardware_interface::ForceTorqueSensorInterface* hw,
                            ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  // get all joint states from the hardware interface
  const std::vector<std::string>& sensor_names = hw->getNames();
  for (unsigned i = 0; i < sensor_names.size(); i++)
    ROS_INFO("Got sensor %s", sensor_names[i].c_str());
  if (sensor_names.empty()) {
    ROS_WARN_STREAM("No force torque sensor found. ");
    return false;
  }

  if (sensor_names.size() > 1) ROS_WARN_STREAM("Found more than 1 ft sensor. Only one supported.");
  sensor_ = hw->getHandle(sensor_names[0]);

  // get limits
  if (!controller_nh.getParam("force_max", force_max_)) {
    ROS_ERROR("Parameter 'force_max' not set");
    return false;
  }

  if (!controller_nh.getParam("torque_max", torque_max_)) {
    ROS_ERROR("Parameter 'torque_max' not set");
    return false;
  }

  if (torque_max_ < 0 || force_max_ < 0) {
    ROS_ERROR_STREAM("Parameter force_max and torque_max are invalid: " << force_max_ << ", "
                                                                        << torque_max_);
  }

  ROS_INFO_STREAM("Using threshold f_max=" << force_max_ <<", t_max:=" << torque_max_);

  // get publishing period
  std::string estop_service_name;
  if (!controller_nh.getParam("estop_service_name", estop_service_name)) {
    ROS_ERROR("Parameter 'estop_service_name' not set");
    return false;
  }
  estop_client_ = root_nh.serviceClient<kortex_driver::ApplyEmergencyStop>(estop_service_name);
  if (!estop_client_.waitForExistence(ros::Duration(5.0))) {
    ROS_ERROR_STREAM("Failed to connect to the estop service: " << estop_service_name);
    return false;
  }

  estopped_ = false;
  return true;
}

void ReflexController::update(const ros::Time& time, const ros::Duration& /*period*/) {
  // check norm of force and torque
  const double* force = sensor_.getForce();
  const double* torque = sensor_.getTorque();

  Eigen::Vector3d fv(force[0], force[1], force[2]);
  Eigen::Vector3d tv(torque[0], torque[1], torque[2]);

  double force_norm = fv.norm();
  double torque_norm = tv.norm();

  ROS_INFO_STREAM_THROTTLE(5.0, "[ReflexController::update]: Current force norm is: " << force_norm);
  ROS_INFO_STREAM_THROTTLE(5.0, "[ReflexController::update]: Current torque norm is: " << torque_norm);
  
  if (force_norm > force_max_ && !estopped_) {
    ROS_ERROR_STREAM("Force exceeded force max: " << force_norm << " > " << force_max_);
    kortex_driver::ApplyEmergencyStopRequest req;
    kortex_driver::ApplyEmergencyStopResponse res;
    estop_client_.call(req, res);
    estopped_ = true;
  }

  if (torque_norm > torque_max_  && !estopped_) {
    ROS_ERROR_STREAM("Force exceeded force max: " << torque_norm << " > " << torque_max_);
    kortex_driver::ApplyEmergencyStopRequest req;
    kortex_driver::ApplyEmergencyStopResponse res;
    estop_client_.call(req, res);
    estopped_ = true;
  }
}

PLUGINLIB_EXPORT_CLASS(kinova_controllers::ReflexController, controller_interface::ControllerBase)