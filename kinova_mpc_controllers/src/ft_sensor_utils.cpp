//
// Created by giuseppe on 24.01.21.
//

#include "kinova_mpc_controllers/ft_sensor_utils.h"

#include <pinocchio/fwd.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/spatial/se3.hpp>

using namespace ft_sensor_utils;

/**
 * Estimate the external wrench applied by a payload in the sensor frame
 * @param payload_mass
 * @param gravity: gravity vector expressed in the sensor frame
 * @param paylod_offset: position of the COM of the payload wrt to the sensor
 * @return
 */
Wrench get_bias(const double& payload_mass, const Eigen::Vector3d& paylod_offset,
                const Eigen::Vector3d& gravity) {
  pinocchio::SE3 offset = pinocchio::SE3::Identity();
  offset.translation() = paylod_offset;

  pinocchio::Force payload = pinocchio::Force::Zero();
  payload.linear() = gravity;

  pinocchio::Force bias = offset.act(payload);
  Wrench wrench(bias.linear(), bias.angular());
  return wrench;
}

std::ostream& operator<<(std::ostream& os, const Wrench& wrench) {
  os << "Force: " << wrench.get_force().transpose()
     << ", Torque: " << wrench.get_torque().transpose();
  return os;
}