//
// Created by giuseppe on 24.01.21.
//

#pragma once
#include <Eigen/Core>

namespace ft_sensor_utils {

struct Wrench {
  Wrench() {
    force.setZero();
    torque.setZero();
  }
  Wrench(const Eigen::Vector3d& f, const Eigen::Vector3d& t) : force(f), torque(t){};
  Wrench(const Wrench& rhs) {
    force = rhs.force;
    torque = rhs.torque;
  }

  Eigen::Vector3d force;
  Eigen::Vector3d torque;

  inline const Eigen::Vector3d& get_force() const { return force; }
  inline const Eigen::Vector3d& get_torque() const { return torque; }
  inline Eigen::Vector3d& get_force() { return force; }
  inline Eigen::Vector3d& get_torque() { return torque; }

  friend std::ostream& operator<<(std::ostream& os, const Wrench&);
};
/**
 * Estimate the external wrench applied by a payload in the sensor frame
 * @param payload_mass
 * @param gravity: gravity vector expressed in the sensor frame
 * @param paylod_offset: position of the COM of the payload wrt to the sensor
 * @return
 */
Wrench get_bias(const double& payload_mass, const Eigen::Vector3d& paylod_offset,
                const Eigen::Vector3d& gravity);

}  // namespace ft_sensor_utils