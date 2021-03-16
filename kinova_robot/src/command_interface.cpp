//
// Created by giuseppe on 07.01.21.
//

#include "kinova_robot/command_interface.h"

namespace hardware_interface {

void KinovaCommandHandle::setMode(const mode_t mode) { 
  std::cout << __FILE__ << ",  " << __LINE__ << ", "<< mode << std::endl;
  *mode_ = mode; 
}

}

std::ostream& operator<<(std::ostream& os, const hardware_interface::KinovaControlMode& mode) {
  if (mode == hardware_interface::KinovaControlMode::NO_MODE)
    return os << " NO_MODE";
  else if (mode == hardware_interface::KinovaControlMode::POSITION)
    return os << " POSITION";
  else if (mode == hardware_interface::KinovaControlMode::VELOCITY)
    return os << " VELOCITY";
  else if (mode == hardware_interface::KinovaControlMode::EFFORT)
    return os << " EFFORT";
  else
    return os << " UNKNOWN (" << (int)mode << ")";
}
