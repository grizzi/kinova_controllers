#pragma once

/*!
 * @file     command_interface.h
 * @author   Giuseppe Rizzi
 * @date     24.10.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>

namespace hardware_interface {

/**
 * Handle to read and command the arm with different modes
 */
enum struct KinovaControlMode { NO_MODE = -1, POSITION = 0, VELOCITY = 1, EFFORT = 2 };

class KinovaCommandHandle : public hardware_interface::JointHandle {
 public:
  using mode_t = KinovaControlMode;
  KinovaCommandHandle(){};

  /**
   * Creates an instance of a KinovaCommandHandle.
   *
   * @param[in] joint_state_handle state handle.
   * @param[in] command A reference to the joint command wrapped by this handle.
   * @param[in] mode A reference to the command mode wrapped by this handle.
   */
  KinovaCommandHandle(const JointStateHandle& joint_state_handle, double* pos, double* vel,
                      double* eff, mode_t* mode)
      : JointHandle(joint_state_handle, cmd_), pos_(pos), vel_(vel), eff_(eff), mode_(mode) {}

  /**
   * Set the current mode
   *
   * @param[in] command Command to set.
   */
  void setMode(const mode_t mode);

  /**
   * Sets the given command for the current mode.
   *
   * @param[in] command Command to set.
   */

  void setCommand(const double cmd){
    if (*mode_ == KinovaControlMode::EFFORT)
      *eff_ = cmd;
    else if (*mode_ == KinovaControlMode::POSITION)
      *pos_ = cmd;
    else if (*mode_ == KinovaControlMode::VELOCITY)
      *vel_ = cmd;
  }

  /**
   * Gets the given command for the current mode.
   *
   * @param[in] command Command to set.
   */
  double getCommand() {
    if (*mode_ == KinovaControlMode::EFFORT)
      return *eff_;
    else if (*mode_ == KinovaControlMode::EFFORT)
      return *pos_;
    else if (*mode_ == KinovaControlMode::VELOCITY)
      return *vel_;
  }

  /**
   * Sets the given command.
   *
   * @param[in] command Command to set.
   */
  void setPositionCommand(double cmd) noexcept {
    *pos_ = cmd;
    *cmd_ = cmd;
  }
  void setVelocityCommand(double cmd) noexcept {  
    *vel_ = cmd;
    *cmd_ = cmd;
  }
  void setEffortCommand(double cmd) noexcept {
    *eff_ = cmd;
    *cmd_ = cmd;
  }

  /**
   * Gets the current command.
   *
   * @return Current command.
   */
  const double getPositionCommand() const noexcept { return *pos_; }
  const double getVelocityCommand() const noexcept { return *vel_; }
  const double getEffortCommand() const noexcept { return *eff_; }
  const mode_t getMode() const noexcept { return *mode_; }

 private:
  double* cmd_;
  double* pos_;
  double* vel_;
  double* eff_;
  mode_t* mode_;
};

/**
 * Hardware interface to command joints
 */
class KinovaCommandInterface
    : public hardware_interface::HardwareResourceManager<KinovaCommandHandle,
        hardware_interface::ClaimResources> {};

}  // namespace hardware_interface

std::ostream& operator<<(std::ostream& os, const hardware_interface::KinovaControlMode& mode);
