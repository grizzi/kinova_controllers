#pragma once

#include <kinova_robot/command_interface.h>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>

namespace kinova_controllers {

class KinovaJointVelocityController : public controller_interface::MultiInterfaceController<
                                        hardware_interface::JointStateInterface> {
 public:
  KinovaJointVelocityController() = default;
  ~KinovaJointVelocityController() = default;

 private:
  bool init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void starting(const ros::Time& time) override;
  void update(const ros::Time& time, const ros::Duration& period) override;
  void stopping(const ros::Time& time) override;

 private:
  std::vector<std::string> joint_names_;
  std::array<hardware_interface::KinovaCommandHandle, 7> command_handles_;
};
}  // namespace kinova_controllers
