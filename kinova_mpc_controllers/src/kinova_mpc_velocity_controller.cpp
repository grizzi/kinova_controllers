//
// Created by giuseppe on 06.01.21.
//

#include "kinova_mpc_controllers/kinova_mpc_velocity_controller.h"
#include <pluginlib/class_list_macros.h>

using namespace hardware_interface;

namespace kinova_controllers {

bool KinovaMpcVelocityController::init(hardware_interface::RobotHW* hw, ros::NodeHandle& root_nh,
                                       ros::NodeHandle& controller_nh) {
  controller_nh.param<std::vector<std::string>>("joint_names", joint_names_, {});
  if (joint_names_.size() != 7) {
    ROS_ERROR("Joint names must be 7.");
    return false;
  }

  bool simulation;
  controller_nh.param<bool>("simulation", simulation, {});
  is_real_robot_ = !simulation;
  ROS_INFO_STREAM("Is real robot? " << is_real_robot_);

  // init model
  if (!root_nh.param<std::string>("/my_gen3/robot_description", robot_description_, "")) {
    ROS_ERROR_STREAM("Could not find param /my_gen3/robot_description");
    return false;
  }
  model_ = std::unique_ptr<rc::RobotWrapper>(new rc::RobotWrapper());
  model_->initFromXml(robot_description_);
  position_current_ = Eigen::VectorXd::Zero(model_->getDof());
  velocity_current_ = Eigen::VectorXd::Zero(model_->getDof());

  if (!is_real_robot_) {
    for (size_t i = 0; i < 7; i++) {
      if (!pid_controllers_[i].init(ros::NodeHandle(controller_nh, "gains/" + joint_names_[i]),
                                    false)) {
        ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
        return false;
      }
      pid_controllers_[i].printValues();
    }
  }
  mpc_controller_ = std::unique_ptr<MPC_Controller>(new MPC_Controller(controller_nh));
  mpc_controller_->init();

  addStateHandles(hw);
  addCommandHandles(hw);
  return true;
}

void KinovaMpcVelocityController::starting(const ros::Time& time) {
  ROS_INFO("Starting KinovaMpcVelocityController!");
  readState();
  mpc_controller_->start(position_current_.head<7>());

  if (!is_real_robot_) {
    model_->updateState(position_current_, velocity_current_);
    std::string tool_frame_id = "tool_frame";
    pinocchio::SE3 tool_pose = model_->getFramePlacement(tool_frame_id);
  }
}

void KinovaMpcVelocityController::update(const ros::Time& time, const ros::Duration& period) {
  readState();
  mpc_controller_->update(time, position_current_.head<7>());
  writeCommand(period);
}

void KinovaMpcVelocityController::stopping(const ros::Time& time) {
  ROS_INFO("Stopping KinovaMpcVelocityController!");
  mpc_controller_->stop();
  if (is_real_robot_) {
    for (auto& handle : robot_command_handles_) handle.setCommand(0.0);
  }
}

bool KinovaMpcVelocityController::addStateHandles(hardware_interface::RobotHW* hw) {
  // only arm joints
  auto state_interface = hw->get<JointStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM("Can't get state interface");
    return false;
  }
  for (size_t i = 0; i < 7; i++) {
    state_handles_[i] = state_interface->getHandle(joint_names_[i]);
  }
  return true;
}

bool KinovaMpcVelocityController::addCommandHandles(hardware_interface::RobotHW* hw) {
  // only arm joints
  if (is_real_robot_) {
    auto command_interface = hw->get<KinovaCommandInterface>();
    if (command_interface == nullptr) {
      ROS_ERROR_STREAM("Can't get state interface");
      return false;
    }
    for (size_t i = 0; i < 7; i++) {
      robot_command_handles_[i] = command_interface->getHandle(joint_names_[i]);
    }
    return true;
  } else {
    auto command_interface = hw->get<EffortJointInterface>();
    if (command_interface == nullptr) {
      ROS_ERROR_STREAM("Can't get state interface");
      return false;
    }
    for (size_t i = 0; i < 7; i++) {
      sim_command_handles_[i] = command_interface->getHandle(joint_names_[i]);
    }
    return true;
  }
}

void KinovaMpcVelocityController::readState() {
  for (size_t i = 0; i < 7; i++) {
    position_current_(i) = state_handles_[i].getPosition();
    velocity_current_(i) = state_handles_[i].getVelocity();
  }
}
void KinovaMpcVelocityController::writeCommand(const ros::Duration& period) {
  if (is_real_robot_) {
    for (size_t i = 0; i < 7; i++) {
      robot_command_handles_[i].setMode(KinovaControlMode::VELOCITY);
      robot_command_handles_[i].setVelocityCommand(mpc_controller_->get_velocity_command()(i));
    }
  } else {
    Eigen::VectorXd position_command = mpc_controller_->get_position_command();
    Eigen::VectorXd velocity_command = mpc_controller_->get_velocity_command();

    // update current state
    readState();

    model_->updateState(position_current_, velocity_current_);
    model_->computeAllTerms();
    gravity_and_coriolis_ = model_->getNonLinearTerms().head<7>();

    // tau = ff + pd
    position_error_ = position_command - position_current_.head<7>();
    velocity_error_ = velocity_command - velocity_current_.head<7>();

    ROS_INFO_STREAM_THROTTLE(1.0, "Vel cmd: " << velocity_command.transpose() << std::endl
                                              << "Vel mes: " << velocity_current_.transpose());

    for (size_t i = 0; i < 7; i++) {
      sim_command_handles_[i].setCommand(
          pid_controllers_[i].computeCommand(position_error_(i), velocity_error_(i), period) +
          gravity_and_coriolis_(i));
    }
  }
}

}  // namespace kinova_controllers

PLUGINLIB_EXPORT_CLASS(kinova_controllers::KinovaMpcVelocityController,
                       controller_interface::ControllerBase)