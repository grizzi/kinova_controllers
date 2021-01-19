//
// Created by giuseppe on 06.01.21.
//

#include "kinova_mpc_controllers/kinova_mpc_velocity_controller.h"
#include <angles/angles.h>
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

  reset_imarker_pose_pub_ = root_nh.advertise<geometry_msgs::Pose>("/reset_marker_pose", 1);
  joint_state_des_pub_ = root_nh.advertise<sensor_msgs::JointState>("/mpc_joint_state_desired", 1);
  joint_state_cur_pub_ = root_nh.advertise<sensor_msgs::JointState>("/mpc_joint_state_current", 1);
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
    // Reset interactive marker pose (if running)
    geometry_msgs::Pose marker_pose;
    marker_pose.position.x = tool_pose.translation()(0);
    marker_pose.position.y = tool_pose.translation()(1);
    marker_pose.position.z = tool_pose.translation()(2);
    Eigen::Quaterniond orientation(tool_pose.rotation());
    marker_pose.orientation.x = orientation.x();
    marker_pose.orientation.y = orientation.y();
    marker_pose.orientation.z = orientation.z();
    marker_pose.orientation.w = orientation.w();
    reset_imarker_pose_pub_.publish(marker_pose);
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
  ROS_INFO("Stopping KinovaMpcVelocityController has been stopped!");
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
    joint_state_cur_.name.push_back(joint_names_[i]);
    joint_state_des_.name.push_back(joint_names_[i]);
  }
  joint_state_cur_.position.resize(7);
  joint_state_cur_.velocity.resize(7);
  joint_state_des_.position.resize(7);
  joint_state_des_.velocity.resize(7);
  joint_state_des_.effort.resize(7);
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
    joint_state_cur_.position[i] = position_current_(i);
    joint_state_cur_.velocity[i] = velocity_current_(i);
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

    model_->updateState(position_current_, Eigen::VectorXd::Zero(model_->getDof()));
    model_->computeAllTerms();
    gravity_and_coriolis_ = model_->getNonLinearTerms().head<7>();

    // tau = ff + pd
    position_error_ = position_command - position_current_.head<7>();
    velocity_error_ = velocity_command - velocity_current_.head<7>();

    ROS_DEBUG_STREAM_THROTTLE(1.0, std::endl
                                      << "Pos cmd: " << position_command.transpose() << std::endl
                                      << "Pos mes: " << position_current_.transpose() << std::endl
                                      << "Vel cmd: " << velocity_command.transpose() << std::endl
                                      << "Vel mes: " << velocity_current_.transpose());

    for (size_t i = 0; i < 7; i++) {
      double tau =
          pid_controllers_[i].computeCommand(position_error_(i), velocity_error_(i), period) +
          gravity_and_coriolis_(i);
      sim_command_handles_[i].setCommand(tau);
      joint_state_des_.position[i] = position_command(i);
      joint_state_des_.velocity[i] = velocity_command(i);
      joint_state_des_.effort[i] = tau;
    }
    joint_state_cur_pub_.publish(joint_state_cur_);
    joint_state_des_pub_.publish(joint_state_des_);
  }
}

}  // namespace kinova_controllers

PLUGINLIB_EXPORT_CLASS(kinova_controllers::KinovaMpcVelocityController,
                       controller_interface::ControllerBase)