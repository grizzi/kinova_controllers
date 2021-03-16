#include "kinova_robot/hardware_interface.h"
#include <eigen_conversions/eigen_msg.h>

#define GRIPPER_MINIMAL_POSITION_ERROR 1.5

using namespace hardware_interface;

KinovaHardwareInterface::KinovaHardwareInterface(ros::NodeHandle& nh) : KortexArmDriver(nh) {
  initialized_ = true;

  // clear all faults (and set to highlevel servoing)
  m_base->ClearFaults();

  // init joint names
  joint_names_.resize(7);
  for (int i = 0; i < 7; i++) {
    joint_names_[i] = m_arm_joint_names[i];
  }


  ROS_INFO_STREAM("Starting Kinova Robot interface in namespace: " << nh.getNamespace());
  for (std::size_t i = 0; i < 7; ++i) {
    hardware_interface::JointStateHandle state_handle(joint_names_[i], &pos_wrapped_[i], &vel_[i],
                                                      &eff_[i]);
    jnt_state_interface_.registerHandle(state_handle);
    
    hardware_interface::KinovaCommandHandle cmd_handle(
        jnt_state_interface_.getHandle(joint_names_[i]), &pos_cmd_[i], &vel_cmd_[i], &eff_cmd_[i],
        &mode_);

    jnt_cmd_interface_.registerHandle(cmd_handle);
  }
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_cmd_interface_);

  if (isGripperPresent()) {
    hardware_interface::JointStateHandle state_handle("gripper", &gripper_position_,
                                                      &gripper_velocity_, &gripper_force_);
    
    hardware_interface::JointHandle gripper_handle(jnt_state_interface_.getHandle("gripper"),
                                                   &gripper_position_command_);
    gripper_cmd_interface_.registerHandle(gripper_handle);
    
    jnt_state_interface_.registerHandle(state_handle);
    registerInterface(&gripper_cmd_interface_);
  }

  bool limits_ok = set_joint_limits();
  if (!limits_ok) {
    ROS_ERROR("Failed to set the joint limits");
    initialized_ = false;
  }

  if (!init_pid()) {
    ROS_ERROR("Failed to initialize the PID controllers");
    initialized_ = false;
  }

  // at start up write to command the current state
  current_mode_ = KinovaControlMode::NO_MODE;
  current_servoing_mode_ = Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING;
  mode_ = current_mode_;
  mode_copy_ = current_mode_;

  // first read and fill command
  read();
  for (size_t i = 0; i < 7; i++) {
    pos_cmd_[i] = pos_[i];
    pos_wrapped_[i] = pos_[i];
    vel_cmd_[i] = 0.0;
    eff_cmd_[i] = eff_[i];
    pos_error_[i] = 0.0;
    vel_error_[i] = 0.0;
    kortex_cmd_.add_actuators();
  }
  copy_commands();

  realtime_state_pub_.init(m_node_handle, "/kinova_ros_control/joint_state", 4);
  realtime_command_pub_.init(m_node_handle, "/kinova_ros_control/joint_command", 4);
  realtime_imu_pub_.init(m_node_handle, "/kinova_ros_control/imu", 1);

  for (unsigned i = 0; i < 7; i++) {
    realtime_state_pub_.msg_.name.push_back(joint_names_[i]);
    realtime_state_pub_.msg_.position.push_back(pos_wrapped_[i]);
    realtime_state_pub_.msg_.velocity.push_back(vel_[i]);
    realtime_state_pub_.msg_.effort.push_back(eff_[i]);

    realtime_command_pub_.msg_.name.push_back(joint_names_[i]);
    realtime_command_pub_.msg_.position.push_back(pos_cmd_[i]);
    realtime_command_pub_.msg_.velocity.push_back(vel_cmd_[i]);
    realtime_command_pub_.msg_.effort.push_back(eff_cmd_[i]);
  }

  if (isGripperPresent()) {
    realtime_state_pub_.msg_.name.push_back("gripper");
    realtime_state_pub_.msg_.position.push_back(gripper_position_);
    realtime_state_pub_.msg_.velocity.push_back(gripper_velocity_);
    realtime_state_pub_.msg_.effort.push_back(gripper_force_);

    gripper_position_error_ = 0.0;
    gripper_position_command_ = gripper_position_;
    gripper_velocity_command_ = gripper_velocity_;
    gripper_force_command_ = 10.0;  // TODO(giuseppe) hard coded -> could be set
    kortex_gripper_cmd_ =
        kortex_cmd_.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
    kortex_gripper_cmd_->set_position(gripper_position_command_);
    kortex_gripper_cmd_->set_velocity(gripper_velocity_command_);
    kortex_gripper_cmd_->set_force(gripper_force_command_);

    realtime_command_pub_.msg_.name.push_back("gripper");
    realtime_command_pub_.msg_.position.push_back(gripper_position_command_);
    realtime_command_pub_.msg_.velocity.push_back(gripper_velocity_command_);
    realtime_command_pub_.msg_.effort.push_back(gripper_force_command_);
  }

  stop_writing_ = false;
  std::string emergency_stop_service_name = "/my_gen3/base/apply_emergency_stop";
  estop_client_ =
      m_node_handle.serviceClient<kortex_driver::ApplyEmergencyStop>(emergency_stop_service_name);
  if (!estop_client_.waitForExistence(ros::Duration(10.0))) {
    ROS_ERROR_STREAM("Could not contact service: " << emergency_stop_service_name);
    initialized_ = false;
  }

  last_time_ = ros::Time::now();
  cm_ = new controller_manager::ControllerManager(&*this);
}

KinovaHardwareInterface::~KinovaHardwareInterface() {
  if (write_thread_.joinable()) {
    write_thread_.join();
  }

  set_actuators_control_mode(KinovaControlMode::VELOCITY);

  if (read_update_thread_.joinable()) read_update_thread_.join();
}

bool KinovaHardwareInterface::init_pid() {
  pid_.resize(7);
  for (size_t i = 0; i < 7; i++) {
    const ros::NodeHandle nh(m_node_handle.getNamespace() + "/arm_pid/" + joint_names_[i]);
    if (!pid_[i].init(nh, false)) {
      ROS_ERROR_STREAM("Failed to initialize PID controller for " << joint_names_[i]);
      return false;
    }
  }
  return true;
}

bool KinovaHardwareInterface::set_joint_limits() {
  bool limits_ok = true;
  limits_.resize(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); i++) {
    limits_ok &= joint_limits_interface::getJointLimits(joint_names_[i], m_node_handle, limits_[i]);
  }
  if (!limits_ok) {
    return false;
  }
  std::stringstream joint_limits_string;
  for (size_t i = 0; i < joint_names_.size(); i++) {
    joint_limits_string << "Limits " << joint_names_[i] << ": " << limits_[i] << std::endl;
  }
  ROS_INFO_STREAM(joint_limits_string.str());
  return true;
}

/// read loop functions
/// keep consistency with simulation: angle in range [-PI, PI] and unlimited continuous joints
void KinovaHardwareInterface::read() {

  // in low-level mode sending the command also returns the current state
  if (current_mode_ == KinovaControlMode::VELOCITY || current_mode_ == KinovaControlMode::NO_MODE)
    current_state_ = m_base_cyclic->RefreshFeedback();

  for (int i = 0; i < current_state_.actuators_size(); i++) {
    std::shared_lock<std::shared_mutex> lock(m_zero_position_mutex);
    pos_[i] = angles::normalize_angle(
        static_cast<double>(angles::from_degrees(current_state_.actuators(i).position())));

    // wrap angle for continuous joints (the even ones) and account for bias
    pos_wrapped_[i] = ((i % 2) == 0) ? wrap_angle(pos_wrapped_[i], pos_[i]) : pos_[i];
    pos_wrapped_[i] -= m_zero_position[i];

    vel_[i] = static_cast<double>(angles::from_degrees(current_state_.actuators(i).velocity()));
    eff_[i] = static_cast<double>(-current_state_.actuators(i).torque());
  }

  imu_.header.frame_id = m_prefix + "interconnect_link";
  imu_.header.stamp = ros::Time::now();

  imu_.linear_acceleration.x = -current_state_.interconnect().imu_acceleration_x();  // IMu output not compliant with the RHR convention
  imu_.linear_acceleration.y = current_state_.interconnect().imu_acceleration_y();
  imu_.linear_acceleration.z = current_state_.interconnect().imu_acceleration_z();
  imu_.angular_velocity.x = current_state_.interconnect().imu_angular_velocity_x();
  imu_.angular_velocity.y = current_state_.interconnect().imu_angular_velocity_y();
  imu_.angular_velocity.z = current_state_.interconnect().imu_angular_velocity_z();

  if (isGripperPresent()) {
    gripper_position_ = current_state_.interconnect().gripper_feedback().motor()[0].position();
    gripper_velocity_ = current_state_.interconnect().gripper_feedback().motor()[0].velocity();
  }
}

void KinovaHardwareInterface::update() { cm_->update(this->get_time(), this->get_period()); }

void KinovaHardwareInterface::enforce_limits() {
  for (size_t i = 0; i < 7; i++) {
    pos_cmd_[i] = std::max(std::min(pos_cmd_[i], limits_[i].max_position), limits_[i].min_position);
    vel_cmd_[i] = std::max(std::min(vel_cmd_[i], limits_[i].max_velocity), -limits_[i].max_velocity);
    eff_cmd_[i] = std::max(std::min(eff_cmd_[i], limits_[i].max_effort), -limits_[i].max_effort);
  }

  gripper_position_command_ = std::max(std::min(gripper_position_command_, 100.0), 0.0);
}

void KinovaHardwareInterface::check_state() {
  bool ok = true;
  for (size_t i = 0; i < 7; i++) {
    ok &= !limits_[i].has_position_limits ||
          ((pos_wrapped_[i] < limits_[i].max_position) && (pos_wrapped_[i] > limits_[i].min_position));
    if (!ok) {
      ROS_ERROR_STREAM_THROTTLE(3.0,
                                "Joint " << i << " violated position limits: " << pos_wrapped_[i]);
      break;
    }

    // remove this hack and replace with soft limits
    ok &= !limits_[i].has_velocity_limits ||
          ((vel_[i] < (limits_[i].max_velocity + 0.1)) && (vel_[i] > -(limits_[i].max_velocity + 0.1)));
    if (!ok) {
      ROS_ERROR_STREAM_THROTTLE(3.0, "Joint " << i << " violated velocity limits: " << vel_[i]);
      break;
    }

    ok &= !limits_[i].has_effort_limits ||
          ((eff_[i] < limits_[i].max_effort) && (eff_[i] > -limits_[i].max_effort));
    if (!ok) {
      ROS_ERROR_STREAM_THROTTLE(3.0, "Joint " << i << " violated effort limits: " << eff_[i]);
      break;
    }
  }

  if (current_state_.base().active_state() == Kinova::Api::Common::ArmState::ARMSTATE_IN_FAULT){
    ROS_ERROR_ONCE("Arm is in fault state");
    stop_writing_ = true;
    estopped_ = true;
    return;
  }

  if (!ok && !estopped_) {
    stop_writing_ = true;
    kortex_driver::ApplyEmergencyStopRequest req;
    kortex_driver::ApplyEmergencyStopResponse res;
    estop_client_.call(req, res);
    estopped_ = true;
  }
}

void KinovaHardwareInterface::switch_mode() {
  if (mode_ != current_mode_) {
    ROS_INFO_STREAM("Switching to new mode: " << mode_);
    set_actuators_control_mode(mode_);
  }
}

void KinovaHardwareInterface::copy_commands() {
  std::lock_guard<std::mutex> lock(cmd_mutex_);
  mode_copy_ = current_mode_;
  for (size_t i = 0; i < 7; i++) {
    pos_cmd_copy_[i] = pos_[i];  // avoid following error
    vel_cmd_copy_[i] = vel_cmd_[i];
    eff_cmd_copy_[i] = eff_cmd_[i];

    pos_error_[i] = pos_cmd_[i] - pos_wrapped_[i];
    vel_error_[i] = -vel_[i];

    // if (mode_copy_ == KinovaControlMode::EFFORT_PID)
    //   eff_cmd_copy_[i] += pid_[i].computeCommand(pos_error_[i], vel_error_[i], ros::Duration(dt));  
  }

  if (isGripperPresent()) {
    gripper_position_error_ = gripper_position_command_ - gripper_position_;
  }
}

void KinovaHardwareInterface::publish_state() {
  if (realtime_state_pub_.trylock()) {
    realtime_state_pub_.msg_.header.stamp = ros::Time::now();
    for (unsigned i = 0; i < 7; i++) {
      realtime_state_pub_.msg_.position[i] = pos_wrapped_[i];
      realtime_state_pub_.msg_.velocity[i] = vel_[i];
      realtime_state_pub_.msg_.effort[i] = eff_[i];
    }

    if (isGripperPresent()) {
      realtime_state_pub_.msg_.position[7] = gripper_position_;
      realtime_state_pub_.msg_.velocity[7] = gripper_velocity_;
      realtime_state_pub_.msg_.effort[7] = gripper_force_;
    }
    realtime_state_pub_.unlockAndPublish();
  }

  if (realtime_imu_pub_.trylock()){
    realtime_imu_pub_.msg_ = imu_;
    realtime_imu_pub_.unlockAndPublish();
  }
}

void KinovaHardwareInterface::publish_commands() {
  if (realtime_command_pub_.trylock()) {
    realtime_command_pub_.msg_.header.stamp = ros::Time::now();
    for (unsigned i = 0; i < 7; i++) {
      realtime_command_pub_.msg_.position[i] = pos_cmd_[i];
      realtime_command_pub_.msg_.velocity[i] = vel_cmd_[i];
      realtime_command_pub_.msg_.effort[i] = eff_cmd_[i];
    }

    if (isGripperPresent()) {
      realtime_command_pub_.msg_.position[7] = gripper_position_command_;
      realtime_command_pub_.msg_.velocity[7] = gripper_velocity_command_;
      realtime_command_pub_.msg_.effort[7] = gripper_force_command_;
    }

    realtime_command_pub_.unlockAndPublish();
  }
}


void KinovaHardwareInterface::read_loop(const double f /* 1/sec */) {
  std::chrono::time_point<std::chrono::steady_clock> start, end;
  double dt_ms = 1000. / f;
  double elapsed_ms = dt_ms;
  while (ros::ok()) {
    start = std::chrono::steady_clock::now();

    // check the robot is safe
    check_state();

    // read the latest state
    read();

    // update the control commands
    update();
    
    // commands in the limits
    enforce_limits();

    // switch mode if required
    switch_mode();
    
    // copy ros commands to hardware commands
    copy_commands();
    
    // publish commands
    publish_commands();

    // publish state
    publish_state();

    
    end = std::chrono::steady_clock::now();
    elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1e6;
    if (elapsed_ms > dt_ms)
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Read and update took too long: " << elapsed_ms << " > " << dt_ms << " ms.");
    else {
      this_thread::sleep_for(std::chrono::nanoseconds((int)((dt_ms - elapsed_ms) * 1e6)));
    }
  }
}

void KinovaHardwareInterface::write_loop() {
  std::chrono::time_point<std::chrono::steady_clock> prev_start, start, end;
  double elapsed_ms;
  double dt_ms = 10.0;
  double dt = 0.0;
  prev_start = std::chrono::steady_clock::now();
  while (ros::ok()) {
    start = std::chrono::steady_clock::now();
    dt = std::chrono::duration_cast<std::chrono::nanoseconds>(start - prev_start).count() / 1e9;
    write(dt);

    end = std::chrono::steady_clock::now();
    elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1e6;
    if (elapsed_ms < dt_ms)
      std::this_thread::sleep_for(std::chrono::nanoseconds((int)((dt_ms - elapsed_ms) * 1e6)));
      
    prev_start = start;
  }
}

void KinovaHardwareInterface::write(const double dt) {
  if (stop_writing_) return;

  if (mode_copy_ == KinovaControlMode::POSITION || mode_copy_ == KinovaControlMode::EFFORT) 
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    set_hardware_command(dt);
    send_lowlevel_command();
  } 
  else if (mode_copy_ == KinovaControlMode::VELOCITY) 
  {
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        for (size_t i = 0; i < 7; ++i) {
          kortex_joint_speeds_cmd_.mutable_joint_speeds(i)->set_value(vel_cmd_copy_[i] * 180.0 / M_PI);
        }
    }

    // Do not lock when sending the command
    m_base->SendJointSpeedsCommand(kortex_joint_speeds_cmd_);

  } 
  else if (mode_copy_ == KinovaControlMode::NO_MODE) 
  {} 
  else 
  {
    ROS_WARN_STREAM("Unknown mode: " << mode_copy_);
  }
}

/// Start main threads
void KinovaHardwareInterface::run() {
  if (!initialized_) {
    ROS_ERROR_STREAM("Kinova Robot interface failed to initialize. Not running.");
    return;
  }

  write_thread_ = std::thread(&KinovaHardwareInterface::write_loop, this);
  read_update_thread_ = std::thread(&KinovaHardwareInterface::read_loop, this, 100);
  ROS_INFO_STREAM("Kinova Robot interface is running.");
}

/// Additional methods

bool KinovaHardwareInterface::set_servoing_mode(Kinova::Api::Base::ServoingMode new_mode) {
  bool success = false;
  Kinova::Api::Base::ServoingModeInformation servoing_mode;
  
  try {
    servoing_mode.set_servoing_mode(new_mode);
    m_base->SetServoingMode(servoing_mode);
    ROS_INFO("New servoing mode set.");
    current_servoing_mode_ = new_mode;
    success = true;
  } catch (Kinova::Api::KDetailedException& ex) {
    ROS_ERROR_STREAM_THROTTLE(5.0, "Kortex exception: " << ex.what());
    ROS_ERROR_STREAM_THROTTLE(
        5.0, "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(
                 Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  } catch (std::runtime_error& ex2) {
    ROS_ERROR_STREAM_THROTTLE(5.0, "runtime error: " << ex2.what());
  } catch (...) {
    ROS_ERROR_STREAM_THROTTLE(5.0, "Unknown error.");
  }
  return success;
}

bool KinovaHardwareInterface::set_actuators_control_mode(KinovaControlMode new_mode) {
  std::cout << "In set_actuators_control_mode" << std::endl;
  Kinova::Api::ActuatorConfig::ControlModeInformation control_mode_info;
  try {
    // SINGLE LEVEL SERVOING (aka high level position/velocity control)
    // switch happens in reverse mode: first actuator, then servoing mode
    if (new_mode == KinovaControlMode::NO_MODE || new_mode == KinovaControlMode::VELOCITY) {
      std::cout << "Here 0" << std::endl;
      if (current_mode_ == KinovaControlMode::VELOCITY && new_mode == KinovaControlMode::NO_MODE){
        current_mode_ = new_mode;
        ROS_INFO("Sending zero joint velocities when switching from VELOCITY to NO_MODE");
        for (size_t i = 0; i < 7; ++i){
          kortex_joint_speeds_cmd_.mutable_joint_speeds(i)->set_value(0.0);  
        } 
        m_base->SendJointSpeedsCommand(kortex_joint_speeds_cmd_);
        std::cout << "Here1" << std::endl;
        return true;
      }
      
      if (current_servoing_mode_ == Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING){
        std::cout << "Here3" << std::endl;
        current_mode_ = new_mode;
        std::cout << "Here4" << std::endl;
        ROS_INFO("Already in high servoing mode.");
        return true;
      }

      // set command same as the current measurement
      stop_writing_ = true;
      bool same_as_readings = true;
      set_hardware_command(0.0, same_as_readings);
      send_lowlevel_command();

      // switch actuators to position mode again
      control_mode_info.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
      for (size_t i = 1; i < 8; i++) m_actuator_config->SetControlMode(control_mode_info, i);

      // go to single level servoing mode
      if (!set_servoing_mode(Kinova::Api::Base::ServoingMode::SINGLE_LEVEL_SERVOING)) {
        return false;
      } else {
        ROS_INFO_STREAM("Mode switched to " << new_mode);
        current_mode_ = new_mode;
        return true;
      }
    }

    // LOW LEVEL SERVOING (position or torque)
    if (new_mode == KinovaControlMode::POSITION || new_mode == KinovaControlMode::EFFORT) {
      stop_writing_ = false;
      if (!set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING)) {
        return false;
      }
      bool same_as_readings = true;
      set_hardware_command(0.0, same_as_readings);
      send_lowlevel_command();
    }

    if (new_mode == KinovaControlMode::EFFORT) {
      control_mode_info.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::TORQUE);
    } else {
      control_mode_info.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
    }

    for (size_t i = 1; i < 8; i++) {
      m_actuator_config->SetControlMode(control_mode_info, i);
    }
    ROS_INFO_STREAM("Mode switched to " << new_mode);
    current_mode_ = new_mode;
    return true;
  } catch (Kinova::Api::KDetailedException& ex) {
    ROS_ERROR_STREAM("Kortex exception: " << ex.what());
    ROS_ERROR_STREAM(
        "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(
            Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
    return false;
  } catch (std::runtime_error& ex2) {
    ROS_ERROR_STREAM("runtime error: " << ex2.what());
    return false;
  } catch (...) {
    ROS_ERROR_STREAM("Unknown error.");
    return false;
  }
  return true;
}

void KinovaHardwareInterface::set_hardware_command(const double dt, bool same_as_readings) {
  kortex_cmd_.set_frame_id(kortex_cmd_.frame_id() + 1);  // unique-id to reject out-of-time frames
  if (kortex_cmd_.frame_id() > 65535) {
    kortex_cmd_.set_frame_id(0);
  }

  if (same_as_readings) {
    for (int idx = 0; idx < 7; idx++) {
      kortex_cmd_.mutable_actuators(idx)->set_command_id(kortex_cmd_.frame_id());
      kortex_cmd_.mutable_actuators(idx)->set_position(
          angles::to_degrees(angles::normalize_angle_positive(pos_[idx])));
      kortex_cmd_.mutable_actuators(idx)->set_torque_joint(eff_[idx]);
    }
  } else {
    for (int idx = 0; idx < 7; idx++) {
      kortex_cmd_.mutable_actuators(idx)->set_command_id(kortex_cmd_.frame_id());
      kortex_cmd_.mutable_actuators(idx)->set_position(
          angles::to_degrees(angles::normalize_angle_positive(pos_cmd_copy_[idx])));
      kortex_cmd_.mutable_actuators(idx)->set_torque_joint(eff_cmd_copy_[idx]);
    }
  }

  if (isGripperPresent()) {
    if (fabs(gripper_position_error_) < GRIPPER_MINIMAL_POSITION_ERROR) {
      gripper_velocity_command_ = 0.0;
    } else {
      gripper_velocity_command_ = std::min(std::max(2.0 * fabs(gripper_position_error_), 0.0), 100.0);
    }
    kortex_gripper_cmd_->set_position(gripper_position_command_);
    kortex_gripper_cmd_->set_velocity(gripper_velocity_command_);
    kortex_gripper_cmd_->set_force(gripper_force_command_);
  }
}

bool KinovaHardwareInterface::send_lowlevel_command() {
  bool success = false;
  try 
  {
    ROS_INFO_ONCE("Sending first low level command");
    auto start = std::chrono::steady_clock::now();
    current_state_ = m_base_cyclic->Refresh(kortex_cmd_, 0);
    auto end = std::chrono::steady_clock::now();
    double dt_refresh =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1e6;
    if (dt_refresh > 1.0)
      ROS_WARN_STREAM_THROTTLE(1.0, "Refresh command took :" << dt_refresh << " ms.");
    success = true;
  } 
  catch (Kinova::Api::KDetailedException& ex) 
  {
    ROS_ERROR_STREAM("Kortex exception: " << ex.what());
    ROS_ERROR_STREAM(
        "Error sub-code: " << Kinova::Api::SubErrorCodes_Name(
            Kinova::Api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))));
  } catch (std::runtime_error& ex2) {
    ROS_ERROR_STREAM("runtime error: " << ex2.what());
  } catch (...) {
    ROS_ERROR_STREAM("Unknown error.");
  }

  return success;
}

bool KinovaHardwareInterface::send_joint_velocity_command() {
  for (size_t i = 0; i < 7; ++i) {
    kortex_joint_speeds_cmd_.mutable_joint_speeds(i)->set_value(vel_cmd_copy_[i] * 180.0 / M_PI);
  }

  m_base->SendJointSpeedsCommand(kortex_joint_speeds_cmd_);
  return true;
}

double KinovaHardwareInterface::wrap_angle(const double a_prev, const double a_next) const {
  double a_wrapped;
  angles::shortest_angular_distance_with_large_limits(
      a_prev, a_next, std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
      a_wrapped);
  a_wrapped = a_wrapped + a_prev;
  return a_wrapped;
}

ros::Time KinovaHardwareInterface::get_time() { return ros::Time::now(); }

ros::Duration KinovaHardwareInterface::get_period() {
  ros::Time current_time = ros::Time::now();
  ros::Duration period = current_time - last_time_;
  last_time_ = current_time;
  return period;
}

std::ostream& operator<<(std::ostream& os, const joint_limits_interface::JointLimits& limits) {
  os << "q_min: " << limits.min_position << "|| q_max: " << limits.max_position
     << "|| v_max: " << limits.max_velocity << "|| eff_max: " << limits.max_effort;
  return os;
}