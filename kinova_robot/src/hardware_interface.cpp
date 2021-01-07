#include "kinova_robot/hardware_interface.h"

#define GRIPPER_MINIMAL_POSITION_ERROR 1.5

using namespace hardware_interface;

KinovaHardwareInterface::KinovaHardwareInterface(ros::NodeHandle& nh) : KortexArmDriver(nh) {
  initialized_ = true;

  // clear all faults (and set to highlevel servoing)
  m_base->ClearFaults();

  // init joint names
  joint_names.resize(7);
  for (int i = 0; i < 7; i++) {
    joint_names[i] = m_arm_joint_names[i];
  }

  ROS_INFO_STREAM("Starting Kinova hardware interface in namespace: " << nh.getNamespace());
  for (std::size_t i = 0; i < 7; ++i) {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle(joint_names[i], &pos_wrapped[i], &vel[i],
                                                      &eff[i]);
    jnt_state_interface.registerHandle(state_handle);

    // connect and register the joint command interface
    hardware_interface::KinovaCommandHandle cmd_handle(
        jnt_state_interface.getHandle(joint_names[i]), &pos_cmd[i], &vel_cmd[i], &eff_cmd[i],
        &mode);
    jnt_cmd_interface.registerHandle(cmd_handle);
  }
  registerInterface(&jnt_state_interface);
  registerInterface(&jnt_cmd_interface);

  if (isGripperPresent()) {
    hardware_interface::JointStateHandle state_handle("gripper", &gripper_position,
                                                      &gripper_velocity, &gripper_force);
    jnt_state_interface.registerHandle(state_handle);

    hardware_interface::JointHandle gripper_handle(jnt_state_interface.getHandle("gripper"),
                                                   &gripper_position_command);
    gripper_cmd_interface.registerHandle(gripper_handle);
    registerInterface(&gripper_cmd_interface);
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
  current_mode = KinovaControlMode::NO_MODE;
  mode = current_mode;
  mode_copy = current_mode;

  // first read and fill command
  read();
  for (size_t i = 0; i < 7; i++) {
    pos_cmd[i] = pos[i];
    pos_wrapped[i] = pos[i];
    vel_cmd[i] = vel[i];
    eff_cmd[i] = eff[i];
    pos_error[i] = 0.0;
    vel_error[i] = 0.0;
    kortex_cmd.add_actuators();
  }
  copy_commands();

  realtime_state_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(
      m_node_handle, "/kinova_ros_control/joint_state", 4));
  realtime_command_pub_.reset(new realtime_tools::RealtimePublisher<sensor_msgs::JointState>(
      m_node_handle, "/kinova_ros_control/joint_command", 4));
  // realtime_wrench_pub_.reset(new
  // realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>(m_node_handle,
  // "/kinova_ros_control/external_wrench", 4));

  for (unsigned i = 0; i < 7; i++) {
    realtime_state_pub_->msg_.name.push_back(joint_names[i]);
    realtime_state_pub_->msg_.position.push_back(pos_wrapped[i]);
    realtime_state_pub_->msg_.velocity.push_back(vel[i]);
    realtime_state_pub_->msg_.effort.push_back(eff[i]);

    realtime_command_pub_->msg_.name.push_back(joint_names[i]);
    realtime_command_pub_->msg_.position.push_back(pos_cmd[i]);
    realtime_command_pub_->msg_.velocity.push_back(vel_cmd[i]);
    realtime_command_pub_->msg_.effort.push_back(eff_cmd[i]);
  }

  if (isGripperPresent()) {
    realtime_state_pub_->msg_.name.push_back("gripper");
    realtime_state_pub_->msg_.position.push_back(gripper_position);
    realtime_state_pub_->msg_.velocity.push_back(gripper_velocity);
    realtime_state_pub_->msg_.effort.push_back(gripper_force);

    gripper_position_error = 0.0;
    gripper_position_command = gripper_position;
    gripper_velocity_command = gripper_velocity;
    gripper_force_command = 10.0;  // TODO(giuseppe) hard coded -> could be set
    kortex_gripper_cmd =
        kortex_cmd.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
    kortex_gripper_cmd->set_position(gripper_position_command);
    kortex_gripper_cmd->set_velocity(gripper_velocity_command);
    kortex_gripper_cmd->set_force(gripper_force_command);

    realtime_command_pub_->msg_.name.push_back("gripper");
    realtime_command_pub_->msg_.position.push_back(gripper_position_command);
    realtime_command_pub_->msg_.velocity.push_back(gripper_velocity_command);
    realtime_command_pub_->msg_.effort.push_back(gripper_force_command);
  }

  // set_actuators_control_mode(current_mode);

  stop_writing = false;
  std::string emergency_stop_service_name = "/my_gen3/base/apply_emergency_stop";
  estop_client_ =
      m_node_handle.serviceClient<kortex_driver::ApplyEmergencyStop>(emergency_stop_service_name);
  if (!estop_client_.waitForExistence(ros::Duration(10.0))) {
    ROS_ERROR_STREAM("Could not contact service: " << emergency_stop_service_name);
    initialized_ = false;
  }

  // KDL
  //  std::string robot_description;
  //  if (!m_node_handle.param<std::string>("robot_description", robot_description, "")){
  //    ROS_ERROR("Failed to get robot description");
  //    initialized_ = false;
  //  }

  //  if (!kdl_parser::treeFromString(robot_description, kdlTree)){
  //    ROS_ERROR_STREAM("Failed to initialized the dynamic parameter solver.");
  //    initialized_ = false;
  //  }
  //
  //  std::string root_link = m_prefix + "base_link";
  //  std::string tip_link = m_prefix + "tool_frame";
  //  if (!kdlTree.getChain(root_link, tip_link, kdlChain))
  //  {
  //    ROS_ERROR("Failed to extract chain");
  //    initialized_ = false;
  //  }
  //  KDL::Vector grav(0.0,0.0,-9.81);
  //  dynamic_parameter_solver_ = std::make_unique<KDL::ChainDynParam>(kdlChain, grav);
  //  fk_pos_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdlChain);
  //  jacobian_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(kdlChain);
  //  init_wrench_estimator();

  // FT sensor interface
  hardware_interface::ForceTorqueSensorHandle ft_handle("ft_sensor", "ee", force_, torque_);
  force_torque_interface.registerHandle(ft_handle);
  registerInterface(&force_torque_interface);

  last_time = ros::Time::now();
  cm = new controller_manager::ControllerManager(&*this);
}

KinovaHardwareInterface::~KinovaHardwareInterface() {
  if (write_thread.joinable()) {
    write_thread.join();
  }

  set_actuators_control_mode(KinovaControlMode::VELOCITY);

  if (read_update_thread.joinable()) read_update_thread.join();
}

bool KinovaHardwareInterface::init_pid() {
  pid_.resize(7);
  for (size_t i = 0; i < 7; i++) {
    const ros::NodeHandle nh(m_node_handle.getNamespace() + "/arm_pid/" + joint_names[i]);
    if (!pid_[i].init(nh, false)) {
      ROS_ERROR_STREAM("Failed to initialize PID controller for " << joint_names[i]);
      return false;
    }
  }
  return true;
}

bool KinovaHardwareInterface::set_joint_limits() {
  bool limits_ok = true;
  limits.resize(joint_names.size());
  for (size_t i = 0; i < joint_names.size(); i++) {
    limits_ok &= joint_limits_interface::getJointLimits(joint_names[i], m_node_handle, limits[i]);
  }
  if (!limits_ok) {
    return false;
  }
  std::stringstream joint_limits_string;
  for (size_t i = 0; i < joint_names.size(); i++) {
    joint_limits_string << "Limits " << joint_names[i] << ": " << limits[i] << std::endl;
  }
  ROS_INFO_STREAM(joint_limits_string.str());
  return true;
}

/// read loop functions
/// keep consistency with simulation: angle in range [-PI, PI] and unlimited continuous joints
void KinovaHardwareInterface::read() {
  // in low-level mode sending the command also returns the current state
  if (current_mode == KinovaControlMode::VELOCITY || current_mode == KinovaControlMode::NO_MODE)
    current_state = m_base_cyclic->RefreshFeedback();

  for (int i = 0; i < current_state.actuators_size(); i++) {
    std::shared_lock<std::shared_mutex> lock(m_zero_position_mutex);
    pos[i] = angles::normalize_angle(
        static_cast<double>(angles::from_degrees(current_state.actuators(i).position())));

    // wrap angle for continuous joints (the even ones) and account for bias
    pos_wrapped[i] = ((i % 2) == 0) ? wrap_angle(pos_wrapped[i], pos[i]) : pos[i];
    pos_wrapped[i] -= m_zero_position[i];

    vel[i] = static_cast<double>(angles::from_degrees(current_state.actuators(i).velocity()));
    eff[i] = static_cast<double>(-current_state.actuators(i).torque());
  }

  if (isGripperPresent()) {
    gripper_position = current_state.interconnect().gripper_feedback().motor()[0].position();
    gripper_velocity = current_state.interconnect().gripper_feedback().motor()[0].velocity();
  }
}

void KinovaHardwareInterface::update() { cm->update(this->get_time(), this->get_period()); }

void KinovaHardwareInterface::enforce_limits() {
  for (size_t i = 0; i < 7; i++) {
    pos_cmd[i] = std::max(std::min(pos_cmd[i], limits[i].max_position), limits[i].min_position);
    vel_cmd[i] = std::max(std::min(vel_cmd[i], limits[i].max_velocity), -limits[i].max_velocity);
    eff_cmd[i] = std::max(std::min(eff_cmd[i], limits[i].max_effort), -limits[i].max_effort);
  }

  gripper_position_command = std::max(std::min(gripper_position_command, 100.0), 0.0);
}

void KinovaHardwareInterface::check_state() {
  bool ok = true;
  for (size_t i = 0; i < 7; i++) {
    ok &= !limits[i].has_position_limits ||
          ((pos_wrapped[i] < limits[i].max_position) && (pos_wrapped[i] > limits[i].min_position));
    if (!ok) {
      ROS_ERROR_STREAM_THROTTLE(3.0,
                                "Joint " << i << " violated position limits: " << pos_wrapped[i]);
      break;
    }

    ok &= !limits[i].has_velocity_limits ||
          ((vel[i] < limits[i].max_velocity) && (vel[i] > -limits[i].max_velocity));
    if (!ok) {
      ROS_ERROR_STREAM_THROTTLE(3.0, "Joint " << i << " violated velocity limits: " << vel[i]);
      break;
    }

    ok &= !limits[i].has_effort_limits ||
          ((eff[i] < limits[i].max_effort) && (eff[i] > -limits[i].max_effort));
    if (!ok) {
      ROS_ERROR_STREAM_THROTTLE(3.0, "Joint " << i << " violated effort limits: " << eff[i]);
      break;
    }
  }

  if (!ok && !estopped_) {
    stop_writing = true;
    kortex_driver::ApplyEmergencyStopRequest req;
    kortex_driver::ApplyEmergencyStopResponse res;
    estop_client_.call(req, res);
    estopped_ = true;
  }
}

void KinovaHardwareInterface::switch_mode() {
  if (mode != current_mode) {
    ROS_INFO_STREAM("Switching to mode: " << mode);
    set_actuators_control_mode(mode);
    return;
  }
}

void KinovaHardwareInterface::copy_commands() {
  std::lock_guard<std::mutex> lock(cmd_mutex);
  mode_copy = current_mode;
  for (size_t i = 0; i < 7; i++) {
    pos_cmd_copy[i] = pos[i];  // avoid following error
    vel_cmd_copy[i] = vel_cmd[i];
    eff_cmd_copy[i] = eff_cmd[i];

    pos_error[i] = pos_cmd[i] - pos_wrapped[i];
    vel_error[i] = -vel[i];
  }

  if (isGripperPresent()) {
    gripper_position_error = gripper_position_command - gripper_position;
  }
}

void KinovaHardwareInterface::publish_state() {
  if (realtime_state_pub_->trylock()) {
    realtime_state_pub_->msg_.header.stamp = ros::Time::now();
    for (unsigned i = 0; i < 7; i++) {
      realtime_state_pub_->msg_.position[i] = pos_wrapped[i];
      realtime_state_pub_->msg_.velocity[i] = vel[i];
      realtime_state_pub_->msg_.effort[i] = eff[i];
    }

    if (isGripperPresent()) {
      realtime_state_pub_->msg_.position[7] = gripper_position;
      realtime_state_pub_->msg_.velocity[7] = gripper_velocity;
      realtime_state_pub_->msg_.effort[7] = gripper_force;
    }

    realtime_state_pub_->unlockAndPublish();
  }

  //  if (realtime_wrench_pub_->trylock()){
  //    realtime_wrench_pub_->msg_.header.stamp = ros::Time::now();
  //    realtime_wrench_pub_->msg_.header.frame_id = "arm_frame_tool"; // TODO(giuseppe) change to
  //    the right one programmtically realtime_wrench_pub_->msg_.wrench.force.x =
  //    external_wrench_(0); realtime_wrench_pub_->msg_.wrench.force.y = external_wrench_(1);
  //    realtime_wrench_pub_->msg_.wrench.force.z = external_wrench_(2);
  //    realtime_wrench_pub_->msg_.wrench.torque.x = external_wrench_(3);
  //    realtime_wrench_pub_->msg_.wrench.torque.y = external_wrench_(4);
  //    realtime_wrench_pub_->msg_.wrench.torque.z = external_wrench_(5);
  //    realtime_wrench_pub_->unlockAndPublish();
  //  }
}

void KinovaHardwareInterface::publish_commands() {
  if (realtime_command_pub_->trylock()) {
    realtime_command_pub_->msg_.header.stamp = ros::Time::now();
    for (unsigned i = 0; i < 7; i++) {
      realtime_command_pub_->msg_.position[i] = pos_cmd[i];
      realtime_command_pub_->msg_.velocity[i] = vel_cmd[i];
      realtime_command_pub_->msg_.effort[i] = eff_cmd[i];
    }

    if (isGripperPresent()) {
      realtime_command_pub_->msg_.position[7] = gripper_position_command;
      realtime_command_pub_->msg_.velocity[7] = gripper_velocity_command;
      realtime_command_pub_->msg_.effort[7] = gripper_force_command;
    }

    realtime_command_pub_->unlockAndPublish();
  }
}

void KinovaHardwareInterface::init_wrench_estimator() {
  //  KDL::JntArray control_torque(7), joint_position_measured(7), joint_velocity_measured(7);
  //  for(size_t i=0; i<7; i++){
  //    joint_position_measured(i) = pos_wrapped[i];
  //    joint_velocity_measured(i) = vel[i];
  //    control_torque(i) = eff[i];
  //  }
  //
  //  previous_jnt_mass_matrix_.resize(7);
  //  jnt_mass_matrix_.resize(7);
  //  jnt_mass_matrix_dot_.resize(7);
  //  coriolis_torque_.resize(7);
  //  gravity_torque_.resize(7);
  //  estimated_ext_torque_.resize(7);
  //  filtered_estimated_ext_torque_.resize(7);
  //  model_based_jnt_momentum_.resize(7);
  //  estimated_momentum_integral_.resize(7);
  //  initial_jnt_momentum_.resize(7);
  //  jacobian_end_eff_.resize(7);
  //
  //  int solver_result = this->dynamic_parameter_solver_->JntToMass(joint_position_measured,
  //  previous_jnt_mass_matrix_); initial_jnt_momentum_.data = previous_jnt_mass_matrix_.data *
  //  joint_velocity_measured.data; std::cout << "Initial joint momentum is: " <<
  //  initial_jnt_momentum_.data.transpose() << std::endl; external_wrench_.setZero(6);
}

int KinovaHardwareInterface::estimate_external_wrench(const double dt) {
  /**
   * ==========================================================================
   * Momentum observer, implementation based on:
   * S. Haddadin, A. De Luca and A. Albu-Schäffer,
   * "Robot Collisions: A Survey on Detection, Isolation, and Identification,"
   * in IEEE Transactions on Robotics, vol. 33(6), pp. 1292-1312, 2017.
   * ==========================================================================
   */

  //  KDL::JntArray control_torque(7), joint_position_measured(7), joint_velocity_measured(7);
  //  for(size_t i=0; i<7; i++){
  //    joint_position_measured(i) = pos_wrapped[i];
  //    joint_velocity_measured(i) = vel[i];
  //    control_torque(i) = eff[i];
  //  }
  //
  //  Eigen::VectorXd wrench_estimation_gain_(7);
  //  wrench_estimation_gain_.setConstant(100.0);
  //
  //
  //  int solver_result = this->dynamic_parameter_solver_->JntToMass(joint_position_measured,
  //  jnt_mass_matrix_); if (solver_result != 0){
  //    std::cout << "JntToMass failed: " << jacobian_solver_->strError(solver_result) << std::endl;
  //    return solver_result;
  //  }
  //
  //  solver_result = this->dynamic_parameter_solver_->JntToCoriolis(joint_position_measured,
  //  joint_velocity_measured, coriolis_torque_); if (solver_result != 0){
  //    std::cout << "JntToCoriolis failed: " << jacobian_solver_->strError(solver_result)  <<
  //    std::endl; return solver_result;
  //  }
  //
  //  solver_result = this->dynamic_parameter_solver_->JntToGravity(joint_position_measured,
  //  gravity_torque_); if (solver_result != 0){
  //    std::cout << "JntToGravity failed: " << jacobian_solver_->strError(solver_result)  <<
  //    std::endl; return solver_result;
  //  }
  //
  //  jnt_mass_matrix_dot_.data = (jnt_mass_matrix_.data - previous_jnt_mass_matrix_.data) / dt;
  //  previous_jnt_mass_matrix_.data = jnt_mass_matrix_.data;
  //
  //  std::cout << "jnt mass matrix dot is: " << jnt_mass_matrix_dot_.data.transpose() << std::endl;
  //  std::cout << "jnt position: " << joint_position_measured.data.transpose() << std::endl;
  //  std::cout << "jnt velocity: " << joint_velocity_measured.data.transpose() << std::endl;
  //  std::cout << "jnt torques: " << control_torque.data.transpose() << std::endl;
  //  total_torque_estimation_.data = control_torque.data - gravity_torque_.data -
  //  coriolis_torque_.data + jnt_mass_matrix_dot_.data * joint_velocity_measured.data; std::cout <<
  //  "total torque estimation is: " << total_torque_estimation_.data.transpose() << std::endl;
  //  // total_torque_estimation_.data = -joint_torque_measured.data - gravity_torque_.data -
  //  coriolis_torque_.data + jnt_mass_matrix_dot_.data * joint_velocity_measured.data;
  //  estimated_momentum_integral_.data += (total_torque_estimation_.data +
  //  estimated_ext_torque_.data) * dt; std::cout << "Estimated momentum integral is :" <<
  //  estimated_momentum_integral_.data.transpose() << std::endl;
  //
  //  model_based_jnt_momentum_.data = jnt_mass_matrix_.data * joint_velocity_measured.data;
  //  std::cout << "Model based momentum is :" << model_based_jnt_momentum_.data.transpose() <<
  //  std::endl;
  //
  //  estimated_ext_torque_.data = wrench_estimation_gain_.asDiagonal() *
  //  (model_based_jnt_momentum_.data -
  //                                                                       estimated_momentum_integral_.data
  //                                                                       -
  //                                                                       initial_jnt_momentum_.data);
  //  std::cout << "estimated ext torque: " << estimated_ext_torque_.data.transpose() << std::endl;
  //
  //  // First order low-pass filter
  //  double alpha = 0.5;
  //  for (int i = 0; i < 7; i++)
  //    filtered_estimated_ext_torque_(i) = alpha * filtered_estimated_ext_torque_(i) + (1.0 -
  //    alpha) * estimated_ext_torque_(i);
  //  std::cout << "estimated ext torque filtered: " <<
  //  filtered_estimated_ext_torque_.data.transpose() << std::endl;
  //
  //  /**
  //  * ========================================================================================
  //  * Propagate joint torques to Cartesian wrench using a pseudo inverse of Jacobian-Transpose
  //  * ========================================================================================
  //  */
  //  solver_result = jacobian_solver_->JntToJac(joint_position_measured, jacobian_end_eff_);
  //  if (solver_result != 0){
  //    std::cout << "JntToJac failed: " << jacobian_solver_->strError(solver_result) << std::endl;
  //    return solver_result;
  //  }
  //
  //  solver_result = fk_pos_solver_->JntToCart(joint_position_measured,
  //  tool_tip_frame_full_model_); if (solver_result != 0){
  //    std::cout << "JntToCart failed: " << fk_pos_solver_->strError(solver_result) << std::endl;
  //    return solver_result;
  //  }
  //
  //  // Transform the jacobian from the base to the tool-tip frame
  //  jacobian_end_eff_.changeBase(tool_tip_frame_full_model_.M.Inverse());
  //
  //  // Compute SVD of the jacobian using Eigen functions
  //  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian_end_eff_.data.transpose(), Eigen::ComputeThinU
  //  | Eigen::ComputeThinV); Eigen::VectorXd singular_inv(svd.singularValues()); for (int j = 0; j
  //  < singular_inv.size(); ++j) singular_inv(j) = (singular_inv(j) < 1e-6) ? 0.0 : 1.0 /
  //  singular_inv(j); jacobian_end_eff_inv_.noalias() = svd.matrixV() *
  //  singular_inv.matrix().asDiagonal() * svd.matrixU().adjoint();
  //
  //  // Compute End-Effector Cartesian forces from joint external torques
  //  external_wrench_ = jacobian_end_eff_inv_ * filtered_estimated_ext_torque_.data;
  //  force_[0] = external_wrench_(0);
  //  force_[1] = external_wrench_(1);
  //  force_[2] = external_wrench_(2);
  //  torque_[0] = external_wrench_(3);
  //  torque_[1] = external_wrench_(4);
  //  torque_[2] = external_wrench_(5);
  //  return 0;
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

    // estimate_external_wrench(elapsed_ms/1000.0); // TODO(giuseppe) this is bad timing at the
    // first tick

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
    std:
      this_thread::sleep_for(std::chrono::nanoseconds((int)((dt_ms - elapsed_ms) * 1e6)));
    }
  }
}

void KinovaHardwareInterface::write_loop() {
  std::chrono::time_point<std::chrono::steady_clock> prev_start, start, end;
  double elapsed_ms;
  double dt_ms = 1.0;
  double dt = 0.0;
  prev_start = std::chrono::steady_clock::now();
  while (ros::ok()) {
    start = std::chrono::steady_clock::now();
    dt = std::chrono::duration_cast<std::chrono::nanoseconds>(start - prev_start).count() / 1e9;
    write(dt);

    end = std::chrono::steady_clock::now();
    elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1e6;
    if (elapsed_ms > dt_ms)
      ROS_WARN_STREAM_THROTTLE(1.0, "Write took too long: " << elapsed_ms << " > 1.0 ms.");
    else {
    std:
      this_thread::sleep_for(std::chrono::nanoseconds((int)((dt_ms - elapsed_ms) * 1e6)));
    }
    prev_start = start;
  }
}

void KinovaHardwareInterface::write(const double dt) {
  if (stop_writing) return;

  std::lock_guard<std::mutex> lock(cmd_mutex);
  if (mode_copy == KinovaControlMode::POSITION || mode_copy == KinovaControlMode::EFFORT) {
    set_hardware_command(dt);
    send_lowlevel_command();
  } else if (mode_copy == KinovaControlMode::VELOCITY) {
    send_joint_velocity_command();
  } else if (mode_copy == KinovaControlMode::NO_MODE) {
  } else {
    ROS_WARN_STREAM("Unknown mode: " << mode_copy);
  }
}

/// Start main threads
void KinovaHardwareInterface::run() {
  if (!initialized_) {
    ROS_ERROR_STREAM("Kinova hardware interface failed to initialize. Not running.");
    return;
  }

  write_thread = std::thread(&KinovaHardwareInterface::write_loop, this);
  read_update_thread = std::thread(&KinovaHardwareInterface::read_loop, this, 100);
  ROS_INFO_STREAM("Kinova ros controller interface is running.");
}

/// Additional methods

bool KinovaHardwareInterface::set_servoing_mode(const Kinova::Api::Base::ServoingMode& mode) {
  bool success = false;
  Kinova::Api::Base::ServoingModeInformation servoing_mode;
  try {
    servoing_mode.set_servoing_mode(mode);
    m_base->SetServoingMode(servoing_mode);
    ROS_INFO("New servoing mode set.");
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

bool KinovaHardwareInterface::set_actuators_control_mode(const KinovaControlMode& mode) {
  Kinova::Api::ActuatorConfig::ControlModeInformation control_mode_info;
  try {
    // SINGLE LEVEL SERVOING (aka high level position/velocity control)
    // switch happens in reverse mode: first actuator, then servoing mode
    if (mode == KinovaControlMode::NO_MODE || mode == KinovaControlMode::VELOCITY) {
      stop_writing = true;

      // set command same as the current measurement
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
        ROS_INFO_STREAM("Mode switched to " << mode);
        current_mode = mode;
        return true;
      }
    }

    // LOW LEVEL SERVOING (position or torque)
    if (mode == KinovaControlMode::POSITION || mode == KinovaControlMode::EFFORT) {
      stop_writing = false;
      if (!set_servoing_mode(Kinova::Api::Base::ServoingMode::LOW_LEVEL_SERVOING)) {
        return false;
      }
      bool same_as_readings = true;
      set_hardware_command(0.0, same_as_readings);
      send_lowlevel_command();
    }

    if (mode == KinovaControlMode::EFFORT) {
      control_mode_info.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::TORQUE);
    } else {
      control_mode_info.set_control_mode(Kinova::Api::ActuatorConfig::ControlMode::POSITION);
    }

    for (size_t i = 1; i < 8; i++) {
      m_actuator_config->SetControlMode(control_mode_info, i);
    }
    ROS_INFO_STREAM("Mode switched to " << mode);
    current_mode = mode;
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
  kortex_cmd.set_frame_id(kortex_cmd.frame_id() + 1);  // unique-id to reject out-of-time frames
  if (kortex_cmd.frame_id() > 65535) {
    kortex_cmd.set_frame_id(0);
  }

  if (same_as_readings) {
    for (int idx = 0; idx < 7; idx++) {
      kortex_cmd.mutable_actuators(idx)->set_command_id(kortex_cmd.frame_id());
      kortex_cmd.mutable_actuators(idx)->set_position(
          angles::to_degrees(angles::normalize_angle_positive(pos[idx])));
      kortex_cmd.mutable_actuators(idx)->set_torque_joint(eff[idx]);
    }
  } else {
    for (int idx = 0; idx < 7; idx++) {
      kortex_cmd.mutable_actuators(idx)->set_command_id(kortex_cmd.frame_id());
      kortex_cmd.mutable_actuators(idx)->set_position(
          angles::to_degrees(angles::normalize_angle_positive(pos_cmd_copy[idx])));

      eff_cmd_copy[idx] +=
          pid_[idx].computeCommand(pos_error[idx], vel_error[idx], ros::Duration(dt));
      kortex_cmd.mutable_actuators(idx)->set_torque_joint(eff_cmd_copy[idx]);
    }
  }

  if (isGripperPresent()) {
    if (fabs(gripper_position_error) < GRIPPER_MINIMAL_POSITION_ERROR) {
      gripper_velocity_command = 0.0;
    } else {
      gripper_velocity_command = std::min(std::max(2.0 * fabs(gripper_position_error), 0.0), 100.0);
    }
    kortex_gripper_cmd->set_position(gripper_position_command);
    kortex_gripper_cmd->set_velocity(gripper_velocity_command);
    kortex_gripper_cmd->set_force(gripper_force_command);
  }
}

bool KinovaHardwareInterface::send_lowlevel_command() {
  bool success = false;
  try {
    ROS_INFO_ONCE("Sending first low level command");
    auto start = std::chrono::steady_clock::now();
    current_state = m_base_cyclic->Refresh(kortex_cmd, 0);
    auto end = std::chrono::steady_clock::now();
    double dt_refresh =
        std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() / 1e6;
    if (dt_refresh > 1.0)
      ROS_WARN_STREAM_THROTTLE(1.0, "Refresh command took :" << dt_refresh << " ms.");
    success = true;
  } catch (Kinova::Api::KDetailedException& ex) {
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
  bool success = true;
  auto action = Action();
  action.set_name("regular velocity write");
  action.set_application_data("");

  auto jointSpeeds = action.mutable_send_joint_speeds();

  for (size_t i = 0; i < 7; ++i) {
    auto jointSpeed = jointSpeeds->add_joint_speeds();
    jointSpeed->set_joint_identifier(i);
    jointSpeed->set_value(vel_cmd_copy[i] * 180.0 / M_PI);
    jointSpeed->set_duration(0.0);
  }

  try {
    m_base->SendJointSpeedsCommand(*jointSpeeds);
  } catch (KDetailedException& ex) {
    ROS_WARN_THROTTLE(1, "Kortex exception");
    ROS_WARN_THROTTLE(1, "KINOVA exception error code: %d\n",
                      ex.getErrorInfo().getError().error_code());
    ROS_WARN_THROTTLE(1, "KINOVA exception error sub code: %d\n",
                      ex.getErrorInfo().getError().error_sub_code());
    ROS_WARN_THROTTLE(1, "KINOVA exception description: %s\n", ex.what());
    success = false;
  } catch (std::runtime_error& ex2) {
    ROS_INFO("Other Kortex exception");
    success = false;
  }
  return success;
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
  ros::Duration period = current_time - last_time;
  last_time = current_time;
  return period;
}

std::ostream& operator<<(std::ostream& os, const joint_limits_interface::JointLimits& limits) {
  os << "q_min: " << limits.min_position << "|| q_max: " << limits.max_position
     << "|| v_max: " << limits.max_velocity << "|| eff_max: " << limits.max_effort;
  return os;
}