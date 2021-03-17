//
// Created by giuseppe on 06.01.21.
//

#include <angles/angles.h>

using namespace hardware_interface;

namespace kinova_controllers { 

template <typename Controller>
bool KinovaMpcControllerRos<Controller>::init(hardware_interface::RobotHW* hw,
                                              ros::NodeHandle& root_nh,
                                              ros::NodeHandle& controller_nh) {

  if (!initRos(controller_nh)){
    ROS_ERROR("Failed to initialize parameters!");
  }

  // init model
  model_ = std::unique_ptr<rc::RobotWrapper>(new rc::RobotWrapper());
  model_->initFromXml(robot_description_);
  position_current_ = Eigen::VectorXd::Zero(model_->getDof());
  velocity_current_ = Eigen::VectorXd::Zero(model_->getDof());
  torque_current_ = Eigen::VectorXd::Zero(model_->getDof());

  for (size_t i = 0; i < 7; i++) {
    if (!pid_controllers_[i].init(ros::NodeHandle("/mpc_controller/gains/" + joint_names_[i]),
                                  false)) {
      ROS_ERROR_STREAM("Failed to load PID parameters from " << joint_names_[i] + "/pid");
      return false;
    }
    pid_controllers_[i].printValues();
  }

  mpc_controller_ = std::unique_ptr<Controller>(new Controller(controller_nh));
  if (!mpc_controller_->init()) {
    ROS_ERROR("Failed to initialize the MPC controller");
    return false;
  }

  if (!addStateHandles(hw)){
    ROS_ERROR("Failed to add state handles");
    return false;
  }

  if (!addCommandHandles(hw)){
    ROS_ERROR("Failed to add command handles!");
    return false;
  }
  return true;
}

template <typename Controller>
bool KinovaMpcControllerRos<Controller>::initRos(ros::NodeHandle& nh) {

  nh.param<std::string>("/mpc_controller/tool_link", tool_frame_, "tool_frame");

  nh.param<std::vector<std::string>>("/mpc_controller/joint_names", joint_names_, {});
  if (joint_names_.size() != 7) {
    ROS_ERROR("Joint names must be 7.");
    return false;
  }

  if (!nh.param<std::string>("/my_gen3/robot_description", robot_description_, "")) {
    ROS_ERROR_STREAM("Could not find param /my_gen3/robot_description");
    return false;
  }

  reset_imarker_pose_pub_ = nh.advertise<geometry_msgs::Pose>("/reset_marker_pose", 1);
  joint_state_des_pub_ = nh.advertise<sensor_msgs::JointState>("/mpc_joint_state_desired", 1);
  joint_state_cur_pub_ = nh.advertise<sensor_msgs::JointState>("/mpc_joint_state_current", 1);
  return true;
}

template <typename Controller>
void KinovaMpcControllerRos<Controller>::starting(const ros::Time& time) {
  ROS_INFO("Starting KinovaMpcVelocityController!");
  readState();

  ROS_INFO_STREAM("Starting with current joint position: " << position_current_.transpose());
  mpc_controller_->start(position_current_.head<7>());
  position_integral_ = position_current_;

  // Compute kinova torque offset
  model_->updateState(position_current_, Eigen::VectorXd::Zero(model_->getDof()));
  model_->computeAllTerms();
  torque_offset_ = torque_current_ - model_->getNonLinearTerms().head<7>();
  ROS_INFO_STREAM("Torque offset at startup: " << torque_offset_.transpose());

  // Reset interactive marker pose (if running)
  ROS_INFO_STREAM("Resetting marker pose to current tool pose. Tool frame id: " << tool_frame_);
  model_->updateState(position_current_, velocity_current_);
  pinocchio::SE3 tool_pose = model_->getFramePlacement(tool_frame_);
  geometry_msgs::Pose marker_pose;
  marker_pose.position.x = tool_pose.translation()(0);
  marker_pose.position.y = tool_pose.translation()(1);
  marker_pose.position.z = tool_pose.translation()(2);
  Eigen::Quaterniond orientation(tool_pose.rotation());
  marker_pose.orientation.x = orientation.x();
  marker_pose.orientation.y = orientation.y();
  marker_pose.orientation.z = orientation.z();
  marker_pose.orientation.w = orientation.w();
  std::cout << "Marker pose.position = " << tool_pose.translation().transpose() << std::endl;
  std::cout << "Marker pose.rotation = " << orientation.coeffs().transpose() << std::endl;
  reset_imarker_pose_pub_.publish(marker_pose);
}

template <typename Controller>
void KinovaMpcControllerRos<Controller>::update(const ros::Time& time,
                                                const ros::Duration& period) {
  readState();
  mpc_controller_->update(time, position_current_.head<7>());
  writeCommand(period);
}

template <typename Controller>
bool KinovaMpcControllerRos<Controller>::addStateHandles(hardware_interface::RobotHW* hw) {
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

template <typename Controller>
void KinovaMpcControllerRos<Controller>::readState() {
  for (size_t i = 0; i < 7; i++) {
    position_current_(i) = state_handles_[i].getPosition();
    velocity_current_(i) = state_handles_[i].getVelocity();
    torque_current_(i) = state_handles_[i].getEffort();
    joint_state_cur_.position[i] = position_current_(i);
    joint_state_cur_.velocity[i] = velocity_current_(i);
  }
}

template <typename Controller>
void KinovaMpcControllerRos<Controller>::computeTorqueCommands(joint_vector_t& tau,
                                                               const ros::Duration& period) {
  position_command_ = mpc_controller_->get_position_command();
  velocity_command_ = mpc_controller_->get_velocity_command();
  position_integral_ += velocity_command_ * period.toSec();

  model_->updateState(position_current_, Eigen::VectorXd::Zero(model_->getDof()));
  model_->computeAllTerms();
  gravity_and_coriolis_ = model_->getNonLinearTerms().head<7>();


  ROS_DEBUG_STREAM_THROTTLE(1.0, std::endl
      << "Pos cmd: " << position_command_.transpose() << std::endl
      << "Pos mes: " << position_current_.transpose() << std::endl
      << "Vel cmd: " << velocity_command_.transpose() << std::endl
      << "Vel mes: " << velocity_current_.transpose());

  for (size_t i = 0; i < 7; i++){
    position_error_(i) = angles::shortest_angular_distance(position_current_(i), position_integral_(i));
    velocity_error_(i) = velocity_command_(i) - velocity_current_(i);
    tau(i) = pid_controllers_[i].computeCommand(position_error_(i), velocity_error_(i), period) + 
             gravity_and_coriolis_(i) + torque_offset_(i);
  }
}   

}  // namespace kinova_controllers