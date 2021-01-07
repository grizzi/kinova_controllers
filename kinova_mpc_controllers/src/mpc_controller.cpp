//
// Created by giuseppe on 31.12.20.
//

#include "kinova_mpc_controllers/mpc_controller.h"
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace kinova_controllers {

MPC_Controller::MPC_Controller(const ros::NodeHandle& nh) : nh_(nh) {}

bool MPC_Controller::init() {
  nh_.param<std::string>("/robot_description_mpc", robot_description_, "");
  nh_.param<std::string>("/task_file", taskFile_, "");

  nh_.param<double>("mpc_frequency", mpcFrequency_, -1);

  std::string commandTopic;
  nh_.param<std::string>("command_topic", commandTopic, "/command");
  commandPublisher_ = nh_.advertise<std_msgs::Float64MultiArray>(commandTopic, 1);

  std::string pathTopic;
  nh_.param<std::string>("path_topic", pathTopic, "/desired_path");
  targetPathSubscriber_ =
      nh_.subscribe(pathTopic, 10, &MPC_Controller::pathCallback, this);

  std::string robotName;
  nh_.param<std::string>("robot_name", robotName, "/mobile_manipulator");
  observationPublisher_ =
      nh_.advertise<ocs2_msgs::mpc_observation>("/" + robotName + "_mpc_observation", 10);

  mm_interface_.reset(
      new mobile_manipulator::MobileManipulatorInterface(taskFile_, robot_description_));
  mpcPtr_ = mm_interface_->getMpc();
  mpc_mrt_interface_.reset(new ocs2::MPC_MRT_Interface(*mpcPtr_));

  observation_.state.setZero(mm_interface_->stateDim_);
  observation_.input.setZero(mm_interface_->inputDim_);
  positionCommand_.setZero();
  velocityCommand_.setZero();
  ROS_INFO("MPC Controller successfully initialized.");
  return true;
}

void MPC_Controller::start(const joint_vector_t& initial_observation) {
  // initial observation
  setObservation(initial_observation);

  // reference
  referenceEverReceived_ = false;

  // mpc solution update thread
  mpc_mrt_interface_->reset();
  mpcTimer_.reset();
  mpcThread_ = std::thread(&MPC_Controller::advanceMpc, this);
  ROS_INFO("MPC Controller Successfully started.");
}

void MPC_Controller::advanceMpc() {
  static double elapsed;

  while (ros::ok()) {
    if (!referenceEverReceived_) {
      ROS_WARN_THROTTLE(3.0, "Reference never received. Skipping MPC update.");
      continue;
    }

    mpcTimer_.startTimer();
    {
      std::lock_guard<std::mutex> lock(observationMutex_);
      mpc_mrt_interface_->setCurrentObservation(observation_);
    }

    {
      std::lock_guard<std::mutex> lock(trajectoryMutex_);
      mpc_mrt_interface_->setTargetTrajectories(desiredTrajectory_);
    }

    try {
      mpc_mrt_interface_->advanceMpc();
    } catch (const std::runtime_error& exc) {
      ROS_ERROR_STREAM(exc.what());
    }
    mpcTimer_.endTimer();
    elapsed = mpcTimer_.getLastIntervalInMilliseconds() / 1e3;
    if (1. / elapsed < mpcFrequency_) {
      ROS_WARN_STREAM("Mpc thread running slower than real time: " << elapsed << " > "
                                                                   << 1. / mpcFrequency_);
    }

    if (mpcFrequency_ > 0 && 1. / elapsed < mpcFrequency_)
      std::this_thread::sleep_for(
          std::chrono::microseconds(int(1e6 * (1.0 / mpcFrequency_ - elapsed))));
  }
}

void MPC_Controller::update(const ros::Time& time, const joint_vector_t& observation) {
  setObservation(observation);
  updateCommand();
  publishObservation();
}

void MPC_Controller::setObservation(const joint_vector_t& observation) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  observation_.time = ros::Time::now().toSec();
  observation_.state.tail(7) = observation;
}

void MPC_Controller::updateCommand() {
  static Eigen::VectorXd mpcState;
  static Eigen::VectorXd mpcInput;
  static size_t mode;

  if (!referenceEverReceived_ || !mpc_mrt_interface_->initialPolicyReceived()) {
    std::lock_guard<std::mutex> lock(observationMutex_);
    static auto command = observation_.state;
    positionCommand_ = command.tail(7);
    velocityCommand_.setZero();
    return;
  }

  mpc_mrt_interface_->updatePolicy();
  mpc_mrt_interface_->evaluatePolicy(observation_.time, observation_.state, mpcState, mpcInput,
                                     mode);
  positionCommand_ = mpcState.tail(7);
  velocityCommand_ = mpcInput.tail(7);
}

void MPC_Controller::publishObservation() {
  ocs2_msgs::mpc_observation observationMsg;
  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    ocs2::ros_msg_conversions::createObservationMsg(observation_, observationMsg);
  }
  observationPublisher_.publish(observationMsg);
}

void MPC_Controller::pathCallback(const nav_msgs::PathConstPtr& desiredPath) {
  if (desiredPath->poses.empty()) {
    ROS_WARN("Received path is empty");
    return;
  }

  bool ok = sanityCheck(*desiredPath);
  if (!ok) {
    ROS_WARN("Received path is ill formed.");
    return;
  }

  adaptPath(desiredPath);

  ocs2::CostDesiredTrajectories costDesiredTrajectories(desiredPath->poses.size());
  size_t idx = 0;
  for (const auto& waypoint : desiredPath->poses) {
    // Desired state trajectory
    ocs2::scalar_array_t& tDesiredTrajectory = costDesiredTrajectories.desiredTimeTrajectory();
    tDesiredTrajectory[idx] = waypoint.header.stamp.toSec();

    // Desired state trajectory
    ocs2::vector_array_t& xDesiredTrajectory = costDesiredTrajectories.desiredStateTrajectory();
    xDesiredTrajectory[idx].resize(7);
    xDesiredTrajectory[idx].template tail<4>() =
        Eigen::Quaterniond(waypoint.pose.orientation.w, waypoint.pose.orientation.x,
                           waypoint.pose.orientation.y, waypoint.pose.orientation.z)
            .coeffs();
    xDesiredTrajectory[idx].template head<3>() << waypoint.pose.position.x,
        waypoint.pose.position.y, waypoint.pose.position.z;

    // Desired input trajectory
    ocs2::vector_array_t& uDesiredTrajectory = costDesiredTrajectories.desiredInputTrajectory();
    uDesiredTrajectory[idx].setZero(mm_interface_->inputDim_);
    idx++;
  }

  {
    std::lock_guard<std::mutex> lock(trajectoryMutex_);
    desiredTrajectory_ = costDesiredTrajectories;
    ROS_INFO_STREAM("New target trajectory: " << desiredTrajectory_);
  }
  referenceEverReceived_ = true;
}

bool MPC_Controller::sanityCheck(const nav_msgs::Path& path) {
  // check time monotonicity
  for (size_t idx = 1; idx < path.poses.size(); idx++) {
    if ((path.poses[idx].header.stamp.toSec() - path.poses[idx - 1].header.stamp.toSec()) < 0) {
      return false;
    }
  }
  return true;
}

void MPC_Controller::stop() {
  if (mpcThread_.joinable()) mpcThread_.join();
}

}  // namespace kinova_controllers
