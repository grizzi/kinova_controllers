//
// Created by giuseppe on 31.12.20.
//

#include "kinova_mpc_controllers/mpc_controller.h"
#include <geometry_msgs/TransformStamped.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace kinova_controllers {

MPC_Controller::MPC_Controller(const ros::NodeHandle& nh) : nh_(nh) {}

bool MPC_Controller::init() {
  nh_.param<std::string>("/robot_description_mpc", robot_description_, "");
  nh_.param<std::string>("/task_file", taskFile_, "");

  nh_.param<std::string>("base_link", base_link_, "base_link");
  nh_.param<double>("mpc_frequency", mpcFrequency_, -1);

  std::string commandTopic;
  nh_.param<std::string>("command_topic", commandTopic, "/command");
  commandPublisher_ = nh_.advertise<std_msgs::Float64MultiArray>(commandTopic, 1);

  std::string pathTopic;
  nh_.param<std::string>("path_topic", pathTopic, "/desired_path");
  targetPathSubscriber_ = nh_.subscribe(pathTopic, 10, &MPC_Controller::pathCallback, this);

  std::string robotName;
  nh_.param<std::string>("robot_name", robotName, "/mobile_manipulator");
  observationPublisher_ =
      nh_.advertise<ocs2_msgs::mpc_observation>("/" + robotName + "_mpc_observation", 10);

  mm_interface_.reset(
      new mobile_manipulator::MobileManipulatorInterface(taskFile_, robot_description_));
  mpcPtr_ = mm_interface_->getMpc();
  mpc_mrt_interface_.reset(new ocs2::MPC_MRT_Interface(*mpcPtr_));

  command_path_publisher_.init(nh_, "/command_path", 10);

  observation_.time = 0.0;
  observation_.state.setZero(mm_interface_->stateDim_);
  observation_.input.setZero(mm_interface_->inputDim_);
  positionCommand_.setZero();
  velocityCommand_.setZero();
  stopped_ = true;
  ROS_INFO("MPC Controller successfully initialized.");
  return true;
}

void MPC_Controller::start(const joint_vector_t& initial_observation) {
  // initial observation
  if (!stopped_) return;

  // flags
  policyReady_ = false;
  referenceEverReceived_ = false;
  observationEverReceived_ = false;

  jointInitialState_ = initial_observation;
  setObservation(initial_observation);

  // mpc solution update thread
  start_time_ = ros::Time::now().toSec();
  mpc_mrt_interface_->reset();
  mpcTimer_.reset();

  stopped_ = false;
  mpcThread_ = std::thread(&MPC_Controller::advanceMpc, this);
  ROS_INFO("MPC Controller Successfully started.");
}

void MPC_Controller::advanceMpc() {
  static double elapsed;

  while (ros::ok() && !stopped_) {
    if (!referenceEverReceived_) {
      ROS_WARN_THROTTLE(3.0, "Reference never received. Skipping MPC update.");
      continue;
    }

    if (!observationEverReceived_) {
      ROS_WARN_THROTTLE(3.0, "Observation never received. Skipping MPC update.");
      continue;
    }

    nav_msgs::Path pathTarget;
    {
      std::lock_guard<std::mutex> lock(desiredPathMutex_);
      pathTarget = desiredPath_;
    }

    adjustPath(pathTarget);
    writeDesiredPath(pathTarget);

    if (command_path_publisher_.trylock()){
      command_path_publisher_.msg_ = pathTarget;
      command_path_publisher_.unlockAndPublish();
    }

    {
      std::lock_guard<std::mutex> lock(observationMutex_);
      mpc_mrt_interface_->setCurrentObservation(observation_);
    }

    mpcTimer_.startTimer();
    try {
      mpc_mrt_interface_->advanceMpc();
    } catch (const std::runtime_error& exc) {
      ROS_ERROR_STREAM(exc.what());
    }
    mpcTimer_.endTimer();
    elapsed = mpcTimer_.getLastIntervalInMilliseconds() / 1e3;
    if (1. / elapsed < mpcFrequency_) {
      ROS_WARN_STREAM_THROTTLE(
          5.0, "[MPC_Controller::advanceMpc] Mpc thread running slower than real time: "
                   << elapsed << " > " << 1. / mpcFrequency_);
    }

    if (mpcFrequency_ > 0 && 1. / elapsed < mpcFrequency_)
      std::this_thread::sleep_for(
          std::chrono::microseconds(int(1e6 * (1.0 / mpcFrequency_ - elapsed))));
    policyReady_ = true;
  }
}

void MPC_Controller::update(const ros::Time& time, const joint_vector_t& observation) {
  setObservation(observation);
  updateCommand();
  publishObservation();
}

void MPC_Controller::setObservation(const joint_vector_t& observation) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  observation_.time = ros::Time::now().toSec() - start_time_;
  observation_.state.tail(7) = observation;
  observationEverReceived_ = true;
}

void MPC_Controller::updateCommand() {
  static Eigen::VectorXd mpcState;
  static Eigen::VectorXd mpcInput;
  static size_t mode;

  if (!referenceEverReceived_ || !policyReady_) {
    positionCommand_ = jointInitialState_;
    velocityCommand_.setZero();
    return;
  }

  mpc_mrt_interface_->updatePolicy();
  mpc_mrt_interface_->evaluatePolicy(observation_.time, observation_.state, mpcState, mpcInput,
                                     mode);
  positionCommand_ = mpcState.tail(7);  // when mpc active, only velocity command
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

void MPC_Controller::adjustPathTime(nav_msgs::Path& desiredPath) const {
  if (desiredPath.poses.empty()) return;

  // take only relative timing from path
  double current_mpc_time = ros::Time::now().toSec() - start_time_;
  ROS_INFO_STREAM("[MPC_Controller::adjustPathTime] Current mpc time: " << current_mpc_time);

  double time_offset = desiredPath.poses[0].header.stamp.toSec() - current_mpc_time;
  ROS_INFO_STREAM("[MPC_Controller::adjustPathTime] Time offset is: " << time_offset);

  for (auto& pose : desiredPath.poses) {
    pose.header.stamp = pose.header.stamp - ros::Duration(time_offset);
  }
}

void MPC_Controller::pathCallback(const nav_msgs::PathConstPtr& desiredPath) {
  if (desiredPath->poses.empty()) {
    ROS_WARN("[MPC_Controller::pathCallback] Received path is empty");
    return;
  }

  bool ok = sanityCheck(*desiredPath);
  if (!ok) {
    ROS_WARN("[MPC_Controller::pathCallback] Received path is ill formed.");
    return;
  }

  ROS_INFO_STREAM("[MPC_Controller::pathCallback] Received new path with "
                      << desiredPath->poses.size() << " poses.");

  {
    std::lock_guard<std::mutex> lock(desiredPathMutex_);
    desiredPath_ = *desiredPath;
    transformPath(desiredPath_);   // transform to the correct base frame
    adjustPathTime(desiredPath_);  // adjust time stamps keeping relative time-distance
  }
  referenceEverReceived_ = true;
}

void MPC_Controller::writeDesiredPath(const nav_msgs::Path& desiredPath) {
  ocs2::CostDesiredTrajectories costDesiredTrajectories(desiredPath.poses.size());
  size_t idx = 0;
  for (const auto& waypoint : desiredPath.poses) {
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

  mpc_mrt_interface_->setTargetTrajectories(costDesiredTrajectories);
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

void MPC_Controller::transformPath(nav_msgs::Path& desiredPath) const {
  ROS_INFO_STREAM("[MPC_Controller::transformPath] Transforming path from "
                  << desiredPath.header.frame_id << " to " << base_link_);
  geometry_msgs::TransformStamped transformStamped;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  try {
    // target_frame, source_frame ...
    transformStamped = tfBuffer.lookupTransform(base_link_, desiredPath.header.frame_id,
                                                ros::Time(0), ros::Duration(3.0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }

  ros::Time stamp;
  desiredPath.header.frame_id = base_link_;
  for (auto& pose : desiredPath.poses) {
    stamp = pose.header.stamp;
    tf2::doTransform(pose, pose, transformStamped);  // doTransform overwrites the stamp;
    pose.header.stamp = stamp;
  }
}

void MPC_Controller::stop() {
  ROS_INFO("[MPC_Controller::stop] Stopping MPC update thread");
  stopped_ = true;
  mpcThread_.join();
  ROS_INFO("[MPC_Controller::stop] Stopped MPC update thread");
}

}  // namespace kinova_controllers
