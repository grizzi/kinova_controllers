//
// Created by giuseppe on 25.01.21.
//

#include "sensor_tools/ft_sensor.h"

namespace sensor_tools::ft{

bool ForceTorqueSensor::reset() {
  if (!get_ft_calibration_from_file(calibration_file_, calibration_data_)) {
    ROS_ERROR_STREAM("Failed to reset FT Sensor calibration.");
    return false;
  }
  ROS_INFO_STREAM(calibration_data_);
  wrench_received_ = false;
  return true;
}

bool ForceTorqueSensor::init() {
  if (!nh_.param<std::string>("sensor_frame", sensor_frame_, "")) {
    ROS_ERROR_STREAM("Failed to parse sensor_frame");
    return false;
  }

  if (!nh_.param<std::string>("gravity_aligned_frame", gravity_aligned_frame_, "")) {
    ROS_ERROR_STREAM("Failed to parse gravity_aligned_frame");
    return false;
  }

  if (!nh_.param<std::string>("calibration_file", calibration_file_, "")) {
    ROS_ERROR_STREAM("Could not find calibration_file on param server.");
    return false;
  }
  if (!get_ft_calibration_from_file(calibration_file_, calibration_data_)) {
    ROS_ERROR_STREAM("Failed to parse FT Sensor calibration.");
    return false;
  }
  ROS_INFO_STREAM(calibration_data_);

  std::string raw_wrench_topic;
  if (!nh_.param<std::string>("raw_wrench_topic", raw_wrench_topic, "")) {
    ROS_ERROR_STREAM("Failed to parse raw_wrench_topic");
    return false;
  }

  std::string out_wrench_topic;
  if (!nh_.param<std::string>("out_wrench_topic", out_wrench_topic, "")) {
    ROS_ERROR_STREAM("Failed to parse out_wrench_topic");
    return false;
  }

  wrench_publisher_.init(nh_, out_wrench_topic, 1);

  ros::SubscribeOptions so;
  wrench_callback_queue_ = std::make_unique<ros::CallbackQueue>();
  so.init<geometry_msgs::WrenchStamped>(
      raw_wrench_topic, 1, boost::bind(&ForceTorqueSensor::raw_wrench_callback, this, _1));
  so.callback_queue = wrench_callback_queue_.get();
  raw_wrench_subscriber_ = nh_.subscribe(so);
  wrench_received_ = false;

  estimate_bias_measurements_ = 0;
  estimate_bias_ = false;
  estimate_bias_service_ = nh_.advertiseService("/estimate_bias", &ForceTorqueSensor::estimate_bias_callback, this);
  return true;
}

void ForceTorqueSensor::update() {
  wrench_callback_queue_->callAvailable();
  if (!wrench_received_){
    ROS_WARN_STREAM_THROTTLE(1.0, "No wrench message received yet.");
    return ;
  }

  if (estimate_bias_){
    Eigen::Matrix<double, 6, 1> raw;
    tf::wrenchMsgToEigen(wrench_raw_.wrench, raw);
    bias_ += raw;;
    estimate_bias_measurements_++;
    if (estimate_bias_measurements_ > 100){
      bias_ = 0.01 * bias_;
      calibration_data_.bias = bias_;
      estimate_bias_ = false;
      ROS_INFO_STREAM("Estimated bias is: " << calibration_data_.bias.transpose());
    }
  }

  // In fixed frame gravity is aligned with -z axis
  geometry_msgs::TransformStamped transform;
  try {
    // target_frame, source_frame ...
    transform = tf2_buffer_.lookupTransform(sensor_frame_, gravity_aligned_frame_, ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN_STREAM_THROTTLE(2.0, ex.what());
    return;
  }
  Eigen::Quaterniond q(transform.transform.rotation.w, transform.transform.rotation.x,
                       transform.transform.rotation.y, transform.transform.rotation.z);
  Eigen::Matrix3d R(q);
  tool_wrench_.get_force() = R.transpose() * Eigen::Vector3d::UnitZ() * -9.81 * calibration_data_.mass;
  tool_wrench_.get_torque() = calibration_data_.com.cross(tool_wrench_.get_force());


  /* alternative using IMU
  geometry_msgs::Vector3Stamped g;
  g.vector = gravity.linear_acceleration;
  g.header = gravity.header;
  g.header.stamp = ros::Time();

  // Convert gravity to the FT sensor frame
  geometry_msgs::Vector3Stamped g_ft_frame;
  try{
    tf2_listener.transformVector(sensor_frame_, g, g_ft_frame);
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("Error transforming gravity vector to ft sensor frame...");
    ROS_ERROR("%s.", ex.what());
    return false;
  }
  tf::vectorMsgToEigen(g_ft_frame.vector, gravity);

*/
  Eigen::Matrix<double, 6, 1> raw_wrench;
  tf::wrenchMsgToEigen(wrench_raw_.wrench, raw_wrench);

  Eigen::Matrix<double, 6, 1> compensated_wrench(raw_wrench - calibration_data_.get_bias());
  tf::wrenchEigenToMsg(compensated_wrench, wrench_compensated_.wrench);
  wrench_compensated_.header = wrench_raw_.header;

  if (wrench_publisher_.trylock()){
    wrench_publisher_.msg_ = wrench_compensated_;
    wrench_publisher_.unlockAndPublish();
  }
}

void ForceTorqueSensor::raw_wrench_callback(const geometry_msgs::WrenchStampedConstPtr& msg) {
  wrench_raw_ = *msg;
  wrench_received_ = true;
}

bool ForceTorqueSensor::estimate_bias_callback(std_srvs::EmptyRequest&, std_srvs::EmptyResponse& ){
  if (estimate_bias_){
    ROS_WARN("Already estimating the bias");
    return true;
  }
  bias_.setZero();
  estimate_bias_measurements_ = 0;
  estimate_bias_ = true;
  return true;
}
}