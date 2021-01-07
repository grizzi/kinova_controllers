//
// Created by giuseppe on 07.01.21.
//

#include "kinova_robot/hardware_interface.h"

using namespace std::chrono;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinova_robot");

  ros::NodeHandle node;
  hardware_interface::KinovaHardwareInterface hw(node);

  hw.run();
  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin();                       // spin() will not return until the node has been shutdown
  //ros::spin();
  return 0;
}