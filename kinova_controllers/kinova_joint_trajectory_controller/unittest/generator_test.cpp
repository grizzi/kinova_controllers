//
// Created by giuseppe on 22.03.21.
//

#include <gtest/gtest.h>
#include "kinova_joint_trajectory_controller/trajectory_generator.h"

TEST(KinovaJointTrajectoryController, GeneratorTest)
{
  Eigen::VectorXd start(7);
  Eigen::VectorXd end(7);
  start.setZero();
  end.setConstant(1.0);
  kinova_controllers::TrajectoryGenerator generator(0.5, 1.5, 7);
  generator.compute(start, end, 0.0);

  std::cout << "Start is: " << start.transpose() << std::endl;
  std::cout << "End is: " << end.transpose() << std::endl;

  size_t steps = 10;
  Eigen::VectorXd next;
  for (size_t i=0; i<steps; i++){
    next = generator.get_next_point(i * 0.2);
    std::cout << "Next is: " << next.transpose() << std::endl;
  }
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
