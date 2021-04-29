### kinova_reflex_controller
TODO(giuseppe)

### kinova_joint_velocity_controller
TODO(giuseppe)

### kinova_joint_trajectory_controller
TODO(giuseppe)

### kinova_mpc_controller
TODO(giuseppe)

Depends on `ocs2_dev`. The code is into a local repo at the moment. TODO(giuseppe) clean up the situation with ocs2
The kinematic example uses `pinocchio` and `hpp-fcl` for automatic differentiation of the forward kinematics and to solve for self collision. Support for `hpp-fcl` is not provided by default and therefore these two packages need to be built from source. 


[Build from source](https://github.com/humanoid-path-planner/hpp-fcl/blob/devel/INSTALL) `hpp-fcl`. 


[Build from source](https://stack-of-tasks.github.io/pinocchio/download.html) `pinocchio`. In particular specify `hpp-fcl` support setting the corresponding flag when generating the make files:
```
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DBUILD_WITH_COLLISION_SUPPORT=ON
```

