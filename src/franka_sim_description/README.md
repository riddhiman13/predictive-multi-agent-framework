# franka\_sim\_description
This package contains URDF file for franka arm simulation in Gazebo (`panda_arm.urdf.xacro`). Be aware that the mass and inertia values of the links in the URDF file are from a [third-party repository](https://github.com/corlab/cogimon-gazebo-models/blob/master/franka/model.urdf) and are not official.

Also there is a parameter `dual` in the XACRO macro, which is used by the `alan_description` & `blaise_description` package. This macro removes part of the first link of the arm, so that it fits inside the upper body.

Also the `panda_gazebo.xacro` contains a the `panda_gazebo` macro which can use to setup add gazebo and transmission tags to simulate the robot (with estimated inertias) in Gazebo. This gets called when instantiating the `panda_arm.xacro` macro automatically.
