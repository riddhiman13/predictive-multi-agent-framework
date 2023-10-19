This repository contains the code regarding the paper
Riddhiman Laha, Marvin Becker, Jonathan Vorndamme, Juraj Vrabel, Luis F.C Figueredo, Matthias A. Müller, and Sami Haddadin, "Predictive Multi-Agent based Planning and Landing Controller for Reactive Dual-Arm Manipulation", IEEE Transactions on Robotics 2023.
The repository contains all code necessary for running the predictive multi-agent based planning and landing controller described in the paper. The description of the Kobo system is missing due to a non-disclosure agreement. Nevertheless, all experiments with the dual arm setups can be reproduced.

# Requirements
The code is only tested on Ubuntu 18.04 and Ubuntu 20.04.
The following packages need to be installed:
- CoppeliaSim [https://www.coppeliarobotics.com/downloads](https://www.coppeliarobotics.com/downloads) 
- ROS Melodic/Noetic [http://wiki.ros.org/ROS/Installation](http://wiki.ros.org/ROS/Installation)
- DQ Robotics including the CoppeliaSim interface (formerly V-REP) [https://dqrobotics.github.io/](https://dqroboticsgithubio.readthedocs.io/en/latest/installation/cpp.html#release-ppa). Note that you need to add the stable release PPA and not the development PPA.

## Additional requirements
``` bash
sudo apt install ros-noetic-rviz-visual-tools ros-noetic-franka-* ros-noetic-moveit ros-noetic-moveit-visual-tools ros-noetic-rosparam-shortcuts
```

# Installation and Compilation
1. Create a folder for the catkin workspace and clone the repository:
``` bash
mkdir -p <CatkinWSRootFolder>/pmaf_ws/
cd <CatkinWSRootFolder>/pmaf_ws/
git clone https://github.com/riddhiman13/predictive-multi-agent-framework.git .
```

2. Initialize catkin workspace, adjust compile arguments and compile:
``` bash
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

# Startup
1. Start CoppeliaSim in headless mode with a loaded scene:
``` bash
cd <CoppeliaSimRootFolder>
./coppeliaSim.sh -h <CatkinWSRootFolder>/pmaf_ws/src/bimanual_planning_ros/vrep_scenes/dual_arms.ttt
```

2. In a new terminal: start the CoppeliaSim interface and load the desired task sequence, e.g.:
``` bash
roslaunch bimanual_planning_ros vrep_interface_dual_arms.launch task_sequence:=dual_arms_static1
```

3. In a new terminal: start the planning and control nodes:
``` bash
roslaunch bimanual_planning_ros planning_moveit_dual_arms.launch
```

4. In the same terminal (planning node): hit enter to start planning.


# Additional Information
Note that the last obstacle that is defined in the task sequence .yaml is only used for self collision avoidance with repulsive force generation.
Also note that touching spheres are not merged automatically and need to be merged manually such that the current vectors are not defined in a way such that the robot is guided in between them (cf. the trap-like obstacle in Fig.~9(c), or the sphere in Fig.~9(b) of the paper). 
This repository contains the code 

