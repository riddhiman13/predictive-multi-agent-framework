# Predictive Multi-Agent-Based Planning and Landing Controller
Created by [Riddhiman Laha](https://sites.google.com/view/riddhimanlaha), [Marvin Becker](https://www.irt.uni-hannover.de/de/mbecker), [Jonathan Vorndamme](https://www.ce.cit.tum.de/rsi/team/vorndamme-jonathan/), [Juraj Vrabel](), [Luis F.C Figueredo](https://www.luisfigueredo.com), [Matthias A. Müller](https://www.irt.uni-hannover.de/de/institut/team/mueller), and [Sami Haddadin](https://www.professoren.tum.de/en/haddadin-sami) from the Technical University of Munich and Leibniz University Hannover.
![qual_ex_2_sc2](https://github.com/riddhiman13/predictive-multi-agent-framework/assets/44759480/4dde1e2f-e560-4ef0-9a44-c0091272da5c)

This repository contains the code regarding the [paper](https://ieeexplore.ieee.org/abstract/document/10354340):  
"Predictive Multi-Agent based Planning and Landing Controller for Reactive Dual-Arm Manipulation", IEEE Transactions on Robotics, 2023.

The repository contains all the code necessary for running the predictive multi-agent-based planning and landing controller described in the paper. 

### Citation

If you find our work useful in your research, please consider citing:
``` bash
@article{lahaTRO23,
  author={Laha, Riddhiman and Becker, Marvin and Vorndamme, Jonathan and Vrabel, Juraj and Figueredo, Luis F.C. and Müller, Matthias A. and Haddadin, Sami},
  journal={IEEE Transactions on Robotics}, 
  title={Predictive Multi-Agent based Planning and Landing Controller for Reactive Dual-Arm Manipulation}, 
  year={2023},
  volume={},
  number={},
  pages={1-20},
  doi={10.1109/TRO.2023.3341689}}
```

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
git clone https://github.com/riddhiman13/predictive-multi-agent-framework.git
```

2. Initialize catkin workspace, adjust compile arguments and compile:
``` bash
catkin init
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
```

# Startup
1. Start CoppeliaSim in headless mode with a loaded scene, e.g.:
``` bash
cd <CoppeliaSimRootFolder>
./coppeliaSim.sh -h <CatkinWSRootFolder>/pmaf_ws/src/bimanual_planning_ros/vrep_scenes/dual_arms.ttt
```

2. In a new terminal: start the CoppeliaSim interface and load the desired task sequence, e.g.:
``` bash
roslaunch bimanual_planning_ros vrep_interface_dual_arms.launch task_sequence:=dual_arms_static1
```

3. In a new terminal: start the planning and control nodes, e.g.:
``` bash
roslaunch bimanual_planning_ros planning_moveit_dual_arms.launch
```

4. In the same terminal (planning node): hit enter to start planning.

For the simulations with the Kobo robot, replace "dual_arms" with "kobo" in the launch files, load the kobo.ttt in CoppeliaSim and use the corresponding task sequences.

# Additional Information
Note that the last obstacle that is defined in the task sequence .yaml is only used for self collision avoidance with repulsive force generation.
Also note that touching spheres are not merged automatically and need to be merged manually such that the current vectors are not defined in a way such that the robot is guided in between them (cf. the trap-like obstacle in Fig.~9(c), or the sphere in Fig.~9(b) of the paper).

# Disclaimer
Only the code in src/bimanual_planning_ros is written and owned by us. All other code in this repository is from third parties and belongs to the ROS planning software stack and the MoveIt motion planning framework, for more details see:
- [https://github.com/ros-planning](https://github.com/ros-planning)
- [https://moveit.ros.org/](https://moveit.ros.org/)

