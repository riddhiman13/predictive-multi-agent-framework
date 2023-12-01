blaise_description Package
===================

Blaise is the humanoid mobile robot with two actuated wheels, four caster wheels and two Franka arms. The current hardware consists of a _static_ dual arm setup only with no mobile base. You can, however, simulate the mobile base e.g. in Gazebo.

## Blaise Robot Visualization

This package can be used to visualize the static systems with the arms only as well as the simulated mobile system in RViz:

```
roslaunch blaise_description visualize.launch [simulation:={false|true}]
                                              [gui:={true|false}]
                                              [camera:={astra|realsense}]
                                              [with_hands:={true|false}]
```
