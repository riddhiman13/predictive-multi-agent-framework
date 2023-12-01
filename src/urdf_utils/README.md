# URDF-Utils
This repo contains utility xacro scripts which can be used for URDF generation. It contains the following xacro files:

## `inertia.xacro`
Contains an `inertia-cylinder` macro, which puts a `<inertial ... />` tag for a clinder with a `mass`, `radius` and `h`.

Contains an `inertia-sphere` macro, which puts an `<inertial ... />` tag for a sphere with a `mass` and `radius`.

Contains an `inertia-box` macro, which puts an `<inertial ... />` tag for a box with a `mass`, `width`, `height` and `depth`.

Contains an `inertia-default` macro, which puts an `<inertial ... />` tag with a diagonal inertia tensor.

## `transmissions.xacro`
Contains the `trans` macro used to declare a effort/velocity or position transmission interface for a joint.

## `gazebo.xacro`
Contains the `gazebo-friction` macro, which lets you specify friction parameters `mu1`, `mu2`, `kp` & `kd` for a `link`.

Contains the `gazebo-zero-g` macro, which lets you simulate gravity compensation for a `link`.

Contains the `gazebo_ros_control` macro, which should be included in your top level XACRO file, to enable Gazebo support.
