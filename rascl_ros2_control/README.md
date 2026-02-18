# pacr_docker

**Find the readme for automation task 1+2 [here](/src/rascl_automation_ws2526_group1/readme_task1_and_2.md)**

This is the default docker development container for ros2 pkg for rascl.

## Usage
To start the container use the bash script ```rosws.sh```. This will automatically configure a container with a default environment.
```
.\rosws.sh
```
### Alias inside container
To source all ROS2 packages included in the ```src``` folder use the following command:
```
rossetup
```
To build all ROS2 packages included in the ```src``` folder use the following command in the root of the workspace ```/root/ws```:
```
rosbuild
```
To clean build all ROS2 packages included in the ```src``` folder use the following command i the root of the workspace ```/root/ws```. 
> [!warning]
> All files of ```/root/build```, ```/root/install``` and ```/root/log``` will be lost! 

```
rosclean
```
### Configuration
In order to install additional packages for the container, use the external definition files
- apt_requirements
- python_requirements

These can contain a list of apt/pip packages for installation inside the container.
In order to get the depencencies for a running container, a rebuild is necessary.

### Rebuild
When reconfiguring the container, a rebuild might be necessary.
This can be done by passing the ```REBUILD``` flag when starting the container.
```
.\rosws.sh REBUILD=true
``` 

# rascl_ros2_control
## The holy grale of documentation
- seems to be a ros book: https://codewithurooj.github.io/my_book/docs/chapter-2-ros2/launch-parameters
- launch file guide: 
    - https://notes.rdu.im/robotics/ros/ros2_launch/#sample-launch
    - https://roboticsbackend.com/ros2-launch-file-example/
- on motion planning: https://manipulation.csail.mit.edu/
- ros2 node lifecycle: 
    - https://design.ros2.org/articles/node_lifecycle.html
    - https://github.com/ros2/demos/blob/rolling/lifecycle/README.rst
- writing a simple python publisher and subscriber: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html
- how to use actions: https://github.com/ros2/examples/tree/master/rclpy/actions
- rclpy source code: https://github.com/ros2/rclpy/

## Launching arviz
The docker container needs sufficent access rights to launch a display.
Execute the following:
```bash
xhost +local:docker
```

## Writing HWI
1. finish tutorial on how to write [HWI](https://control.ros.org/rolling/doc/ros2_control/hardware_interface/doc/writing_new_hardware_component.html)
2. [7 DoF example](https://control.ros.org/jazzy/doc/ros2_control_demos/example_7/doc/userdoc.html) with more information on HWI class declaration
-> hardware specific communication in
    - `read` method getting the states from the hardware and storing them to internal variables defined in export_state_interfaces
    - `write` method that commands the hardware based on the values stored in internal variables defined in export_command_interfaces
3. [Ros2 control source](https://github.com/ros-controls/ros2_control/blob/master/hardware_interface/src/hardware_component.cpp)
## Cheat Sheet for ROS2 Commands
- `ros2 launch rascl_description ros2_control.launch.py`: start the ros2 control stack loading the robot from urdf and setting up required services
- `ros2 node list`: list all available nodes
- `ros2 topic list`: list all available topics
- `ros2 topic echo /joint_states`: subscribe to the topic with the current cli window (prints the data in real time)
- `ros2 topic hz /<topic>`: shows publishing rate
- `ros2 topic info /<topic>`: shows topic details
- `ros2 control list_controllers`: See all controllers and their state
- `ros2 control list_hardware_components`: See hardware component status
- `colcon build --packages-select rascl_hardware_interface`
- list interfaces (control strings and msgs): `ros2 interface list | grep control_msg`


## Control Modes in Robotics
When should we use *position*, *velocity* or *torque control*? Check out these blog posts for insides:
https://robotics.stackexchange.com/questions/10052/position-control-vs-velocity-control-vs-torque-control

## Kinematics Libraries in ROS2
- orocos_kdl: https://github.com/orocos/orocos_kinematics_dynamics
- trac_ik - they say its faster than kdl (IK solver): https://docs.ros.org/en/ros2_packages/jazzy/api/trac_ik/
- pinocchio - very fast for FK and Jacobians: https://docs.ros.org/en/ros2_packages/jazzy/api/pinocchio/index.html
- Drake - optimization based lib (careful: in experimental state): https://github.com/RobotLocomotion/drake-ros
- Ruckig for time-parameterization

chatGPT notice: When solving IK for via points, always use seeded IK:
Solve waypoint 1 with a reasonable initial guess. Then, solve waypoint k using the solution of waypoint k−1 as the seed. This massively improves continuity and avoids “elbow flips” (TRAC-IK and MoveIt support this pattern; Pinocchio/Drake you implement it).

## pick and place
how to publish:

ros2 topic pub --once /pick_and_place_task std_msgs/msg/String "{data: '{\"positions\": {\"prepick\": [-1.27, -0.65, 0.4, 0.0], \"pick\": [-1.27, -0.8, 0.4, 0.0], \"grasped\": [-1.27, -0.8, 0.4, -0.9], \"safety\": [0, -0.2, 0.4, -0.9], \"preplace\": [1.5, -0.65, 0.35, -0.9], \"place\": [1.5, -0.85, 0.25, -0.9], \"unloaded\": [1.5, -0.85, 0.25, 0.0], \"postplace\": [1.5, -0.65, 0.35, 0.0]}}'}"


