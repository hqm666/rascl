# RASCL Automation Task

After starting the container using the `rosws.sh` script, the packages have to be build:
```bash
rosbuild
rossetup
```

Now, the particular launch files for task 1 or 2 can be executed to launch the automation.

## Gripper Mounting
Until now we only used the standard gripper and not our own. Each time when mounting the gripper, the closed and opened position changes. Because of that, the opened and closed joint position have to be adjusted accordingly after first startup.
We use two constants to individually adjust the closed and opened gripper position: gripper_closed, gripper_open in the `automation_tsk1.py`


## Dependencies

The automation task is built upon the following libraries, which are automatically installed when starting the container:

- yasmin
- roboticstoolbox-python

The implementation of the tasks is done in python.

## Automation Task 1: Offline Motion Planning
Starting the launch file:
`ros2 launch rascl_automation_ws2526_group1 automation_tsk1.launch.py`

The pick and place motion controller waits until the robot is fully initialized. Then, it starts up and independently starts the execution of the chained pick and place tasks to stack three cubes on the goal position in the correct order. 

## Automatin Task 2: Online Motion Planning
Starting the launch file:
`ros2 launch rascl_automation_ws2526_group1 automation_tsk2.launch.py`

Publish a cube position to the motion controller: 
`ros2 topic pub --once /goal_poses geometry_msgs/msg/Point "{x: -230, y: 50, z: 7}"`

## Cube Placement
Cubes can be placed on the most inner, the middle and the outside radius (using the standard RASCL gripper). We did not evaluate the radii for our own gripper yet. We will try to do this for the final presentation.

## Implementation
The implementation of the pick and place motion consists of two parts:

1. state machine (based on yasmin library) for a pick and place motion with joint angles as input
2. the motion planner that incorporates the IK solver of the roboticstoolbox to obtain the correct joint angles from the cartesian position coordinates

These two communicate over the topic `pick_and_place_task` (the state machine subscribes to the topic, the motion planner publishes the joint positions to it).

The state machine uses derived yasmin `State` classes for the individual static positions (prepick, pick, preplace, place, postplace, ...). All motions are executed deriving the yasmin class `ActionState`. There it acts as an action client for the JointTrajectoryController representing transition state between each static position state.

The IK Motion Planner listens to the `/goal_poses` topic to receive online motion planning tasks for task 2.
