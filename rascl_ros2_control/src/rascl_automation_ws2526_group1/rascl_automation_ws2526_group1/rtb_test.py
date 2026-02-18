#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String
import threading

# dependency for roboticstoolbox 
import roboticstoolbox as rtb
from roboticstoolbox import ERobot
import os
from spatialmath import SE3
import swift
import spatialmath as sm
import numpy as np
from spatialmath.base import *


class RtbTest(Node):
    def __init__(self):
        super().__init__('rtb_test')

        # Action client to joint_trajectory_controller
        self._ac = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        # Optional: publish a simplified result for upper-level node/UI
        self.result_pub = self.create_publisher(String, 'rtbtest/motion_result', 10)

        self.get_logger().info("rtbtest started. Waiting for action server...")

    def send_target(self, joint_names, target_positions, duration_sec=4.0):
        """
        EN: Send a single-point trajectory (go to target joint angles).
        
        """

        # 1) Wait for controller action server
        if not self._ac.wait_for_server(timeout_sec=5.0):
            msg = "Action server not available: /joint_trajectory_controller/follow_joint_trajectory"
            self.get_logger().error(msg)
            self.result_pub.publish(String(data=f"FAILED: {msg}"))
            return

        # 2) Build goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = list(target_positions)
        pt.time_from_start.sec = int(duration_sec)
        pt.time_from_start.nanosec = int((duration_sec - int(duration_sec)) * 1e9)
        goal.trajectory.points = [pt]

        # Optional tolerances (leave default unless you need strict control)
        # goal.goal_time_tolerance.sec = 1

        self.get_logger().info(f"Sending target: {target_positions} in {duration_sec}s")

        # 3) Send goal async, with feedback callback
        send_future = self._ac.send_goal_async(goal, feedback_callback=self.feedback_cb)
        send_future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, feedback_msg):
        # EN: feedback_msg.feedback has desired/actual/error for joints (depends on controller impl)
        
        self.get_logger().debug("Feedback received.")

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            msg = "Goal rejected by controller."
            self.get_logger().error(msg)
            self.result_pub.publish(String(data=f"FAILED: {msg}"))
            return

        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        status = future.result().status

        # status is a GoalStatus code (SUCCEEDED=4 in ROS2 action_msgs)
        # FollowJointTrajectory.Result has error_code + error_string (commonly used)
        if result.error_code == 0:
            msg = f"SUCCEEDED (status={status})"
            self.get_logger().info(msg)
            self.result_pub.publish(String(data=msg))
        else:
            msg = f"FAILED (status={status}) error_code={result.error_code}, error_string='{result.error_string}'"
            self.get_logger().error(msg)
            self.result_pub.publish(String(data=msg))


def keyboard_loop(node):
    joint_names = ["shoulder_joint", "upperarm_joint", "lowerarm_joint", "gripper_joint"]
    # 'shoulder_joint', 'upperarm_joint', 'lowerarm_joint', 'gripper_joint'
    cwd = os.getcwd()
    path = cwd + '/src/rascl_description/urdf/rascl.urdf'
    print("Path to urdf file:", path)
    robot = ERobot.URDF(path)
    print(robot)
    print(robot.ets())

    while rclpy.ok():
        try:
            s = input("Enter xyz coordinates (mm): (e.g. -230 50 10 ): ")
            coord = [float(x) for x in s.split()]
            if len(coord) != 3:
                print("Need exactly 3 values")
                continue
            values = get_joint_config_from_box_coord(robot, coord[0], coord[1], coord[2])
            print(values)
            node.send_target(joint_names, values, duration_sec=4.0)

        except ValueError:
            print("Invalid input")
        except EOFError:
            break

def get_joint_config_from_box_coord(robot, x, y, z):
    x_box = (x + 0) / 1000
    y_box = (y - 0) / 1000
    z_box = (z + 50) / 1000
    # print(np.array([x_box, y_box, z_box]))
    target_pose = SE3(x_box, y_box, z_box)

    sol = robot.ikine_LM(target_pose, mask=[1,1,1,0,0,0], q0 = np.array([0, 0, 0, 0]))
    sol_arr = np.asarray(sol.q, dtype=float).reshape(-1).tolist()
    sol_arr[3]=0.42
    return sol_arr


def main():
    rclpy.init()
    node = RtbTest()

    kb_thread= threading.Thread(
        target =keyboard_loop,
        args=(node, ),
        daemon=True
        )
    kb_thread.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
