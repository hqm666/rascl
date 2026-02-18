import roboticstoolbox as rtb
from roboticstoolbox import ERobot
import os
from spatialmath import SE3
import swift
import spatialmath as sm
import numpy as np
from spatialmath.base import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import math
import threading
from collections import deque

gripper_closed = -0.6
gripper_open = -0.0
# gripper_closed = -1.0
# gripper_open = 0.0

LEN_QUEUE=10

# x_goal = 250
# y_goal = 30
# z_goal = 7

# is_idle = False

import time
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectoryPoint
from trajectory_msgs.msg import JointTrajectory


class MotionPlanner(Node):

    def __init__(self):
        super().__init__('rascl_motion_planner')
        

        self.subscription = self.create_subscription(String, "/pick_and_place_status", self.statemachine_callback, 10)
        self.subscription

        self.sub2 = self.create_subscription(
            Point, "/goal_poses", self.poses_callback, 10
        )
        self.sub2 # prevent unused variable warning

        self.publisher = self.create_publisher(JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10)

        cwd = os.getcwd()
        path = cwd + "/src/rascl_description/urdf/rascl.urdf"
        print("Path to urdf file:", path)
        self.robot = ERobot.URDF(path)
        # @ marks a leaf node of the robotics tree -> all @ marked links are endeffectors
        # if there are multiple end effectors you need to specify which to use as tcp (end=...)
        # print(self.robot)
        # print(self.robot.ets(end=self.robot.link_dict["tcp_link"].name))
        

        self.is_idle=False
        self.busy=False
        self.goal_queue=deque()
        self.lock=threading.Lock()
        self.print_count=0



    def poses_callback(self, msg: Point):
        # msg only has msg.data object, positions have to be parsed from i
        with self.lock:
            qsize=len(self.goal_queue)
            if qsize >=LEN_QUEUE:
                self.get_logger().info(f"The goal queue is full(capacity is {LEN_QUEUE}), this message is rejected...")
            else:
                self.goal_queue.append((msg.x, msg.y, msg.z))
                qsize=len(self.goal_queue)
                self.get_logger().info(f"Enqueued goal: {msg.x}, {msg.y}, {msg.z} (queue={qsize}/{LEN_QUEUE}) ")
            
        self.try_send_msg()
    
    def try_send_msg(self):
        with self.lock:
            if not self.is_idle or self.busy or len(self.goal_queue) == 0:
                return
            x, y, z = self.goal_queue.popleft()
            self.busy = True

        try:
            self.change_cube_position(self.robot, x, y, z, 250, 30, 15)
        except Exception as e:
            self.get_logger().error(f"Failed to send task: {e}")
            # allow next task
            with self.lock:
                self.busy = False


    def get_joint_config_from_box_coord(self, robot, x, y, z):
        r = np.sqrt(x**2 + y**2)
        r_new = r + 15
        theta = np.arctan2(y, x)
        self.get_logger().info(f"theta: {theta}, x: {x}, y: {y}, r: {r}, r_new: {r_new}")
        x_box = r_new * math.cos(theta) / 1000
        y_box = r_new * math.sin(theta) / 1000
        z_box = z / 1000

        # x_box = x / 1000
        # y_box = y / 1000
        # z_box = z / 1000

        print(np.array([x_box, y_box, z_box]))
        target_pose = SE3(np.array([x_box, y_box, z_box]))

        ik_solution = robot.ikine_LM(
            target_pose, mask=[1, 1, 1, 0, 0, 0], q0=np.array([0, 0, 0]), tol = 1e-6, end=self.robot.link_dict["tcp_link"].name
        )

        #print("\nIK-solution: " , ik_solution)

        joint_config = ik_solution.q

        assert isinstance(joint_config, np.ndarray), "Expected numpy array"
        full_joint_config = np.append(joint_config, 0.0)
        assert len(full_joint_config) == 4, (
            f"Expected array of length 4, got {len(full_joint_config)}"
        )
        if ik_solution.success == False:
            self.get_logger().info(f"IK did NOT find a solution for coords: {x_box}, {y_box}, {z_box}")
        else:
            self.get_logger().info(f"IK did find a solution for coords: {x_box}, {y_box}, {z_box}")

        return full_joint_config

    def get_pick_positions_from_coord(self, robot, x1, y1, z1):
        position = self.get_joint_config_from_box_coord(robot, x1, y1, z1)
        position[3] = gripper_open

        return position

    def change_cube_position(self, robot, x1, y1, z1, x2, y2, z2):

        position = self.get_pick_positions_from_coord(robot, x1, y1, z1)

        goal = JointTrajectory()

        joint_names = [
            "shoulder_joint",
            "upperarm_joint",
            "lowerarm_joint",
            "gripper_joint",
        ]

        goal.joint_names = joint_names

        duration = 4.0
        goal_position = JointTrajectoryPoint()
        goal_position.positions = position
        goal_position.velocities = []  # Empty list
        goal_position.accelerations = []  # Empty list
        goal_position.time_from_start.sec = int(duration)
        goal_position.time_from_start.nanosec = int((duration % 1) * 1e9)

        goal.points = [goal_position]

        self.publisher.publish(goal)
        
        #time.sleep(45)

    def statemachine_callback(self, msg):

        self.print_count+=1
        if self.print_count==10:
            self.get_logger().info(f"Received status: {msg.data}")
            self.print_count=0

        # state after the robot finishes its mission
        # now_idle = ("IDLE" in msg.data) or ("idle" in msg.data.lower())
        with self.lock:
            self.is_idle=("IDLE" in msg.data)
            if self.is_idle:
                self.busy = False
        if self.is_idle:
            self.try_send_msg()

def main(args=None):
    print("Hi from automation_tsk2 main")

    rclpy.init(args=args)

    mp = MotionPlanner()
    rclpy.spin(mp)


if __name__ == "__main__":
    main()
