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

# gripper_closed = 0.95
# gripper_open = 0.0

gripper_closed = -0.6
gripper_open = -0.0

is_idle = False

import time
import copy


class MotionPlanner(Node):
    # cube_placements = [
    #     [-55, 105, 25, 245, 30, 5],
    #     [30, 110, 25, 245, 30, 5],
    #     [100, 70, 25, 245, 30, 5],
    # ]

    # placments for our OWN gripper
    cube_placements = [
        [-230, 50, 20, 245, 30, 20],
        [-20, 250, 60, 110, 150, 20],
        [-20, 250, 20, 250, 30, 60],
        [110, 150, 20, 250, 30, 100],
    ]

    # placement for standard gripper
    # cube_placements = [
    #     [-230, 50, 0, 250, 30, 0],
    #     [-20, 250, 40, 110, 150, 5],
    #     [-20, 250, 0, 250, 30, 40],
    #     [110, 150, 0, 250, 30, 80],
    # ]

    placement_index: int
    last_msg: str

    def __init__(self):
        super().__init__("rascl_motion_planner")

        self.subscription = self.create_subscription(
            String, "/pick_and_place_status", self.statemachine_callback, 10
        )
        self.subscription

        self.publisher = self.create_publisher(String, "/pick_and_place_task", 10)

        cwd = os.getcwd()
        path = cwd + "/src/rascl_description/urdf/rascl.urdf"
        print("Path to urdf file:", path)
        self.robot = ERobot.URDF(path)
        print(self.robot)
        print(self.robot.ets(end=self.robot.link_dict["tcp_link"].name))
        print(f"robot link dict: \n{self.robot.link_dict["tcp_link"]}")

        self.placement_index = 0
        self.last_msg = None

    def get_joint_config_from_box_coord(self, robot, x, y, z):
        r = np.sqrt(x**2 + y**2)
        r_new = r + 15
        theta = np.arctan2(y, x)
        x_box = r_new * math.cos(theta) / 1000
        y_box = r_new * math.sin(theta) / 1000
        z_box = z / 1000

        print(np.array([x_box, y_box, z_box]))
        target_pose = SE3(np.array([x_box, y_box, z_box]))

        ik_solution = robot.ikine_LM(
            target_pose, mask=[1, 1, 1, 0, 0, 0], q0=np.array([0, 0, 0]), tol=1e-6, end=self.robot.link_dict["tcp_link"].name
        )
        # end=self.robot.link_dict["tcp_link"]
        print("\nIK-solution: " , ik_solution)

        joint_config = ik_solution.q

        assert isinstance(joint_config, np.ndarray), "Expected numpy array"
        full_joint_config = np.append(joint_config, 0.0)
        assert len(full_joint_config) == 4, (
            f"Expected array of length 4, got {len(full_joint_config)}"
        )

        return full_joint_config

    def get_pick_positions_from_coord(self, robot, x1, y1, z1, x2, y2, z2):
        prepick = self.get_joint_config_from_box_coord(robot, x1, y1, z1 + 40)
        prepick[3] = gripper_open

        pick = self.get_joint_config_from_box_coord(robot, x1, y1, z1)
        pick[3] = gripper_open

        grasped = self.get_joint_config_from_box_coord(robot, x1, y1, z1)
        grasped[3] = gripper_closed

        safety = self.get_joint_config_from_box_coord(robot, x1, y1, z1 + 180)
        safety[3] = gripper_closed

        preplace = self.get_joint_config_from_box_coord(robot, x2, y2, z2 + 60)
        preplace[3] = gripper_closed
        # preplace_ = [round(pos, 2) for pos in preplace]

        place = self.get_joint_config_from_box_coord(robot, x2, y2, z2)
        place[3] = gripper_closed

        unloaded = self.get_joint_config_from_box_coord(robot, x2, y2, z2)
        unloaded[3] = gripper_open

        postplace = copy.deepcopy(preplace)
        postplace[3] = gripper_open

        return prepick, pick, grasped, safety, preplace, place, unloaded, postplace

    def change_cube_position(self, robot, x1, y1, z1, x2, y2, z2):
        prepick, pick, grasped, safety, preplace, place, unloaded, postplace = (
            self.get_pick_positions_from_coord(robot, x1, y1, z1, x2, y2, z2)
        )

        positions = {
            "prepick": prepick.tolist(),
            "pick": pick.tolist(),
            "grasped": grasped.tolist(),
            "safety": safety.tolist(),
            "preplace": preplace.tolist(),
            "place": place.tolist(),
            "unloaded": unloaded.tolist(),
            "postplace": postplace.tolist(),
        }

        # Create message
        msg = String()
        msg.data = json.dumps({"positions": positions})
        print(msg.data)

        self.publisher.publish(msg)

        # time.sleep(45)

    def statemachine_callback(self, msg):
        if (msg.data == "IDLE" and self.last_msg == "BUSY") or (msg.data == "IDLE" and self.last_msg == None):
            # start task
            placements = self.cube_placements[self.placement_index]
            print(
                f"New idle state received: {msg.data}, last msg: {self.last_msg}, starting pick and place {str(self.placement_index + 1)}: {placements}"
            )
            x1 = placements[0]
            y1 = placements[1]
            z1 = placements[2]
            x2 = placements[3]
            y2 = placements[4]
            z2 = placements[5]
            self.change_cube_position(self.robot, x1, y1, z1, x2, y2, z2)
            self.placement_index += 1
            if self.placement_index >= len(self.cube_placements):
                # shutdown
                self.destroy_node()
                rclpy.shutdown()
        self.last_msg = msg.data
        


def main(args=None):
    rclpy.init(args=args)

    mp = MotionPlanner()
    rclpy.spin(mp)

    # # bring cube 1 to goal position
    # change_cube_position(robot, -230, 50, 7, 245, 30, 7)

    # # bring cube 3 to intermediate position
    # change_cube_position(robot, -20, 250, 47, 110, 150, 12)

    # # # stack cube 2 onto cube 1
    # change_cube_position(robot, -20, 250, 7, 255, 30, 47)

    # # # stack cube 3 onto cube 2
    # change_cube_position(robot, 110, 150, 7, 255, 30, 87)


if __name__ == "__main__":
    main()
