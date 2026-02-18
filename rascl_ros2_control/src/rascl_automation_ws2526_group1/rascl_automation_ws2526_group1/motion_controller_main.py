import rclpy
from rascl_automation_ws2526_group1.motion_controller import MotionController
import threading
import time

import roboticstoolbox as rtb
from roboticstoolbox import ERobot
import os
from spatialmath import SE3
import swift
import spatialmath as sm
import numpy as np
from spatialmath.base import *


def get_joint_config_from_box_coord(robot, x, y, z):
    x_box = (x + 0) / 1000
    y_box = (y - 0) / 1000
    z_box = (z + 0) / 1000
    print("\nPosition (x, y, z): ", np.array([x_box, y_box, z_box]))
    target_pose = SE3(np.array([x_box, y_box, z_box]))

    joint_config = robot.ikine_LM(
        target_pose, mask=[1, 1, 1, 0, 0, 0], q0=np.array([0, 0, 0, 0])
    )
    return joint_config


def main():
    rclpy.init()

    mc = MotionController()
    # start state machine in own thread, so the ros node can run here
    mc_thread = threading.Thread(target=mc.run_fsm)
    mc_thread.start()

    try:
        rclpy.spin(mc)

        # prepick = [-1.27, -0.65, 0.4, 0.0]
        # pick = [-1.27, -0.8, 0.4, 0.0]
        # grasped = [-1.27, -0.8, 0.4, -0.9]
        # safety = [0, -0.2, 0.4, -0.9]
        # preplace = [1.5, -0.65, 0.35, -0.9]
        # place = [1.5, -0.85, 0.25, -0.9]
        # unloaded = [1.5, -0.85, 0.25, 0.0]
        # postplace = [1.5, -0.65, 0.35, 0.0]

        # prepick = get_joint_config_from_box_coord(robot, -230, 50, 40).q
        # prepick[3] = -0.5
        #
        # print(prepick)
        #
        # pick = get_joint_config_from_box_coord(robot, -230, 50, 0).q
        # pick[3] = -0.5
        #
        # grasped = get_joint_config_from_box_coord(robot, -230, 50, 0).q
        # grasped[3] = 0.7
        #
        # safety = get_joint_config_from_box_coord(robot, -230, 50, 80).q
        # safety[3] = 0.7
        #
        # preplace = get_joint_config_from_box_coord(robot, 250, 30, 40).q
        # preplace[3] = 0.7
        #
        # place = get_joint_config_from_box_coord(robot, 250, 30, 0).q
        # place[3] = 0.7
        #
        # unloaded = get_joint_config_from_box_coord(robot, 250, 30, 0).q
        # unloaded[3] = -0.5
        #
        # postplace = prepick
        #
        # time.sleep(5)
        # mc.pick_and_place(
        #     [prepick, pick, grasped, safety, preplace, place, unloaded, postplace]
        # )

        # wait for the robot to get back to zero (IDLE) position
        # time.sleep(6)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        mc.stop_fsm()

        # Wait for FSM thread to finish (with timeout)
        mc_thread.join(timeout=4.0)

        mc.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
