import time
from typing_extensions import List

# ros2 node lifecycle related imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from task_msgs.msg import PickAndPlace


# yasmin related imports for FSM
import yasmin
from yasmin import Blackboard, StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

import json
import math

from geometry_msgs.msg import Point

from rascl_automation_ws2526_group1.states import (
    Pick,
    Idle,
    GetIdle,
    Place,
)


class MotionController(Node):
    bb: Blackboard = None
    sm: StateMachine = None

    def __init__(self) -> None:
        super().__init__("pick_and_place_mc")

        # ToDo: initialize listener to wait for a pick and place task
        self.subscription = self.create_subscription(
            String, "/pick_and_place_task", self.task_callback, 10
        )
        self.subscription  # prevent unused variable warning

        # broadcast current state (ready, busy), so other nodes now whether a pick and place task can be sent
        self.publisher_ = self.create_publisher(String, "/pick_and_place_status", 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.broadcast_state)

        # rcply init already called in main, cannot be called two times
        # rclpy.init()
        set_ros_loggers()

        yasmin.YASMIN_LOG_INFO("RASCL MotionController")

        self.bb = Blackboard()

        self.bb["task_done"] = True

        # Create a finite state machine (FSM)
        self.sm = StateMachine(outcomes=["EXIT", "ERROR"])

        # reusable transitioning states
        pick_state = Pick(whole_body=True)
        moving_all = Pick(whole_body=True)
        get_idle = GetIdle(whole_body=True, fixed_target=[0.0, 0.0, 0.0, 0.0])
        place_state = Place(whole_body=True)

        # StateMachine stays idle until a pick&place task is started
        self.sm.add_state(
            "IDLE",
            Idle(),
            transitions={
                "start_task": "PICK",
            },
        )
        self.sm.add_state(
            "GET_IDLE",
            get_idle,
            transitions={
                "succeeded": "IDLE",
                "aborted": "ERROR",  # we need some error state to prevent infinity loops
                "canceled": "ERROR",
            },
        )
        self.sm.add_state(
            "PICK",
            pick_state,
            transitions={
                "succeeded": "PLACE",
                "aborted": "GET_IDLE",
                "canceled": "GET_IDLE",
            },
        )
        self.sm.add_state(
            "PLACE",
            place_state,
            transitions={
                "succeeded": "GET_IDLE",
                "aborted": "GET_IDLE",
                "canceled": "GET_IDLE",
            },
        )

        # Publish FSM information for visualization
        YasminViewerPub(fsm=self.sm, fsm_name="YasminMotionController", node=self)

    def task_callback(self, msg: String):
        # msg only has msg.data object, positions have to be parsed from it
        if msg.data:
            try:
                # Parse JSON and extract positions in one go
                positions = json.loads(msg.data).get("positions")

                self._process_input(positions)

                # start the motion, after successful input processing
                self.bb["task_done"] = False

                yasmin.YASMIN_LOG_INFO(
                    f"Starting pick and place task with {len(positions)} positions"
                )

            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Error: {e}")

    def broadcast_state(self):
        current_state = self.sm.get_current_state()
        msg=String()
        msg.data = "IDLE" if current_state == "IDLE" else "BUSY"
        self.publisher_.publish(msg)
        # if current_state == "IDLE":
        #     # yasmin.YASMIN_LOG_INFO(f"{current_state}")
        #     msg = String()
        #     msg.data = current_state
        #     self.publisher_.publish(msg)

    def run_fsm(self):
        # Execute the FSM
        outcome = self.sm(self.bb)  # run state machine with blackboard
        if rclpy.ok():
            yasmin.YASMIN_LOG_INFO(outcome)

    def stop_fsm(self):
        if self.sm.is_running():
            self.sm.cancel_state()

        # run in terminal with ros2 run rascl_automation_ws2526_group1 automation_tsk1

    def _process_input(self, joint_positions: dict):
        """
        Process and validate incoming joint positions from topic.

        Args:
            joint_positions: Dictionary with position names as keys,
                            lists of joint values as values

        Expected format:
        {
            "prepick": [-1.27, -0.65, 0.4, 0.0],
            "pick": [-1.27, -0.8, 0.4, 0.0],
            ...
        }
        """
        # Define required position names
        required_positions = [
            "prepick",
            "pick",
            "grasped",
            "safety",
            "preplace",
            "place",
            "unloaded",
            "postplace",
        ]

        # Validate: check all required positions are present
        assert len(joint_positions) == 8, (
            f"Expected 8 positions, got {len(joint_positions)}"
        )

        for pos_name in required_positions:
            assert pos_name in joint_positions, f"Missing required position: {pos_name}"

        # Validate: each position should have 4 joint values
        for pos_name, pos_values in joint_positions.items():
            assert len(pos_values) == 4, (
                f"Position '{pos_name}' must have 4 values, got {len(pos_values)}"
            )

        # Store in blackboard using dictionary keys
        self.bb["prepick"] = joint_positions["prepick"]
        self.bb["pick"] = joint_positions["pick"]
        self.bb["grasped"] = joint_positions["grasped"]
        self.bb["safety"] = joint_positions["safety"]
        self.bb["preplace"] = joint_positions["preplace"]  # ← Fixed from [4]
        self.bb["place"] = joint_positions["place"]  # ← Fixed from [5]
        self.bb["unloaded"] = joint_positions["unloaded"]  # ← Fixed from [6]
        self.bb["postplace"] = joint_positions["postplace"]  # ← Fixed from [7]

        yasmin.YASMIN_LOG_INFO(
            f"Loaded {len(joint_positions)} positions into blackboard"
        )

    def pick_and_place(self, joint_positions: List[List[float]]):
        # ToDo: extract joint positions and set them in the blackboard for further use
        self._process_input(joint_positions)
        self.bb["task_done"] = False

        while not self.bb["task_done"]:
            time.sleep(1)
