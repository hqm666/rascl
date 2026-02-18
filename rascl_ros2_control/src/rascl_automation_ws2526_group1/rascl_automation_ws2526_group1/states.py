import time
from typing import List

import yasmin
from yasmin import State, Blackboard, StateMachine
from yasmin_ros import ActionState
from yasmin_viewer import YasminViewerPub

# control_msgs/action/FollowJointTrajectory
# <controller_name>/follow_joint_trajectory [control_msgs::action::FollowJointTrajectory]
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

# Todo: migrate to action states (handles action client lifecycle)
# yasmin docu: The ActionState automatically manages the action client lifecycle,
# handles timeouts, and provides clean integration with the state machine.

WAIT_CONSTANT = 0.5  # in sec


class Pick(ActionState):
    """
    Generic motion state, which is used as transitioning state to perform a motion from one state to another by moving the robot to a target position.
    Target position is read from blackboard["target_position"] in radians.
    """

    logging_const: int
    whole_body: bool

    def __init__(self, whole_body=False, fixed_target=None):
        super().__init__(
            FollowJointTrajectory,  # action type
            "joint_trajectory_controller/follow_joint_trajectory",  # action name
            self.create_goal_handler,  # callback to create the goal
            None,  # outcomes: SUCCEED, ABORT, CANCEL
            self.response_handler,  # callback to process the response
            self.print_feedback,  # callback to process the feedback
        )
        self.fixed_target = fixed_target
        self.whole_body = whole_body

    def create_goal_handler(self, blackboard: Blackboard) -> FollowJointTrajectory.Goal:
        """
        Creates trajectory goal from blackboard target position.
        """

        goal = FollowJointTrajectory.Goal()

        # Get configuration from blackboard
        # ToDo
        joint_names = [
            "shoulder_joint",
            "upperarm_joint",
            "lowerarm_joint",
            "gripper_joint",
        ]
        # if not self.whole_body:
        #     # only move non-gripper joints
        #     joint_names.pop()
        #     if self.fixed_target:
        #         target = self.fixed_target[:3]
        #         target_positions = target
        #     else:
        #         target_positions = blackboard["target_position"]
        #         target_positions = target_positions[:3]  # remove the gripper pos
        #
        #     assert len(target_positions) == 3, (
        #         "expected only three joint positions for a robot motion"
        #     )
        # else:
        #     # allow to move all joints
        #     if self.fixed_target:
        #         target_positions = self.fixed_target
        #     else:
        #         target_positions = blackboard["target_position"]
        #
        #     assert len(target_positions) == 4, (
        #         "expected 4 joint positions for a whole-body robot motion"
        #     )

        # Build trajectory
        goal.trajectory.joint_names = joint_names

        duration = 2.0
        prepick = JointTrajectoryPoint()
        prepick.positions = blackboard["prepick"]
        prepick.time_from_start.sec = int(duration)
        prepick.time_from_start.nanosec = int((duration % 1) * 1e9)

        duration += 1.0
        pick = JointTrajectoryPoint()
        pick.positions = blackboard["pick"]
        pick.time_from_start.sec = int(duration)
        pick.time_from_start.nanosec = int((duration % 1) * 1e9)

        duration += 2
        grasped = JointTrajectoryPoint()
        grasped.positions = blackboard["grasped"]
        grasped.time_from_start.sec = int(duration)
        grasped.time_from_start.nanosec = int((duration % 1) * 1e9)

        goal.trajectory.points = [
            prepick,
            pick,
            grasped,
        ]  # end sequence here to have precise grasp

        rounded_prepick = [round(pos, 2) for pos in blackboard["prepick"]]
        rounded_pick = [round(pos, 2) for pos in blackboard["pick"]]
        rounded_grasped = [round(pos, 2) for pos in blackboard["grasped"]]

        message = f"Moving to positions: \nprepick: {rounded_prepick}, \npick: {rounded_pick}, \ngrasp: {rounded_grasped}"
        yasmin.YASMIN_LOG_INFO(message)

        self.logging_const = 0

        return goal

    def response_handler(
        self, blackboard: Blackboard, response: FollowJointTrajectory.Result
    ) -> str:
        """
        Handles trajectory execution result.
        """
        if response.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            yasmin.YASMIN_LOG_INFO("Motion completed successfully")
            # Store that we reached the target
            blackboard["motion_successful"] = True
            return "succeeded"
        else:
            yasmin.YASMIN_LOG_ERROR(
                f"Motion failed with error code: {response.error_code}"
            )
            blackboard["motion_successful"] = False
            return "aborted"

    def print_feedback(
        self, blackboard: Blackboard, feedback: FollowJointTrajectory.Feedback
    ) -> None:
        """
        Prints motion feedback (during execution).
        """

        if self.logging_const % 10 == 0:
            # yasmin.YASMIN_LOG_INFO(
            #     f"Executing motion... {type(feedback.actual.positions)}"
            # )
            act_pos = feedback.actual.positions
            # yasmin.YASMIN_LOG_INFO(
            #     f"current pos: q1={round(feedback.actual.positions[0], 2)}, q2={round(feedback.actual.positions[1], 2)} q3={round(feedback.actual.positions[2], 2)} q4={round(feedback.actual.positions[3], 2)}"
            # )

        if self.logging_const <= 10:
            self.logging_const += 1
        else:
            self.logging_const = 0


class Place(Pick):
    def __init__(self, whole_body=False, fixed_target=None):
        super().__init__(whole_body, fixed_target)

    def create_goal_handler(self, blackboard: Blackboard) -> FollowJointTrajectory.Goal:
        # wait a short time before continuing
        time.sleep(2)

        goal = FollowJointTrajectory.Goal()

        joint_names = [
            "shoulder_joint",
            "upperarm_joint",
            "lowerarm_joint",
            "gripper_joint",
        ]

        goal.trajectory.joint_names = joint_names

        duration = 2.0
        safety = JointTrajectoryPoint()
        safety.positions = blackboard["safety"]
        safety.time_from_start.sec = int(duration)
        safety.time_from_start.nanosec = int((duration % 1) * 1e9)

        duration += 1.5
        preplace = JointTrajectoryPoint()
        preplace.positions = blackboard["preplace"]
        preplace.time_from_start.sec = int(duration)
        preplace.time_from_start.nanosec = int((duration % 1) * 1e9)

        duration += 1.0
        place = JointTrajectoryPoint()
        place.positions = blackboard["place"]
        place.time_from_start.sec = int(duration)
        place.time_from_start.nanosec = int((duration % 1) * 1e9)

        duration += 2
        unloaded = JointTrajectoryPoint()
        unloaded.positions = blackboard["unloaded"]
        unloaded.time_from_start.sec = int(duration)
        unloaded.time_from_start.nanosec = int((duration % 1) * 1e9)

        duration += 1
        postplace = JointTrajectoryPoint()
        postplace.positions = blackboard["postplace"]
        postplace.time_from_start.sec = int(duration)
        postplace.time_from_start.nanosec = int((duration % 1) * 1e9)

        goal.trajectory.points = [safety, preplace, place, unloaded, postplace]

        rounded_safety = [round(pos, 2) for pos in blackboard["safety"]]
        rounded_preplace = [round(pos, 2) for pos in blackboard["preplace"]]
        rounded_place = [round(pos, 2) for pos in blackboard["place"]]
        rounded_unloaded = [round(pos, 2) for pos in blackboard["unloaded"]]

        message = f"Moving to positions: \nsafety: {rounded_safety}, \npreplace: {rounded_preplace}, \nplace: {rounded_place}, \nunloaded: {rounded_unloaded}"
        yasmin.YASMIN_LOG_INFO(message)
        self.logging_const = 0

        return goal


class GetIdle(Pick):
    """
    State for gripper actions, which is used to particularly only perform gripper manipulations perform a motion from one state to another by moving the robot to a target position.
    Target position is read from blackboard["target_position"] in radians.
    """

    def __init__(self, whole_body, fixed_target):
        super().__init__(whole_body=whole_body, fixed_target=fixed_target)
        self.logging_const = 0

    def create_goal_handler(self, blackboard: Blackboard) -> FollowJointTrajectory.Goal:
        """
        Creates trajectory goal from blackboard target position.
        """
        goal = FollowJointTrajectory.Goal()

        # Get configuration from blackboard
        joint_names = [
            "shoulder_joint",
            "upperarm_joint",
            "lowerarm_joint",
            "gripper_joint",
        ]

        duration = 2

        # mark the pick and place task as done
        blackboard["task_done"] = True

        # Build trajectory
        goal.trajectory.joint_names = joint_names

        # requires change in the controller configuration to allow only partial joints
        # Joints on incoming trajectory don't match the controller joints.
        point = JointTrajectoryPoint()
        point.positions = self.fixed_target
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1) * 1e9)

        goal.trajectory.points = [point]

        yasmin.YASMIN_LOG_INFO(f"Moving to position: {self.fixed_target}")

        return goal


# tutorial: https://uleroboticsgroup.github.io/yasmin/4.0.2/tutorials/python/basic_fsm.html
class Idle(State):
    """
    Represents an idle state of the robot.
    """

    def __init__(self) -> None:
        """
        Initializes the FooState instance, setting up the possible transitions.

        Outcomes:
            : Should transition to the prepick position
            error: error detected (e. g. in input positions)
        """
        super().__init__(["start_task", "EXIT"])

    def execute(self, blackboard: Blackboard) -> str:
        """
        Robot is idle and ready to receive tasks

        Args:
            blackboard (Blackboard): The shared data structure for states. (understand later)

        Returns:
            str: The outcome of the execution, which can be "start_pick_and_place".

        Raises:
            Exception: May raise exceptions related to state execution.
        """
        # yasmin.YASMIN_LOG_INFO("(IDLE) Motion Controller is ready to process tasks")

        while blackboard["task_done"]:
            time.sleep(WAIT_CONSTANT)

        return "start_task"
