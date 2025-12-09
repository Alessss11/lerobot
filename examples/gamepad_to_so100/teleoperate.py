#!/usr/bin/env python

import time
import math

from lerobot.model.kinematics import RobotKinematics
from lerobot.processor import RobotAction, RobotObservation, RobotProcessorPipeline
from lerobot.processor.converters import (
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so100_follower.config_so100_follower import SO100FollowerConfig
from lerobot.robots.so100_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    EEReferenceAndDelta,
    InverseKinematicsEEToJoints,
)
from lerobot.robots.so100_follower.so100_follower import SO100Follower

from lerobot.teleoperators.gamepad.configuration_gamepad import GamepadTeleopConfig
from lerobot.teleoperators.gamepad.teleop_gamepad import GamepadTeleop

from lerobot.utils.robot_utils import precise_sleep
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

FPS = 30
DEADZONE = 0.2  # zona morta per considerare lo stick "rilasciato"


def main():
    # Configurazione robot    
    robot_config = SO100FollowerConfig(
        port="/dev/ttyACM0",
        id="my_awesome_follower_arm",
        use_degrees=True,
    )

    teleop_config = GamepadTeleopConfig(use_gripper=False)

    robot = SO100Follower(robot_config)
    teleop_device = GamepadTeleop(teleop_config)

    kinematics_solver = RobotKinematics(
        urdf_path="./SO101/so101_new_calib.urdf", 
        target_frame_name="gripper_frame_link",
        joint_names=list(robot.bus.motors.keys()),
    )

    gamepad_to_robot_joints_processor = RobotProcessorPipeline[
        tuple[RobotAction, RobotObservation],
        RobotAction,
    ](
        steps=[
            EEReferenceAndDelta(
                kinematics=kinematics_solver,
                end_effector_step_sizes={"x": 0.02, "y": 0.02, "z": 0.02},
                motor_names=list(robot.bus.motors.keys()),
                use_latched_reference=True,
            ),
            EEBoundsAndSafety(
                end_effector_bounds={
                    "max": [1, 1, 1],
                    "min": [-1, -1, -1]
                },
                max_ee_step_m=0.02,
            ),
            InverseKinematicsEEToJoints(
                kinematics=kinematics_solver,
                motor_names=list(robot.bus.motors.keys()),
                initial_guess_current_joints=True,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    robot.connect()
    teleop_device.connect()

    init_rerun(session_name="gamepad_so100_teleop")

    if not robot.is_connected or not teleop_device.is_connected:
        raise ValueError("Robot or teleop is not connected!")

    print("Gamepad controls:")
    print("  Left analog stick: Move in X-Y plane")
    print("  Right analog stick (vertical): Move in Z axis")
    print("  B/Circle button: Exit")
    print("  Y/Triangle button: End episode with SUCCESS")
    print("  A/Cross button: End episode with FAILURE")
    print("  X/Square button: Rerecord episode")
    print("Starting teleop loop. Move your gamepad to teleoperate the robot...")

    ee_gripper_pos = 0.0

    while True:
        t0 = time.perf_counter()

        robot_obs = robot.get_observation()
        gamepad_action = teleop_device.get_action()

        dx = gamepad_action.get("delta_x", 0.0)
        dy = gamepad_action.get("delta_y", 0.0)
        dz = gamepad_action.get("delta_z", 0.0)

        mag = max(abs(dx), abs(dy), abs(dz))
        enabled = mag > DEADZONE

        if not enabled:
            tx = ty = tz = 0.0
            twx = twy = twz = 0.0
        else:
            tx = dx
            ty = dy
            tz = dz
            twx = twy = twz = 0.0

        action_for_ik = {
            "enabled": enabled,
            "target_x": tx,
            "target_y": ty,
            "target_z": tz,
            "target_wx": twx,
            "target_wy": twy,
            "target_wz": twz,
            "gripper_vel": 0.0,
            "ee.gripper_pos": ee_gripper_pos,
        }

        joint_action = gamepad_to_robot_joints_processor((action_for_ik, robot_obs))
        _ = robot.send_action(joint_action)

        log_rerun_data(observation=gamepad_action, action=joint_action)

        precise_sleep(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))


if __name__ == "__main__":
    main()

