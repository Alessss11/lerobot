import time
from typing import Any

try:
    from piper_sdk import C_PiperInterface_V2
except ImportError:
    print("Is the piper_sdk installed: pip install piper_sdk")
    C_PiperInterface_V2 = None


class Piper_7dofSDKInterface:

    def __init__(self, port: str = "can0"):
        if C_PiperInterface_V2 is None:
            raise ImportError("piper_sdk is not installed. Please install it with `pip install piper_sdk`.")
        try:
            self.piper = C_PiperInterface_V2(port)
        except Exception as e: 
            print(
                "Failed to initialize Piper SDK: "
                f"{e} Did you activate the can interface with `piper_sdk/can_activate.sh can0 1000000`"
            )
            self.piper = None
            return
        self.piper.ConnectPort()
        time.sleep(0.1)

        print(self.piper.GetArmStatus().arm_status.motion_status)

        if self.piper.GetArmStatus().arm_status.motion_status != 0:
            self.piper.EmergencyStop(0x02)

        if self.piper.GetArmStatus().arm_status.ctrl_mode == 2:
            print("The arm is in teaching mode, the light is green, press the button to exit teaching mode.")
            self.piper.EmergencyStop(0x02)

        while not self.piper.EnablePiper():
            time.sleep(0.01)

        # Set motion control to joint mode at 100% speed
        self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)

        # Get the min and max positions for each joint and gripper
        angel_status = self.piper.GetAllMotorAngleLimitMaxSpd()
        self.min_pos = [
            pos.min_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]
        ] + [0]
        self.max_pos = [
            pos.max_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]
        ] + [10]  # Gripper max position in mm

    def set_joint_positions(self, positions):

        # positions are in -100% to 100% range
        scaled_positions = [
            self.min_pos[i] + (self.max_pos[i] - self.min_pos[i]) * (pos + 100) / 200
            for i, pos in enumerate(positions[:6])
        ]
        scaled_positions = [100.0 * pos for pos in scaled_positions]

        # the gripper is from 0 to 100% range
        scaled_positions.append(self.min_pos[6] + (self.max_pos[6] - self.min_pos[6]) * positions[6] / 100)
        scaled_positions[6] = int(scaled_positions[6] * 10000)  # Convert to mm

        joint_0 = int(-scaled_positions[0])
        joint_1 = int(scaled_positions[1])
        joint_2 = int(scaled_positions[2])
        joint_3 = int(-scaled_positions[3])
        joint_4 = int(scaled_positions[4])
        joint_5 = int(-scaled_positions[5])
        joint_6 = int(scaled_positions[6])

        self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        self.piper.GripperCtrl(joint_6, 1000, 0x01, 0)

    def get_status(self) -> dict[str, Any]:
        joint_status = self.piper.GetArmJointMsgs()
        gripper = self.piper.GetArmGripperMsgs()

        joint_state = joint_status.joint_state
        obs_dict = {
            "joint_1.pos": joint_state.joint_1,
            "joint_2.pos": joint_state.joint_2,
            "joint_3.pos": joint_state.joint_3,
            "joint_4.pos": joint_state.joint_4,
            "joint_5.pos": joint_state.joint_5,
            "joint_6.pos": joint_state.joint_6,
            "gripper.pos": gripper.gripper_state.grippers_angle,
        }

        return obs_dict
    #----------------------------------------------------------------------------------
    def get_torques(self) -> dict[str, float]:
        """Return joint and gripper torques in N·m."""

        # High-speed feedback for joints
        highspd = self.piper.GetArmHighSpdInfoMsgs()
        # Gripper feedback
        gripper = self.piper.GetArmGripperMsgs()

        torques: dict[str, float] = {}

        motors = [
            highspd.motor_1,
            highspd.motor_2,
            highspd.motor_3,
            highspd.motor_4,
            highspd.motor_5,
            highspd.motor_6,
        ]

        # effort è in 0.001 N·m → divido per 1000
        for i, motor in enumerate(motors, start=1):
            torques[f"joint_{i}.tau"] = motor.effort / 1000.0

        # grippers_effort è anche lui in 0.001 N·m
        torques["gripper.tau"] = gripper.gripper_state.grippers_effort / 1000.0
        '''
        joint_state = self.piper.GetArmJointMsgs().joint_state
        print("joint_2 angle:", joint_state.joint_2)
        print("gripper state:", gripper.gripper_state)
        '''
        
        return torques
    #----------------------------------------------------------------------------


    def disconnect(self):
        self.piper.JointCtrl(0, 0, 0, 0, 0, 0)
