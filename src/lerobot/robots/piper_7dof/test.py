from piper_7dof_sdk_interface import Piper_7dofSDKInterface
import time

sdk = Piper_7dofSDKInterface("can0")

#---------------------------
gripper_status = sdk.piper.GetArmGripperMsgs()
current_angle = gripper_status.gripper_state.grippers_angle
sdk.piper.GripperCtrl(
    gripper_angle=current_angle,
    gripper_effort=2000,
    gripper_code=0x03,
    set_zero=0x00,
)
#---------------------------

while True:
    torques = sdk.get_torques()
    print(torques['gripper.tau'])
    time.sleep(1)