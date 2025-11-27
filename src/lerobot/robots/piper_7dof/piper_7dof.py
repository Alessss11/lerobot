from dataclasses import dataclass, field
from typing import Any

from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots import Robot, RobotConfig

from .piper_7dof_sdk_interface import Piper_7dofSDKInterface

@RobotConfig.register_subclass("piper_7dof")
@dataclass
class Piper_7dofConfig(RobotConfig):
    port: str
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "cam_1": OpenCVCameraConfig(
                index_or_path=0,
                fps=30,
                width=640,
                height=480,
            ),
        }
    )

class Piper_7dof(Robot):
    config_class = Piper_7dofConfig
    name = "piper_7dof"

    _SDK_JOINT_KEYS = (
        "joint_1.pos",
        "joint_2.pos",
        "joint_3.pos",
        "joint_4.pos",
        "joint_5.pos",
        "joint_6.pos",
        "gripper.pos",
    )

    _JOINT_KEYS = (
        "joint_1.pos",
        "joint_2.pos",
        "joint_3.pos",
        "joint_5.pos",
        "joint_6.pos",
        "gripper.pos",
        "joint_4.pos",
    )

    def __init__(self, config: Piper_7dofConfig):
        super().__init__(config)
        self.sdk = Piper_7dofSDKInterface(port=config.port)
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {joint: float for joint in self._JOINT_KEYS}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras}

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        # Assume always connected after SDK init
        return True

    def connect(self, calibrate: bool = True) -> None:  # noqa: ARG002
        # Already connected in SDK init
        for cam in self.cameras.values():
            cam.connect()
        self.configure()

    def disconnect(self) -> None:
        self.sdk.disconnect()
        for cam in self.cameras.values():
            cam.disconnect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:  # noqa: D401
        """No calibration routine is required for this SDK."""

    def configure(self) -> None:  # noqa: D401
        """Hook for additional configuration; not used for the Piper SDK."""

    def get_observation(self) -> dict[str, Any]:
        obs_dict = self.sdk.get_status()

        for joint_key in self._SDK_JOINT_KEYS:
            if joint_key in obs_dict and obs_dict[joint_key] is not None:
                obs_dict[joint_key] = float(obs_dict[joint_key]) / 1000.0

        ordered_obs = {key: obs_dict.get(key) for key in self._JOINT_KEYS}

        for cam_key, cam in self.cameras.items():
            ordered_obs[cam_key] = cam.async_read()
        return ordered_obs

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        ordered_positions = []
        for key in self._SDK_JOINT_KEYS:
            value = action.get(key)
            if value is None:
                raise ValueError(f"Action value for '{key}' cannot be None.")
            ordered_positions.append(value)

        self.sdk.set_joint_positions(ordered_positions)
        return {key: action.get(key) for key in self._JOINT_KEYS}
