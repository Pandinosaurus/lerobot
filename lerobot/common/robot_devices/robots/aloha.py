import time
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
import torch

from lerobot.common.robot_devices.cameras.utils import Camera
from lerobot.common.robot_devices.motors.dynamixel import (
    DriveMode,
    OperatingMode,
    TorqueMode,
)
from lerobot.common.robot_devices.motors.utils import MotorsBus
from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError

########################################################################
# Aloha / Aloha2 robot arm
########################################################################


def reset_arm(arm: MotorsBus):
    # Note: same as in `koch.py``
    # To be configured, all servos must be in "torque disable" mode
    arm.write("Torque_Enable", TorqueMode.DISABLED.value)

    # Use 'extended position mode' for all motors except gripper, because in joint mode the servos can't
    # rotate more than 360 degrees (from 0 to 4095) And some mistake can happen while assembling the arm,
    # you could end up with a servo with a position 0 or 4095 at a crucial point See [
    # https://emanual.robotis.com/docs/en/dxl/x/x_series/#operating-mode11]
    all_motors_except_gripper = [name for name in arm.motor_names if name != "gripper"]
    arm.write("Operating_Mode", OperatingMode.EXTENDED_POSITION.value, all_motors_except_gripper)

    # TODO(rcadene): why?
    # Use 'position control current based' for gripper
    arm.write("Operating_Mode", OperatingMode.CURRENT_CONTROLLED_POSITION.value, "gripper")

    # Make sure the native calibration (homing offset abd drive mode) is disabled, since we use our own calibration layer to be more generic
    arm.write("Homing_Offset", 0)
    arm.write("Drive_Mode", DriveMode.NON_INVERTED.value)


@dataclass
class AlohaRobotConfig:
    """
    Example of usage:
    ```python
    AlohaRobotConfig()
    ```
    """

    # Define all components of the robot
    leader_arms: dict[str, MotorsBus] = field(default_factory=lambda: {})
    follower_arms: dict[str, MotorsBus] = field(default_factory=lambda: {})
    cameras: dict[str, Camera] = field(default_factory=lambda: {})


class AlohaRobot:
    """Trossen Robotics: https://www.trossenrobotics.com/aloha-stationary

    Example of highest frequency teleoperation without camera:
    ```python
    # Defines how to communicate with the motors of the leader and follower arms
    widow_x_motors = {
        # name: (index, model)
        "waist_a": (1, "xm430-w350"),
        "waist_b": (2, "xm430-w350"),
        "shoulder_a": (3, "xm430-w350"),
        "shoulder_b": (4, "xm430-w350"),
        "elbow": (5, "xm430-w350"),
        "forearm_roll": (6, "xm430-w350"),
        "wrist_angle": (7, "xm430-w350"),
        "wrist_rotate": (8, "xl430-w250"),
        "gripper": (9, "xl430-w250"),
    }
    viper_x_motors = {
        # name: (index, model)
        "waist_a": (1, "xm540-w270"),
        "waist_b": (2, "xm540-w270"),
        "shoulder_a": (3, "xm540-w270"),
        "shoulder_b": (4, "xm540-w270"),
        "elbow": (5, "xm540-w270"),
        "forearm_roll": (6, "xm540-w270"),
        "wrist_angle": (7, "xm540-w270"),
        "wrist_rotate": (8, "xm430-w350"),
        "gripper": (9, "xm430-w350"),
    }
    robot = AlohaRobot(
        leader_arms={
            "left": DynamixelMotorsBus(
                port="/dev/ttyDXL_master_left",
                motors=widow_x_motors,
            ),
            "right": DynamixelMotorsBus(
                port="/dev/ttyDXL_master_right",
                motors=widow_x_motors,
            ),
        },
        follower_arms={
            "left": DynamixelMotorsBus(
                port="/dev/ttyDXL_puppet_left",
                motors=viper_x_motors,
            ),
            "right": DynamixelMotorsBus(
                port="/dev/ttyDXL_puppet_right",
                motors=viper_x_motors,
            ),
        },
    )

    # Connect motors buses and cameras if any (Required)
    robot.connect()

    while True:
        robot.teleop_step()
    ```

    Example of highest frequency data collection without camera:
    ```python
    # Assumes leader and follower arms have been instantiated already (see first example)
    robot = AlohaRobot(leader_arms, follower_arms)
    robot.connect()
    while True:
        observation, action = robot.teleop_step(record_data=True)
    ```

    Example of highest frequency data collection with cameras:
    ```python
    # Defines how to communicate with 2 cameras connected to the computer.
    # Here, the webcam of the mackbookpro and the iphone (connected in USB to the macbookpro)
    # can be reached respectively using the camera indices 0 and 1. These indices can be
    # arbitrary. See the documentation of `OpenCVCamera` to find your own camera indices.
    cameras={
        "cam_high": OpenCVCamera(0, fps=30, width=640, height=480),
        "cam_low": OpenCVCamera(1, fps=30, width=640, height=480),
        "cam_left_wrist": OpenCVCamera(2, fps=30, width=640, height=480),
        "cam_right_wrist": OpenCVCamera(3, fps=30, width=640, height=480),
    }

    # Assumes leader and follower arms have been instantiated already (see first example)
    robot = AlohaRobot(leader_arms, follower_arms, cameras)
    robot.connect()
    while True:
        observation, action = robot.teleop_step(record_data=True)
    ```

    Example of controlling the robot with a policy (without running multiple policies in parallel to ensure highest frequency):
    ```python
    # Assumes leader and follower arms + cameras have been instantiated already (see previous example)
    robot = AlohaRobot(leader_arms, follower_arms, cameras)
    robot.connect()
    while True:
        # Uses the follower arms and cameras to capture an observation
        observation = robot.capture_observation()

        # Assumes a policy has been instantiated
        with torch.inference_mode():
            action = policy.select_action(observation)

        # Orders the robot to move
        robot.send_action(action)
    ```

    Example of disconnecting which is not mandatory since we disconnect when the object is deleted:
    ```python
    robot.disconnect()
    ```
    """

    def __init__(
        self,
        config: AlohaRobotConfig | None = None,
        # TODO(rcadene): change calibration?
        calibration_path: Path = ".cache/calibration/koch.pkl",
        **kwargs,
    ):
        if config is None:
            config = AlohaRobotConfig()
        # Overwrite config arguments using kwargs
        self.config = replace(config, **kwargs)
        self.calibration_path = Path(calibration_path)

        self.leader_arms = self.config.leader_arms
        self.follower_arms = self.config.follower_arms
        self.cameras = self.config.cameras
        self.is_connected = False
        self.logs = {}

    def connect(self):
        if self.is_connected:
            raise RobotDeviceAlreadyConnectedError(
                "AlohaRobot is already connected. Do not run `robot.connect()` twice."
            )

        if not self.leader_arms and not self.follower_arms and not self.cameras:
            raise ValueError(
                "AlohaRobot doesn't have any device to connect. See example of usage in docstring of the class."
            )

        # Connect the arms
        for name in self.follower_arms:
            self.follower_arms[name].connect()
            self.leader_arms[name].connect()

        # Reset all arms
        for name in self.follower_arms:
            reset_arm(self.follower_arms[name])
        for name in self.leader_arms:
            reset_arm(self.leader_arms[name])

        # Enable torque on all motors of the follower arms
        for name in self.follower_arms:
            self.follower_arms[name].write("Torque_Enable", 1)

        # Enable torque on the gripper of the leader arms, and move it to 45 degrees,
        # so that we can use it as a trigger to close the gripper of the follower arms.
        for name in self.leader_arms:
            self.leader_arms[name].write("Torque_Enable", 1, "gripper")
            # TODO(rcadene): what value for gripper?
            # self.leader_arms[name].write("Goal_Position", GRIPPER_OPEN, "gripper")

        # Connect the cameras
        for name in self.cameras:
            self.cameras[name].connect()

        self.is_connected = True

    def teleop_step(
        self, record_data=False
    ) -> None | tuple[dict[str, torch.Tensor], dict[str, torch.Tensor]]:
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "AlohaRobot is not connected. You need to run `robot.connect()`."
            )

        # Prepare to assign the positions of the leader to the follower
        leader_pos = {}
        for name in self.leader_arms:
            now = time.perf_counter()
            leader_pos[name] = self.leader_arms[name].read("Present_Position")
            self.logs[f"read_leader_{name}_pos_dt_s"] = time.perf_counter() - now

        follower_goal_pos = {}
        for name in self.leader_arms:
            follower_goal_pos[name] = leader_pos[name]

        # Send action
        for name in self.follower_arms:
            now = time.perf_counter()
            self.follower_arms[name].write("Goal_Position", follower_goal_pos[name])
            self.logs[f"write_follower_{name}_goal_pos_dt_s"] = time.perf_counter() - now

        # Early exit when recording data is not requested
        if not record_data:
            return

        # Read follower position
        follower_pos = {}
        for name in self.follower_arms:
            now = time.perf_counter()
            follower_pos[name] = self.follower_arms[name].read("Present_Position")
            self.logs[f"read_follower_{name}_pos_dt_s"] = time.perf_counter() - now

        # Create state by concatenating follower current position
        state = []
        for name in self.follower_arms:
            if name in follower_pos:
                state.append(follower_pos[name])
        state = np.concatenate(state)

        # Create action by concatenating follower goal position
        action = []
        for name in self.follower_arms:
            if name in follower_goal_pos:
                action.append(follower_goal_pos[name])
        action = np.concatenate(action)

        # Capture images from cameras
        images = {}
        for name in self.cameras:
            now = time.perf_counter()
            images[name] = self.cameras[name].async_read()
            self.logs[f"read_camera_{name}_dt_s"] = self.cameras[name].logs["delta_timestamp_s"]
            self.logs[f"async_read_camera_{name}_dt_s"] = time.perf_counter() - now

        # Populate output dictionnaries and format to pytorch
        obs_dict, action_dict = {}, {}
        obs_dict["observation.state"] = torch.from_numpy(state)
        action_dict["action"] = torch.from_numpy(action)
        for name in self.cameras:
            obs_dict[f"observation.images.{name}"] = torch.from_numpy(images[name])

        return obs_dict, action_dict

    def capture_observation(self):
        """The returned observations do not have a batch dimension."""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "AlohaRobot is not connected. You need to run `robot.connect()`."
            )

        # Read follower position
        follower_pos = {}
        for name in self.follower_arms:
            follower_pos[name] = self.follower_arms[name].read("Present_Position")

        # Create state by concatenating follower current position
        state = []
        for name in self.follower_arms:
            if name in follower_pos:
                state.append(follower_pos[name])
        state = np.concatenate(state)

        # Capture images from cameras
        images = {}
        for name in self.cameras:
            images[name] = self.cameras[name].async_read()

        # Populate output dictionnaries and format to pytorch
        obs_dict = {}
        obs_dict["observation.state"] = torch.from_numpy(state)
        for name in self.cameras:
            # Convert to pytorch format: channel first and float32 in [0,1]
            img = torch.from_numpy(images[name])
            img = img.type(torch.float32) / 255
            img = img.permute(2, 0, 1).contiguous()
            obs_dict[f"observation.images.{name}"] = img
        return obs_dict

    def send_action(self, action: torch.Tensor):
        """The provided action is expected to be a vector."""
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "AlohaRobot is not connected. You need to run `robot.connect()`."
            )

        from_idx = 0
        to_idx = 0
        follower_goal_pos = {}
        for name in self.follower_arms:
            if name in self.follower_arms:
                to_idx += len(self.follower_arms[name].motor_names)
                follower_goal_pos[name] = action[from_idx:to_idx].numpy()
                from_idx = to_idx

        for name in self.follower_arms:
            self.follower_arms[name].write("Goal_Position", follower_goal_pos[name].astype(np.int32))

    def disconnect(self):
        if not self.is_connected:
            raise RobotDeviceNotConnectedError(
                "AlohaRobot is not connected. You need to run `robot.connect()` before disconnecting."
            )

        for name in self.follower_arms:
            self.follower_arms[name].disconnect()

        for name in self.leader_arms:
            self.leader_arms[name].disconnect()

        for name in self.cameras:
            self.cameras[name].disconnect()

        self.is_connected = False

    def __del__(self):
        if getattr(self, "is_connected", False):
            self.disconnect()
