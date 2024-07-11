def make_robot(name):
    if name == "koch":
        # TODO(rcadene): Add configurable robot from command line and yaml config
        # TODO(rcadene): Add example with and without cameras
        from lerobot.common.robot_devices.cameras.opencv import OpenCVCamera
        from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
        from lerobot.common.robot_devices.robots.koch import KochRobot

        robot = KochRobot(
            leader_arms={
                "main": DynamixelMotorsBus(
                    port="/dev/tty.usbmodem575E0031751",
                    motors={
                        # name: (index, model)
                        "shoulder_pan": (1, "xl330-m077"),
                        "shoulder_lift": (2, "xl330-m077"),
                        "elbow_flex": (3, "xl330-m077"),
                        "wrist_flex": (4, "xl330-m077"),
                        "wrist_roll": (5, "xl330-m077"),
                        "gripper": (6, "xl330-m077"),
                    },
                ),
            },
            follower_arms={
                "main": DynamixelMotorsBus(
                    port="/dev/tty.usbmodem575E0032081",
                    motors={
                        # name: (index, model)
                        "shoulder_pan": (1, "xl430-w250"),
                        "shoulder_lift": (2, "xl430-w250"),
                        "elbow_flex": (3, "xl330-m288"),
                        "wrist_flex": (4, "xl330-m288"),
                        "wrist_roll": (5, "xl330-m288"),
                        "gripper": (6, "xl330-m288"),
                    },
                ),
            },
            cameras={
                "laptop": OpenCVCamera(0, fps=30, width=640, height=480),
                "phone": OpenCVCamera(1, fps=30, width=640, height=480),
            },
        )
    elif name == "aloha":
        from lerobot.common.robot_devices.cameras.opencv import OpenCVCamera
        from lerobot.common.robot_devices.motors.dynamixel import DynamixelMotorsBus
        from lerobot.common.robot_devices.robots.aloha import AlohaRobot

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
            # "gripper": (9, "xl430-w250"),
            "gripper": (9, "xc430-w150"),
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
                    # port="/dev/ttyDXL_master_left",
                    port="/dev/tty.usbserial-FT89FM77",
                    motors=widow_x_motors,
                ),
                "right": DynamixelMotorsBus(
                    # port="/dev/ttyDXL_master_right",
                    port="/dev/tty.usbserial-FT891KPN",
                    motors=widow_x_motors,
                ),
            },
            follower_arms={
                "left": DynamixelMotorsBus(
                    # port="/dev/ttyDXL_puppet_left",
                    port="/dev/tty.usbserial-FT891KBG",
                    motors=viper_x_motors,
                ),
                "right": DynamixelMotorsBus(
                    # port="/dev/ttyDXL_puppet_right",
                    port="/dev/tty.usbserial-FT89FM09",
                    motors=viper_x_motors,
                ),
            },
            # cameras={
            #     "cam_high": OpenCVCamera(0, fps=30, width=640, height=480),
            #     "cam_low": OpenCVCamera(1, fps=30, width=640, height=480),
            #     "cam_left_wrist": OpenCVCamera(2, fps=30, width=640, height=480),
            #     "cam_right_wrist": OpenCVCamera(3, fps=30, width=640, height=480),
            # },
        )
    else:
        raise ValueError(f"Robot '{name}' not found.")

    return robot
