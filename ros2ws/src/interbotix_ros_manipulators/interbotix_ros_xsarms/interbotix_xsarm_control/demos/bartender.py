#!/usr/bin/env python3

import sys
import numpy as np

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


def main():
    bot = InterbotixManipulatorXS(
        robot_model='rx200',
        group_name='arm',
        gripper_name='gripper',
    )

    robot_startup()

    # Make sure arm has at least 5 joints
    if bot.arm.group_info.num_joints < 5:
        bot.get_node().logfatal("RX200 requires 5 joints.")
        robot_shutdown()
        sys.exit()

    # 1. Move to a safe starting position
    bot.arm.go_to_home_pose()
    bot.gripper.release()

    # 2. Move directly under the base (x=0, y=0)
    bot.arm.set_ee_pose_components(
        x=0.0,
        y=0.0,
        z=0.05,       # small positive height first (safe)
        roll=0.0,
        pitch=0.0,
        yaw=0.0
    )

    # 3. Move down 15 cm below base_link
    bot.arm.set_ee_cartesian_trajectory(
        z=-0.15       # go straight down
    )

    # Keep gripper open
    bot.gripper.release()

    # OPTIONAL: move back up
    bot.arm.set_ee_cartesian_trajectory(
        z=0.15
    )

    # Finish
    bot.arm.go_to_sleep_pose()
    bot.arm.relax()
    robot_shutdown()


if __name__ == '__main__':
    main()
