#!/usr/bin/env python3

from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from time import sleep

def main():
    bot = InterbotixManipulatorXS(
        robot_model='rx200',
        group_name='arm',
        gripper_name='gripper'
    )

    robot_startup()

    # Go to home first
    bot.arm.go_to_home_pose()
    sleep(1)

    # FORCE POSITION: manually chosen joint angles
    # These angles push the arm downward below normal IK reachability
    # Format: [base, shoulder, elbow, wrist_angle, wrist_rotate]
    forced_pose = [
        0.0,      # base
        1.0,      # shoulder up
        -0.25,    # elbow down
        0.9,      # wrist up
        0.0       # wrist rotation
    ]

    # Send raw joint command bypassing IK
    bot.arm.set_joint_positions(forced_pose)
    sleep(2)

    # optional return to sleep
    bot.arm.go_to_sleep_pose()

    robot_shutdown()

if __name__ == '__main__':
    main()
