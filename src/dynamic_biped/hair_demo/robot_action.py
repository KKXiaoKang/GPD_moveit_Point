#!/usr/bin/env python
import rospy
import sys
from kuavoRobotSDK import kuavo

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: robot_action.py <action_name>")
        sys.exit(1)

    action_name = sys.argv[1].lower()  # 获取命令行参数并转换为小写

    kuavo_robot = kuavo("kuavo_robot_3.5_awe")

    result = None

    if action_name == "say_hello":
        result = kuavo_robot.set_action_play_hello(True)   # 执行打招呼的动作
    elif action_name == "bowing":
        result = kuavo_robot.set_action_play_bowing(True)  # 执行作揖的动作
    else:
        print(f"Unknown action: {action_name}")
        sys.exit(1)

    print(result)
