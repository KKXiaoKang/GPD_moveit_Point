#!/usr/bin/env python
import rospy
import sys
from kuavoRobotSDK import kuavo

if __name__ == "__main__":
    
    if len(sys.argv) < 2:
        print("Usage: robot joller.py <flower_tag>")
        sys.exit(1)

    joller_tag = int(sys.argv[1])  # 获取命令行参数并转换为整数
    
    kuavo_robot = kuavo("kuavo_robot_3.5_awe")

    if joller_tag == 1:
        kuavo_robot.set_robot_joller_position(250, 250)
    elif joller_tag == 2:
        kuavo_robot.set_robot_joller_position(10, 10)