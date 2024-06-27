#!/usr/bin/env python
import rospy
import sys
from kuavoRobotSDK import kuavo
import time

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: robot arm toHome.py <flower_tag>")
        sys.exit(1)

    flower_tag = int(sys.argv[1])  # 获取命令行参数并转换为整数

    kuavo_robot = kuavo("kuavo_robot_3.5_awe")
   
    time.sleep(2)

    kuavo_robot.set_action_play_tohome(flower_tag, True) 