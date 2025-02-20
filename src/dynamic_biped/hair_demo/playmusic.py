#!/usr/bin/env python
import rospy
import sys  # 导入sys模块
from kuavoRobotSDK import kuavo

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: playmusic.py <music_filename>")
        sys.exit(1)

    music_filename = sys.argv[1]  # 获取命令行参数作为音乐文件名

    kuavo_robot = kuavo("kuavo_robot_3.5_awe")

    result = kuavo_robot.set_robot_play_music(music_filename, 0)

    print(result)
