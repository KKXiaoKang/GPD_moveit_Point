#!/usr/bin/env python
# -*- coding: utf-8 -*-
import argparse
import rospy
from kuavoRobotSDK import kuavo
import sys
import os
import math
import termios
import threading
import time
import numpy as np
import rospy
import rospkg
import yaml
from std_msgs.msg import *
from geometry_msgs.msg import *
import tty
import select

class keyboardlinstener(object):
    def __init__(self):
        super(keyboardlinstener,self).__init__()
        self.key_val = ""
        self.update = False

    def getKey(self, key_timeout):
        settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ' '
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


def menu():
    print("选择要播放的音乐,直接点击对应序号即可,无需按下Enter:")
    print("1. 茉莉花花语")
    print("2. 玫瑰花语")
    print("3. 郁金香花语")
    print("4. 你好小朋友")
    print("5. 你好女士")
    print("6. 你好先生")
    print("7. 再见")
    print("8. 全场打招呼")
    print("9. 退出音乐播放")

def main():
    rospy.init_node("robot_node_demo_control")
    parser = argparse.ArgumentParser(description="Kuavo Robot Control Script")
    args = parser.parse_args()

    robot = kuavo("kuavo_robot")  # 请替换成实际的机器人名字
    key_lisnter = keyboardlinstener()

    while True:
        print('\033c')
        menu()
        # choice = input("请输入选择的操作编号 (6 退出): ")
        choice = key_lisnter.getKey(0.1)

        if choice == 'q':
            break

        try:
            choice = int(choice)
        except ValueError:
            print("无效的选择，请输入数字。")
            continue

        if choice == 1:
            music_string = "Lily_Flower"
            robot.set_robot_play_music(music_string, 0)
        elif choice == 2:
            music_string = "Rose_Flower"
            robot.set_robot_play_music(music_string, 0)
        elif choice == 3:
            music_string = "Tulip_Flower"
            robot.set_robot_play_music(music_string, 0)
        elif choice == 4:
            music_string = "say_hello_kids"
            robot.set_robot_play_music(music_string, 0)
        elif choice == 5:
            music_string = "say_hello_lady"
            robot.set_robot_play_music(music_string, 0)
        elif choice == 6:
            music_string = "say_hello_sir"
            robot.set_robot_play_music(music_string, 0)  
        elif choice == 7:
            music_string = "say_goodbye"
            robot.set_robot_play_music(music_string, 0)  
        elif choice == 8:
            music_string = "everybody_hello"
            robot.set_robot_play_music(music_string, 0)    
        elif choice == 9:
            # 退出遥控操作
            print("退出遥控操作...")
            break         
        else:
            print("无效的选择, 请选择1-9之间的数字。")

if __name__ == "__main__":
    main()
