#!/usr/bin/env python3
import rospy
import os
import subprocess  # 引入 subprocess 模块
import argparse
import sys
import math
import termios
import threading
import time
import numpy as np
import rospkg
import yaml
import tty
import select

# 拍视频临时专用目录
audio_path = os.environ['HOME'] + '/kuavo_ros_application/Music/hair_awe_video'

print(audio_path)

class MusicPlayerNode:
    def __init__(self):
        rospy.init_node('video_music_player_node')
        self.music_directory = audio_path

    def play_music_callback(self, p_music_number:str, p_volume:int):
        try:
            # 获取服务请求的音乐文件序号和声音大小
            music_number = p_music_number
            volume = p_volume

            # 构建音乐文件路径
            music_file = os.path.join(self.music_directory, f"{music_number}.mp3")

            # 使用 subprocess 调用 play 命令播放音乐
            play_command = ['play', '-q', music_file]
            subprocess.call(play_command)

            rospy.loginfo(f"Playing music {music_file} with volume {volume}")

        except Exception as e:
            rospy.logerr(f"Error playing music: {str(e)}")
 

    def run(self):
        rospy.spin()

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
    print("选择要播放的音乐,(短视频拍摄音乐)直接点击对应序号即可,无需按下Enter:")
    print("1. 可可，听说你是个舞蹈老师，刚刚是我特意为你准备的舞蹈你喜欢吗？")
    print("2. 那你可以跟我玩个游戏吗？我问你答，赢了有奖励")
    print("3. 你还记得圆周率完整版是多少吗")
    print("4. 对！就像你的美一样没有尽头")
    print("5. 那Sin平方+COS平方等于多少")
    print("6. 就像我喜欢你，始终如一")
    print("7. 最后一个,C14的半衰期有多少久")
    print("8. 它不及我在冥冥之中等你时间的千分之一，就为了在这把这朵花送给你。")
    print("9. 退出音乐播放")

def main():
    parser = argparse.ArgumentParser(description="Kuavo Robot Control Script")
    args = parser.parse_args()

    robot = MusicPlayerNode()
    key_lisnter = keyboardlinstener() # 创建键盘监听器对象 

    while True:
        print('\033c')
        menu()

        choice = key_lisnter.getKey(0.1)

        if choice == 'q':
            break

        try:
            choice = int(choice)
        except ValueError:
            print("无效的选择，请输入数字。")
            continue

        if choice == 1:
            robot.play_music_callback('001', 0)
        elif choice == 2:
            robot.play_music_callback('002', 0)
        elif choice == 3:
            robot.play_music_callback('003', 0)
        elif choice == 4:
            robot.play_music_callback('004', 0)
        elif choice == 5:
            robot.play_music_callback('005', 0)
        elif choice == 6:
            robot.play_music_callback('006', 0)  
        elif choice == 7:
            robot.play_music_callback('007', 0)  
        elif choice == 8:
            robot.play_music_callback('008', 0)    
        elif choice == 9:
            # 退出遥控操作
            print("退出遥控操作...")
            break         
        else:
            print("无效的选择, 请选择1-9之间的数字。")

if __name__ == "__main__":
    main()
