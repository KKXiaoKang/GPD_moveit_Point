#!/usr/bin/python3
import rospy
from kuavoRobotSDK import kuavo
import geometry_msgs.msg
import time
from collections import deque

POSTION_CONTROL = "Position"
VELOCITY_CONTROL = "Velocity"
STEPCONTROL = "Step"

# init robot
kuavo_robot = kuavo("kuavo_robot_3.5_awe")

def Step_demo():
    # 单步控制模式
    kuavo_robot.set_robot_status_ctl_mode(0, STEPCONTROL)
    
    # # 进入到行走状态
    # time.sleep(2)
    # kuavo_robot.set_robot_Phases(0, "P_walk", "")
    # time.sleep(2)

    # 
    time.sleep(2)

    # 发送控制指令 走5步 
    kuavo_robot.set_walk_speed(2, 0, 0.15, 0, 3.0)

def Velocity_demo():
    # 单步控制模式
    kuavo_robot.set_robot_status_ctl_mode(0, VELOCITY_CONTROL)
    
    # # 进入到行走状态
    time.sleep(2)
    kuavo_robot.set_robot_Phases(0, "P_walk", "")
    time.sleep(2)

    # 发送控制指令 往前一直走0.1m/s 
    kuavo_robot.set_walk_speed(1, 0.1, 0, 0)    

if __name__ == "__main__":
    # 
    rospy.init_node('test_kuavo_node')
    
    # 
    Step_demo()


