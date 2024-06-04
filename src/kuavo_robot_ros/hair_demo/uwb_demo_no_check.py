#!/usr/bin/python3
# -*- coding: utf-8 -*-

import rospy
import math
import tf
import time
import threading
import signal

from kuavo_robot_ros.msg import robotPhase
from kuavo_robot_ros.msg import robotQVTau
from nlink_parser.msg import LinktrackNodeframe2

from kuavoRobotSDK import kuavo

last_phase = None

# 任务的起始点、途经点及目标点坐标
START_POINT_COORDINATE = {
    "x": 1.12899,
    "y": 0.04399
}
MIDDLE_POINT_COORDINATE = {
    "x": 1.08999,
    "y": 1.76000
}
TARGET_POINT_COORDINATE = {
    "x": 0.70899,
    "y": 3.46800
}

LEFT_LIMIT_PIANO = 0.40999
RIGHT_LIMIT_STAIR = 1.32111
LEFT_LIMIT_TABLE = 0.33999
RIGHT_LIMIT_TABLE = 1.32111

X_DISTANCE_THRESHOLD = 0.3
Y_DISTANCE_THRESHOLD = 0.3

# 程序检查当前 uwb 位置频率
OPERATING_FREQUENCY = 2

# 运动状态及对应数值
STAND_STATE = 'P_stand'
STAND_CODE = 0
WALK_STATE = 'P_walk'
WALK_CODE = 1

# 默认使用速度控制模式，对应值为 1
VEL_CONTROL_CODE = 1

DEFAULT_MASTER_ID = 0

class UWBController:
    def __init__(self):
        # 初始化运动控制接口类
        self.robot_instance = kuavo("3_4_kuavo")

        self.last_x = 0
        self.last_y = 0
        self.cur_x = 0
        self.cur_y = 0
        self.target_x = 0
        self.target_y = 0

        self.state = STAND_CODE
        self.yaw = 0

        self.uwb_sub = rospy.Subscriber("/nlink_linktrack_nodeframe2", LinktrackNodeframe2, self.uwb_callback)
        self.robot_state_sub = rospy.Subscriber("/leju_robot_phase", robotPhase, self.state_callback)
        self.robot_QVTau_sub = rospy.Subscriber("/robot_q_v_tau", robotQVTau, self.imu_callback)
    
    def uwb_callback(self, data):
        tmp_x = data.pos_3d[0]
        tmp_y = data.pos_3d[1]

        if self.cur_x == 0 and self.cur_y == 0:
            # 第一次监听数据，记录初始位置
            self.cur_x = tmp_x
            self.cur_y = tmp_y
        else:
            distance = math.sqrt((self.cur_x - tmp_x)**2 + (self.cur_y - tmp_y)**2)
            if distance > 0.1:
                self.last_x = self.cur_x
                self.last_y = self.cur_y
                self.cur_x = tmp_x
                self.cur_y = tmp_y
            else:
                # 当两次读数的距离小于 0.1 时进行过滤处理，减少读数波动带来的误差
                pass
    
    def state_callback(self, data):
        tmp_state = data.mainPhase
        if tmp_state != self.state:
            print("Action state changed, new state: {}".format(tmp_state))
            self.state = tmp_state
    
    def imu_callback(self, data):
        tmp_roll, tmp_pitch, tmp_yaw = tf.transformations.euler_from_quaternion([data.q[1], data.q[2], data.q[3], data.q[0]])
        self.yaw = tmp_yaw
    
    def change_robot_state(self, new_state):
        print("Change state to {}".format(new_state))
        self.robot_instance.set_robot_Phases(DEFAULT_MASTER_ID, new_state, '')
    
    def set_robot_vel(self, vel_y = 0.2, vel_x = 0.1):
        # 在 uwb 坐标系中 y 轴为前后方向
        # set_walk_speed 第二个参数为前后速度，正值向前运动，负值向后运动
        # 在 uwb 坐标系中 x 轴为左右方向
        # set_walk_speed 第三个参数为左右速度，正值向左运动，负值向右运动
        self.robot_instance.set_walk_speed(VEL_CONTROL_CODE, vel_y, vel_x, 0.0)
    
    def turn_robot(self, angle = 15):
        # set_walk_speed 第四个参数为转动角度，正值向左转动，负值向右转动
        self.robot_instance.set_walk_speed(VEL_CONTROL_CODE, 0.0, 0.0, angle)

    def move_to_target(self, target_point, left_limit, right_limit, is_exceeding_allow = False,):
        """
        控制机器人移动到指定坐标位置附近

        Args:
            target: 目标点坐标
            left_limit: 通道最左侧坐标
            right_limit: 通道最右侧坐标
            is_exceeding_allow: 是否允许超过目标点，对于途经点来说不需要准确停在位置上面，只要超过后停止即可；对于终点则需要尽可能准确
        """

        vel_x = 0.0
        vel_y = 0.0

        # 设置机器人进入行走模式，原地踏步 2 秒后开始运动
        self.change_robot_state(WALK_STATE)
        time.sleep(2)

        rate = rospy.Rate(OPERATING_FREQUENCY)
        while not self.is_at_target(target_point, is_exceeding_allow):
            vel_x = 0.0
            vel_y = 0.0
            if self.state == STAND_CODE:
                # 机器人保护后会自动进入站立状态，需要处理此特殊情况
                self.change_robot_state(WALK_STATE)
                time.sleep(1)
            else:
                # TODO: yaw 角偏移处理待补充
                # 移动策略：当过于靠近左右边界时，优先向中心移动，其他情况直接向前直走
                # 移动策略：当抵达终点附近后再统一进行左右位置调整
                if self.cur_x < left_limit or self.cur_x - left_limit < 0.2:
                    vel_x = -0.1
                elif self.cur_x > right_limit or right_limit - self.cur_x < 0.2:
                    vel_x = 0.1
                else:
                    if self.cur_y + Y_DISTANCE_THRESHOLD < target_point["y"]:
                        vel_y = 0.3
                    elif self.cur_y - Y_DISTANCE_THRESHOLD > target_point["y"]:
                        vel_y = -0.1
                    else:
                        # 已经走到目标点附近，开始左右调整
                        if self.cur_x + X_DISTANCE_THRESHOLD < target_point["x"]:
                            vel_x = -0.1
                        elif self.cur_x - X_DISTANCE_THRESHOLD > target_point["x"]:
                            vel_x = 0.1

            self.set_robot_vel(vel_y, vel_x)

            rate.sleep()

        # 抵达目的地，原地踏步 2 秒然后恢复站立
        self.set_robot_vel(0.0, 0.0)
        time.sleep(2)
        self.change_robot_state(STAND_STATE)
        time.sleep(2)
    
    def is_at_target(self, target_point, is_exceeding_allow):
        if is_exceeding_allow:
            if self.cur_y > target_point["y"]:
                return True
        
        distance_x = abs(self.cur_x - target_point["x"])
        distance_y = abs(self.cur_y - target_point["y"])
        if distance_x < X_DISTANCE_THRESHOLD and distance_y < Y_DISTANCE_THRESHOLD:
            return True
        else:
            return False
    
    def execute_action(self):
        # TODO: 后续需要修改动作函数调用
        self.robot_instance.set_robot_arm_ctl_mode(True)
        self.robot_instance.set_action_play_hello(True)
        time.sleep(2)
        self.robot_instance.set_robot_arm_recenter()

def main():
    rospy.init_node('uwb_demo')
    uwb_controller = UWBController()
    time.sleep(1)

    print("stage1: 从起始点移动到中点")
    uwb_controller.move_to_target(MIDDLE_POINT_COORDINATE, LEFT_LIMIT_PIANO, RIGHT_LIMIT_STAIR, True)
    print("到达中点，开始执行动作....")
    uwb_controller.execute_action()

    print("stage2: 从中点移动到桌子")
    uwb_controller.move_to_target(TARGET_POINT_COORDINATE, LEFT_LIMIT_TABLE, RIGHT_LIMIT_TABLE, False)
    print("抵达终点")

if __name__ == '__main__':
    main()