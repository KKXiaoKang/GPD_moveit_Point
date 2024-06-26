#!/usr/bin/python3
import rospy
from kuavoRobotSDK import kuavo
import geometry_msgs.msg
import time
from collections import deque

g_walk_flag = True # 判断是否上实物进行测试

POSTION_CONTROL = "Position"
VELOCITY_CONTROL = "Velocity"
STEPCONTROL = "Step"

position_cmd = 0
velocity_cmd = 1
step_cmd = 2

class UWBToPoint:
    def __init__(self):
        self.pose_bias_sub = rospy.Subscriber("/pose_bias", geometry_msgs.msg.PoseStamped, self.pose_bias_callback)

        self._pose_bias_x = 0
        self._pose_bias_y = 0
        self._pose_queue = deque(maxlen=5) # 双向队列
    
    def pose_bias_callback(self, pose):
        self._last_header = pose.header.stamp

        self._pose_bias_x = pose.pose.position.x
        self._pose_bias_y = pose.pose.position.y

        self._pose_queue.append((self._pose_bias_x, self._pose_bias_y)) # 入队列

    def _robot_tag_V_pose_bias(self, kuavo_robot):
        global g_walk_flag

        kuavo_robot.set_robot_status_ctl_mode(0, VELOCITY_CONTROL) # 设置为速度控制模式
        print("速度微调模式....开始！.......")
        print("等待标签信息......")

        if g_walk_flag:
            kuavo_robot.set_robot_Phases(0, 'P_walk', '')

        rate = rospy.Rate(5)  # 设置循环频率为每秒5次，即每0.2秒一次

        while not rospy.is_shutdown():
            if self._pose_queue:
                current_pose_bias_x, current_pose_bias_y = self._pose_queue.pop() # 出队列

                print("当前位姿偏移_x：", current_pose_bias_x, "当前位姿偏移_y：", current_pose_bias_y)

                if g_walk_flag:
                    robot_x_speed = 0
                    robot_y_speed = 0
                    # 校准前后的位置 
                    if current_pose_bias_x > 0:    # 机器人需往 前 走
                        # kuavo_robot.set_walk_speed(1, 0.1, 0, 0.0)   # 机器人向前平移0.1m/s
                        robot_x_speed = 0.08
                    elif current_pose_bias_x < 0:  # 机器人需往 后 走
                        # kuavo_robot.set_walk_speed(1, -0.1, 0, 0.0)  # 机器人向后平移0.1m/s
                        robot_x_speed = -0.08
                
                    # 校准左右的位置
                    if current_pose_bias_y > 0:  # 机器人需往 左 走
                        # kuavo_robot.set_walk_speed(1, 0, 0.05, 0.0)   # 机器人向左平移0.1m/s
                        robot_y_speed = 0.08
                    elif current_pose_bias_y < 0:  # 机器人需往 右 走
                        # kuavo_robot.set_walk_speed(1, 0, -0.05, 0.0)  # 机器人向右平移0.1m/s
                        robot_y_speed = -0.08

                    # setting speed
                    kuavo_robot.set_walk_speed(1, robot_x_speed, robot_y_speed, 0)

                # 如果需要退出循环
                if (abs(current_pose_bias_x) < 0.03 and abs(current_pose_bias_y) < 0.03):
                    time.sleep(1)
                    
                    if g_walk_flag:
                        kuavo_robot.set_walk_speed(1, 0, 0, 0.0) 
                        kuavo_robot.set_robot_Phases(0, 'P_stand', '')

                    print("标签完成！.......")
                    break

            else:
                print("未收到位姿偏移，请检查摄像头中的标签.....")
                if g_walk_flag:
                    kuavo_robot.set_walk_speed(1, 0, 0, 0.0) 

            rate.sleep()  # 等待0.1秒，控制循环频率

        print("tag 速度校准 ok ............")
    
    def _robot_tag_S_pose_bias(self, kuavo_robot):
        time.sleep(1)

        global g_walk_flag
        
        kuavo_robot.set_robot_status_ctl_mode(0, STEPCONTROL) # 设置为单步控制模式
        
        print("进入单步微调模式....开始！.......")
        print("等待标签信息......")

        rate = rospy.Rate(1)  # 设置循环频率为每秒2次，即每0.5秒一次

        while not rospy.is_shutdown(): # 校准3次
            if self._pose_queue:
                current_pose_bias_x, current_pose_bias_y = self._pose_queue.pop() # 出队列

                print("当前位姿偏移_x：", current_pose_bias_x, "当前位姿偏移_y：", current_pose_bias_y)

                # robot setting
                if g_walk_flag:
                    step_num = 0
                    vx_speed = 0
                    vy_speed = 0
                    
                    # 策略：先保证左右对齐稳定后才可以前后校准
                    if current_pose_bias_y > 0.02:
                        # 发送控制指令 走3步
                        print(" step left right num go 2 !.......")
                        vy_speed = 0.1
                        step_num = 3
                    elif current_pose_bias_y < -0.02:
                        vy_speed = -0.1
                        step_num = 3
                    else:
                        if current_pose_bias_x > 0.02:
                            vx_speed = 0.1
                            step_num = 3
                        elif current_pose_bias_x < -0.02:
                            vx_speed = -0.1
                            step_num = 3

                    # setting 设置踏步
                    kuavo_robot.set_walk_speed(step_cmd, vx_speed, vy_speed, 0, step_num)
                    time.sleep(5) 

                # 如果需要退出循环
                if (abs(current_pose_bias_x) < 0.02 and abs(current_pose_bias_y) < 0.02):
                    time.sleep(1)
                    
                    if g_walk_flag:
                        kuavo_robot.set_robot_Phases(0, 'P_stand', '')

                    print("速度微调标签完成！.......")
                    break

            else:
                print("未收到位姿偏移，请检查摄像头中的标签.....")

            rate.sleep()  # 等待0.1秒，控制循环频率

        print("tag 单步控制校准 ok ............")

if __name__ == '__main__':
    rospy.init_node('demo_test')
    robot_instance = kuavo("kuavo_robot_3.5_awe")
    
    uwb_to_point = UWBToPoint()
    time.sleep(2)
    
    # 速度微调 
    uwb_to_point._robot_tag_V_pose_bias(robot_instance)

    # 等待机器人站稳
    time.sleep(3)

    # 步幅微调
    uwb_to_point._robot_tag_S_pose_bias(robot_instance)
    
