#!/usr/bin/env python
import rospy
import time

# vision
from kuavo_robot_ros.srv import playmusic, playmusicRequest, playmusicResponse

# moveit 
from kuavo_robot_ros.srv import actionBowing, actionBowingRequest, actionBowingResponse
from kuavo_robot_ros.srv import actionFlower, actionFlowerRequest, actionFlowerResponse
from kuavo_robot_ros.srv import actionHello, actionHelloRequest, actionHelloResponse
from kuavo_robot_ros.srv import Tohome, TohomeRequest, TohomeResponse

# robot control
from kuavo_robot_ros.srv import changeArmCtrlMode, changeArmCtrlModeRequest, changeArmCtrlModeResponse
from kuavo_robot_ros.srv import srvChangePhases, srvChangePhasesRequest, srvChangePhasesResponse
from kuavo_robot_ros.srv import srvchangeCtlMode, srvchangeCtlModeRequest, srvchangeCtlModeResponse  # 0 位置 1 速度 2 步数
from kuavo_robot_ros.srv import srvClearPositionCMD, srvClearPositionCMDRequest, srvClearPositionCMDResponse
from kuavo_robot_ros.srv import changeAMBACCtrlMode, changeAMBACCtrlModeRequest, changeAMBACCtrlModeResponse
from kuavo_robot_ros.srv import srvClearPositionCMD, srvClearPositionCMDRequest, srvClearPositionCMDResponse
from kuavo_robot_ros.srv import srvChangeJoller, srvChangeJollerRequest, srvChangeJollerResponse

from kuavo_robot_ros.msg import walkCommand       
from sensor_msgs.msg import JointState
from kuavo_robot_ros.msg import robotQVTau         # 全身关节的位置、速度、力矩，前四个为躯干
from kuavo_robot_ros.msg import robotTorsoState    # 躯干的旋转角度、旋转速度、旋转加速度 / 质心的旋转角度、旋转速度、旋转加速度
from kuavo_robot_ros.msg import robotPhase         # 机器人目前的主状态，子状态

class kuavo:
    """ 构造机器人基础类 """

    def __init__(self, name):
        self._name = name
        
        # 手臂关节数(默认为14个)
        self.arm_num = 14 

        # 默认的手臂归中数据
        self.arm_recenter_joint_list = [20,0,0,-30,0,0,0,20,0,0,-30,0,0,0]

        # 成员变量用于保存数据
        self.latest_walk_speed = None
        self.latest_QVTau = None
        self.latest_Torso_centroid_State = None
        self.latest_Phase = None
        self.latest_subPhase = None

        # msg Pub 
        self._walk_speed_pub = rospy.Publisher("/walkCommand", walkCommand, queue_size=10)
        self._arm_traj_pub = rospy.Publisher("/kuavo_arm_traj", JointState, queue_size=10)

        # msg Sub
        self._robotQVTau_sub = rospy.Subscriber("/robot_q_v_tau", robotQVTau, self.get_robotQVTau_callback)
        self._robotTorsoState_pub = rospy.Subscriber("/robot_torso_state", robotTorsoState, self.get_robotTorsoState_callback)
        self._robotPhase_pub = rospy.Subscriber("/leju_robot_phase", robotPhase, self.get_robotPhase_callback)
        self._walk_speed_sub = rospy.Subscriber("/walkCommand", walkCommand, self.get_walk_speed_callback)

        # service client
        self._arm_ctrl_mode_client = rospy.ServiceProxy("/change_arm_ctrl_mode", changeArmCtrlMode)
        self._change_phases_client = rospy.ServiceProxy("/setPhase", srvChangePhases)
        self._robot_ctrl_mode_client = rospy.ServiceProxy("/change_ctl_mode", srvchangeCtlMode)
        self._robot_AMBAC_ctrl_mode_client = rospy.ServiceProxy("/change_AMBAC_ctrl_mode", changeAMBACCtrlMode)
        self._robot_clear_position_client = rospy.ServiceProxy("/clear_position_cmd", srvClearPositionCMD)
        self._robot_music_play_client = rospy.ServiceProxy("/play_music", playmusic)
        self._robot_change_joller_client = rospy.ServiceProxy("/change_joller_position", srvChangeJoller)

        # moveit client
        self._robot_moveit_action_Bowing = rospy.ServiceProxy("/kuavo_bowing_srv", actionBowing)
        self._robot_moveit_action_Helloo = rospy.ServiceProxy("/kuavo_hello_srv", actionHello)
        self._robot_moveit_action_Flower = rospy.ServiceProxy("/kuavo_flower_srv", actionFlower)
        self._robot_moveit_action_toHome = rospy.ServiceProxy("/kuavo_tohome_srv", Tohome)

    """ ROS函数 """
    def get_walk_speed_callback(self, msg):
        self.latest_walk_speed = msg

    def get_robotQVTau_callback(self, msg):
        self.latest_QVTau = msg
    
    def get_robotTorsoState_callback(self, msg):
        self.latest_Torso_centroid_State = msg

    def get_robotPhase_callback(self, msg):
        self.latest_mainPhase = msg.mainPhase
        self.latest_subPhase = msg.subPhase

    def pub_kuavo_walk_step_command(self, ctl_mode:int, step_num:float, x:float, y:float, yaw:float):
        walk_speed_msg = walkCommand()

        walk_speed_msg.mode = ctl_mode    
        walk_speed_msg.values = [step_num, x, y, yaw]  

        self._walk_speed_pub.publish(walk_speed_msg)

    def pub_kuavo_walk_speed(self, ctl_mode:int, x:float, y:float, yaw:float):
        walk_speed_msg = walkCommand()

        walk_speed_msg.mode = ctl_mode    
        walk_speed_msg.values = [x, y, yaw]  

        self._walk_speed_pub.publish(walk_speed_msg)

    def pub_kuavo_arm_traj(self, traj_jointstate):
        arm_traj_msg = JointState()

        arm_traj_msg.position = traj_jointstate.position
        self._arm_traj_pub.publish(arm_traj_msg)

    def srv_changeArmCtrlMode(self, control_mode:bool)->bool:
        try:
            request = changeArmCtrlModeRequest()
            request.control_mode = control_mode

            response = self._arm_ctrl_mode_client(request)

            return response.result

        except rospy.ServiceException as e:
            rospy.logerr(f"changeArmCtrlMode Service call failed: {e}")
            return False

    def srv_srvChangePhases(self, master_id:int, state_req:str, sub_state:str)->int:
        try:
            # 客户端发送request
            request = srvChangePhasesRequest()
            request.masterID = master_id
            request.stateReq = state_req
            request.subState = sub_state

            # 客户端接收response
            response = self._change_phases_client(request) 

            return response.stateRes

        except rospy.ServiceException as e:
            rospy.logerr(f"srvChangePhases Service call failed: {e}")
            return 0  
    
    def srv_playmusic_call(self, music_file:str, music_volume:int)->bool:
        try:
            # 客户端发送request
            request = playmusicRequest()
            request.music_number = music_file
            request.volume = music_volume

            # 客户端接收
            response = self._robot_music_play_client(request)
            
            return response.success_flag
        
        except rospy.ServiceException as e:
            rospy.logerr(f"playmusic_call Service call failed: {e}")
            return 0          
    
    def srv_actionBowing_call(self, _start_flag:bool)->bool:
        try:
            # 客户端发送request
            request = actionBowingRequest()
            request.start_flag = _start_flag

            # 客户端接收
            response = self._robot_moveit_action_Bowing(request)
            
            return response.end_flag
        
        except rospy.ServiceException as e:
            rospy.logerr(f"actionBowing_call Service call failed: {e}")
            return 0    
    
    def srv_actionHelloo_call(self, _start_flag:bool)->bool:
        try:
            # 客户端发送request
            request = actionHelloRequest()
            request.start_flag = _start_flag

            # 客户端接收
            response = self._robot_moveit_action_Helloo(request)
            
            return response.end_flag
        
        except rospy.ServiceException as e:
            rospy.logerr(f"actionHelloo_call Service call failed: {e}")
            return 0   

    def srv_actionTohome_call(self, _tag_id:int, _start_flag:bool)->bool:
        try:
            # 客户端发送request
            request = TohomeRequest()
            request.tag_id = _tag_id
            request.start_flag = _start_flag

            # 客户端接收
            response = self._robot_moveit_action_toHome(request)
            
            return response.end_flag
        
        except rospy.ServiceException as e:
            rospy.logerr(f"srv_actionTohome_call Service call failed: {e}")
            return 0   

    def srv_actionFlower_call(self, _tag_id:int, _start_flag:bool)->bool:
        try:
            # 客户端发送request
            request = actionFlowerRequest()
            request.tag_id = _tag_id
            request.start_flag = _start_flag

            # 客户端接收
            response = self._robot_moveit_action_Flower(request)
            
            return response.end_flag
        
        except rospy.ServiceException as e:
            rospy.logerr(f"actionFlower_call Service call failed: {e}")
            return 0   
    
    def srv_ChangeJoller_call(self, _left_pos:int, _right_pos:int)->bool:
        try:
            # 客户端发送request
            request = srvChangeJollerRequest()
            request.l_pos = _left_pos
            request.r_pos = _right_pos

            # 客户端接收
            response = self._robot_change_joller_client(request)
            
            return response.result
        
        except rospy.ServiceException as e:
            rospy.logerr(f"srv_ChangeJoller_call Service call failed: {e}")
            return 0   
    
    def srv_ChangectlMode(self, _master_id:int, _control_mode:int)->int:
        try:
            # 客户端发送request
            request = srvchangeCtlModeRequest()
            request.masterID = _master_id
            request.control_mode = _control_mode

            # 客户端接收response
            response = self._robot_ctrl_mode_client(request) 

            return response.control_mode

        except rospy.ServiceException as e:
            rospy.logerr(f"srv_ChangectlMode Service call failed: {e}")
            return 0          
    
    """ 功能函数 """
    def get_QVTau(self):
        """ 获取全身关节的位置、速度、力矩，前四个为躯干 """
        if self.latest_QVTau is not None:
            return self.latest_QVTau
        else:
            rospy.logwarn("No QVTau data available.")

    def get_Torso_centroid_State(self):
        """ 获取躯干/质心的旋转角度、旋转速度、旋转加速度 """
        if self.latest_Torso_centroid_State is not None:
            return self.latest_Torso_centroid_State
        else:
            rospy.logwarn("No Torso_centroid_State data available.")

    def get_Phase(self):
        """ 获取机器人目前的主状态，子状态 """
        if self.latest_Phase is not None:
            return self.latest_Phase
        else:
            rospy.logwarn("No Phase data available.")

    def get_walk_speedCommand(self):
        """ 获取机器人行走command """
        if self.latest_walk_speed is not None:
            return self.latest_walk_speed
        else:
            rospy.logwarn("No walk speed data available.")

    def set_arm_traj_position(self, joint_positions:list):
        """ 设置机器人行走状态时的速度
        :param joint_positions: list 最终关节的位置
        """
        if len(joint_positions) == self.arm_num:
            arm_traj_msg = JointState()
            arm_traj_msg.position = joint_positions

            self.pub_kuavo_arm_traj(arm_traj_msg)
        else:
            rospy.logerr("Invalid number of joint positions provided.")
    
    def set_walk_speed(self, control_mode:int, v_x:float, v_y:float, v_yaw:float, v_step=0):
        """ 设置机器人行走状态时的速度
        :param control_mode: int # mode: 0->PositionCommand | 1->VelocityCommand | 2->stepCommand
        :param v_x: float, 
        :param v_y: float, 
        :param v_yaw: float,
        """
        if control_mode == 1:
            # 速度模式控制
            print("....VelocityCommand  ", " v_x : ", v_x , " v_y : ", v_y ," v_yaw : ", v_yaw)
            self.pub_kuavo_walk_speed(control_mode, v_x, v_y, v_yaw)
        elif control_mode == 2:
            # 步态模式控制 
            print("....stepCommand v_step: ", v_step, " v_x : ", v_x , " v_y : ", v_y ," v_yaw : ", v_yaw)
            self.pub_kuavo_walk_step_command(control_mode, v_step, v_x, v_y, v_yaw)
        
    
    def set_robot_Phases(self, p_master_id:int, p_state_req:str, p_sub_state:str)->int:
        """ 设置机器人此时的模式 
        :param master_id: uint8, masterID
        :param state_req: string, stateReq
        :param sub_state: string, subState
        :return: int16, stateRes
        """
        result = self.srv_srvChangePhases(p_master_id, p_state_req, p_sub_state)
        rospy.loginfo(f"Service call /setPhase call: {result}")
        return result
    
    def set_robot_joller_position(self, l_position:int, r_position:int)->bool:
        """ 设置末端机械爪的 左右位置
        :param l_position: int32, 左边机械爪的位置 
        :param r_position: int32, 右边机械爪的位置    
        """
        result = self.srv_ChangeJoller_call(l_position, r_position)
        rospy.loginfo(f"Service call /change_joller_position call: {result}")
        return result
    
    def set_robot_arm_ctl_mode(self, p_control_mode:bool)->bool:
        """ 切换手臂规划模式 
        :param control_mode: bool, 控制模式
        :return: bool, 服务调用结果 
        """
        result = self.srv_changeArmCtrlMode(p_control_mode)
        rospy.loginfo(f"Service call /change_arm_ctrl_mode call: {result}")
        return result
    
    def set_robot_arm_recenter(self)->bool:
        """切换手臂为归中模式
        """
        try:
            self.set_arm_traj_position(self.arm_recenter_joint_list)
            rospy.loginfo("set_robot_arm_recenter success!")
        except Exception as e:
            print(f"An error occurred: {e}")
            rospy.loginfo("set_robot_arm_recenter fail!")

    def set_robot_status_ctl_mode(self, p_master_id:int, _control_mode:str)->bool:
        """切换机器人行走时的控制模式(在行走状态开始前进行设置)
        0 - Position 位置控制模式
        1 - Velocity 速度控制模式
        2 - Step     步数控制模式
        """
        # init control cmd
        if _control_mode == "Position":
            control_cmd = 0 
        elif _control_mode == "Velocity":
            control_cmd = 1
        elif _control_mode == "Step":
            control_cmd = 2
        
        try:
            self.srv_ChangectlMode(p_master_id, control_cmd)
        except Exception as e:
            print(f"An error occurred: {e}")

    def set_robot_play_music(self, music_file_name:str, music_volume:int)->bool:
        """机器人播放指定文件的音乐
        :param music_file_name, 音乐文件名字
        :param music_volume, 音乐音量
        :return: bool, 服务调用结果 
        """
        try:
            result = self.srv_playmusic_call(music_file_name, music_volume)
            
            rospy.loginfo(f"Service call /play_music call: {result}")
            
            return result
        
        except Exception as e:
            
            print(f"An error occurred: {e}")
            
            rospy.loginfo("Service /play_music call: fail!...please check again!")
            
            return False
    
    def set_action_play_bowing(self, _start_flag:bool)->bool:
        """机器人执行固定的动作 作揖
        :param _start_flag, 启动标志
        :return: bool, 服务调用结果 
        """
        try:
            result = self.srv_actionBowing_call(_start_flag)
            
            rospy.loginfo(f"Service call /kuavo_bowing_srv call: {result}")
            
            return result
        
        except Exception as e:
            
            print(f"An error occurred: {e}")
            
            rospy.loginfo("Service /kuavo_bowing_srv call: fail!...please check again!")
            
            return False

    def set_action_play_hello(self, _start_flag:bool)->bool:
        """机器人执行固定的动作 打招呼
        :param _start_flag, 启动标志
        :return: bool, 服务调用结果 
        """
        try:
            result = self.srv_actionHelloo_call(_start_flag)
            
            rospy.loginfo(f"Service call /kuavo_hello_srv call: {result}")
            
            return result
        
        except Exception as e:
            
            print(f"An error occurred: {e}")
            
            rospy.loginfo("Service /kuavo_hello_srv call: fail!...please check again!")
            
            return False

    def set_action_play_flower(self, _tag_id:int, _start_flag:bool)->bool:
        """机器人执行 抓花+闻花+送花的动作
        :param _tag_id, int : 要抓取的花的id
        :param _start_flag, bool : 启动标志
        :return: bool, 服务调用结果 
        """
        try:
            result = self.srv_actionFlower_call(_tag_id, _start_flag)
            
            rospy.loginfo(f"Service call /kuavo_flower_srv call: {result}")
            
            return result
        
        except Exception as e:
            
            print(f"An error occurred: {e}")
            
            rospy.loginfo("Service /kuavo_flower_srv  call: fail!...please check again!")
            
            return False

    def set_action_play_tohome(self, _tag_id:int, _start_flag:bool)->bool:
        """机器人执行 桌子规划 动作 回到初始位置 加入避障
        :param _tag_id, int : 要抓取的花的id
        :param _start_flag, bool : 启动标志
        :return: bool, 服务调用结果 
        """
        try:
            result = self.srv_actionTohome_call(_tag_id, _start_flag)
            
            rospy.loginfo(f"Service call /kuavo_tohome_srv call: {result}")
            
            return result
        
        except Exception as e:
            
            print(f"An error occurred: {e}")
            
            rospy.loginfo("Service /kuavo_tohome_srv  call: fail!...please check again!")
            
            return False