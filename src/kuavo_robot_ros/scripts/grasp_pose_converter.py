#!/usr/bin/env python
"""
    brief: 监听/detect_grasps/clustered_grasps的四个最佳抓取姿态，并转换为PoseArray消息发布到/grasp_poses话题
    将四个最佳抓取姿态转换为PoseArray，里面包含了position和orientation:
"""
import rospy
from gpd.msg import GraspConfigList
from geometry_msgs.msg import PoseArray, Pose
from scipy.spatial.transform import Rotation as R
import numpy as np

def vector_to_array(vector):
    return np.array([vector.x, vector.y, vector.z])

def grasp_callback(grasp_msg):
    poses = PoseArray()
    poses.header = grasp_msg.header

    for grasp in grasp_msg.grasps:
        approach = vector_to_array(grasp.approach)
        binormal = vector_to_array(grasp.binormal)
        axis = vector_to_array(grasp.axis)

        # 构建旋转矩阵
        rotation_matrix = np.vstack([approach, binormal, axis]).T

        # 转换为四元数
        rotation = R.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()

        # 创建Pose消息
        pose = Pose()
        pose.position.x = grasp.surface.x
        pose.position.y = grasp.surface.y
        pose.position.z = grasp.surface.z
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        # 添加到PoseArray
        poses.poses.append(pose)

    # 发布PoseArray
    pub.publish(poses)

if __name__ == '__main__':
    rospy.init_node('grasp_pose_converter', anonymous=True)
    
    # 订阅/detect_grasps/clustered_grasps
    rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, grasp_callback)
    
    # 发布新的话题，使用PoseArray来表示多个抓取姿态
    pub = rospy.Publisher('/grasp_poses', PoseArray, queue_size=10)
    
    rospy.spin()
