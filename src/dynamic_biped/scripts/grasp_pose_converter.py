#!/usr/bin/env python
"""
    brief: 监听/detect_grasps/clustered_grasps的四个最佳抓取姿态，并转换为PoseArray消息发布到/grasp_poses话题
    将四个最佳抓取姿态转换为PoseArray，里面包含了position和orientation:
"""
import rospy
from gpd.msg import GraspConfigList
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
import tf2_ros

def vector_to_array(vector):
    return np.array([vector.x, vector.y, vector.z])

def grasp_callback(grasp_msg):
    poses = PoseArray()
    poses.header = grasp_msg.header

    # 定义从物体坐标系到世界坐标系的旋转矩阵
    R_obj_to_world = np.array([[0, 0, 1], 
                               [1, 0, 0], 
                               [0, 1, 0]])

    for i, grasp in enumerate(grasp_msg.grasps):
        approach = vector_to_array(grasp.approach)
        binormal = vector_to_array(grasp.binormal)
        axis = vector_to_array(grasp.axis)

        # 构建旋转矩阵
        rotation_matrix = np.vstack([approach, binormal, axis]).T

        # 转换为世界坐标系的旋转矩阵
        rotation_matrix_world = R_obj_to_world @ rotation_matrix

        # 转换为四元数
        rotation = R.from_matrix(rotation_matrix_world)
        quaternion = rotation.as_quat()

        # 创建Pose消息
        pose = Pose()
        pose.position.x = grasp.surface.z
        pose.position.y = grasp.surface.y
        pose.position.z = grasp.surface.x
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        # 添加到PoseArray
        poses.poses.append(pose)

        # 创建并发布TransformStamped
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = 'camera_link'  # 假设抓取姿态是相对于"world"坐标系的，可以根据实际情况修改
        transform.child_frame_id = f'grasp_{i}'  # 每个抓取姿态都有唯一的frame_id
        transform.transform.translation.x = grasp.surface.z
        transform.transform.translation.y = grasp.surface.y
        transform.transform.translation.z = grasp.surface.x
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        # 发布变换
        tf_broadcaster.sendTransform(transform)

    # 发布PoseArray
    pub.publish(poses)

if __name__ == '__main__':
    rospy.init_node('grasp_pose_converter', anonymous=True)
    
    # 订阅/detect_grasps/clustered_grasps
    rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, grasp_callback)
    
    # 发布新的话题，使用PoseArray来表示多个抓取姿态
    pub = rospy.Publisher('/grasp_poses', PoseArray, queue_size=10)
    
    # 初始化tf2的TransformBroadcaster
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    rospy.spin()