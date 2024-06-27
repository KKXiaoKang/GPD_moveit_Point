#!/usr/bin/env python

import rospy
from gpd.msg import GraspConfigList
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
import tf
import tf2_ros
import numpy as np
import tf.transformations as tf_trans

def grasp_config_to_pose(grasp_config):
    pose = Pose()
    
    # 设置位置
    pose.position.x = grasp_config.bottom.x 
    pose.position.y = grasp_config.bottom.y
    pose.position.z = grasp_config.bottom.z  # 反转 Z 轴方向
    
    # 计算方向（四元数）
    # 将抓取方向（approach）设为Z轴，法线（binormal）设为Y轴，轴线（axis）设为X轴
    approach = np.array([grasp_config.approach.x, grasp_config.approach.y, grasp_config.approach.z])
    binormal = np.array([grasp_config.binormal.x, grasp_config.binormal.y, grasp_config.binormal.z])
    axis = np.array([grasp_config.axis.x, grasp_config.axis.y, grasp_config.axis.z])

    # 创建一个4x4的变换矩阵
    rotation_matrix = np.eye(4)
    rotation_matrix[0:3, 0] = axis    # X轴
    rotation_matrix[0:3, 1] = binormal # Y轴
    rotation_matrix[0:3, 2] = approach # Z轴

    rospy.loginfo(f"Rotation Matrix: \n{rotation_matrix}")

    # 计算四元数
    quaternion = tf_trans.quaternion_from_matrix(rotation_matrix)
    quaternion = tf_trans.unit_vector(quaternion)
    
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]
    
    return pose

def callback(grasp_config_list):
    pose_array = PoseArray()
    pose_array.header = grasp_config_list.header
    
    for i, grasp_config in enumerate(grasp_config_list.grasps):
        pose = grasp_config_to_pose(grasp_config)
        pose_array.poses.append(pose)
        
        # 创建并发布TransformStamped
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = 'camera_color_optical_frame'
        transform_stamped.child_frame_id = f'grasp_{i}'
        transform_stamped.transform.translation.x = pose.position.x
        transform_stamped.transform.translation.y = pose.position.y
        transform_stamped.transform.translation.z = pose.position.z
        transform_stamped.transform.rotation.x = pose.orientation.x
        transform_stamped.transform.rotation.y = pose.orientation.y
        transform_stamped.transform.rotation.z = pose.orientation.z
        transform_stamped.transform.rotation.w = pose.orientation.w
        
        tf_broadcaster.sendTransform(transform_stamped)
    
    pose_pub.publish(pose_array)

if __name__ == '__main__':
    rospy.init_node('grasp_to_pose_converter')
    
    grasp_sub = rospy.Subscriber('/detect_grasps/clustered_grasps', GraspConfigList, callback)
    pose_pub = rospy.Publisher('/camera_link/grasp_poses', PoseArray, queue_size=10)
    
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    
    rospy.spin()
