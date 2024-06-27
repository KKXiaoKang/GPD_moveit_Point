#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tf_trans
import numpy as np

def pose_array_callback(msg):
    # 初始化 tf2 监听器和广播器
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    # 等待从 camera_link 到 torso 的变换
    try:
        transform = tf_buffer.lookup_transform('torso', 'camera_color_optical_frame', rospy.Time(), rospy.Duration(1.0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logerr(f"获取变换失败: {e}")
        return

    # 定义坐标系旋转，以将相机坐标系转换为机器人的世界坐标系
    # rot_quat = tf_trans.quaternion_from_euler(0, 0, 0)  # 根据实际夹爪的形态具体需求调整
    rot_quat = tf_trans.quaternion_from_euler(-1.5708, 0, -1.5708) # 正确的夹爪范围，

    # 转换 PoseArray 中的每个姿态从 camera_link 到 torso 坐标系
    transformed_poses = []
    count = 0 # 计数器
    for pose in msg.poses:
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = pose
        try:
            # 执行姿态转换
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

            # 应用额外旋转
            orig_quat = [
                transformed_pose.pose.orientation.x,
                transformed_pose.pose.orientation.y,
                transformed_pose.pose.orientation.z,
                transformed_pose.pose.orientation.w,
            ]
            new_quat = tf_trans.quaternion_multiply(rot_quat, orig_quat)
            transformed_pose.pose.orientation.x = new_quat[0]
            transformed_pose.pose.orientation.y = new_quat[1]
            transformed_pose.pose.orientation.z = new_quat[2]
            transformed_pose.pose.orientation.w = new_quat[3]

            transformed_poses.append(transformed_pose.pose)

            # 创建并发布 TransformStamped 到 TF 树中
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = 'torso'  # 假设抓取姿态是相对于"world"坐标系的，可以根据实际情况修改
            transform_stamped.child_frame_id = f'torso_grasp_{count}'  # 每个抓取姿态都有唯一的frame_id
            transform_stamped.transform.translation.x = transformed_pose.pose.position.x
            transform_stamped.transform.translation.y = transformed_pose.pose.position.y
            transform_stamped.transform.translation.z = transformed_pose.pose.position.z
            transform_stamped.transform.rotation.x = transformed_pose.pose.orientation.x
            transform_stamped.transform.rotation.y = transformed_pose.pose.orientation.y
            transform_stamped.transform.rotation.z = transformed_pose.pose.orientation.z
            transform_stamped.transform.rotation.w = transformed_pose.pose.orientation.w
            tf_broadcaster.sendTransform(transform_stamped)

        except (tf2_ros.TransformException, tf2_geometry_msgs.TransformException) as e:
            rospy.logerr(f"姿态转换失败: {e}")

        count += 1
    
    # 发布转换后的姿态作为 PoseArray 到 torso 坐标系
    transformed_pose_array = PoseArray()
    transformed_pose_array.header = msg.header
    transformed_pose_array.poses = transformed_poses
    pub.publish(transformed_pose_array)

if __name__ == '__main__':
    rospy.init_node('grasp_transformer_torso', anonymous=True)

    # 订阅 /camera_link/grasp_poses
    rospy.Subscriber('/camera_link/grasp_poses', PoseArray, pose_array_callback)

    # 发布器，发布转换后的姿态到 /torso/grasp_poses
    pub = rospy.Publisher('/torso/grasp_poses', PoseArray, queue_size=10)

    rospy.spin()
