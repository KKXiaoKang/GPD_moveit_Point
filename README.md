# GPD_moveit_Point
a simple demo

* you can find how to use gpd in [ROS workspace to get start](https://blog.csdn.net/Eeko_x/article/details/104835154)
* 该仓库主要用于GPD生成抓取姿态检测的ROS流接口，支持转换为待抓取的物体的点云数据。

## install gpg 
```bash
sudo apt-get install libboost-all-dev

cd gpd
mkdir build && cd build
cmake ..
sudo make install

cd gpg
mkdir build && cd build
cmake ..
sudo make install
```
## OpenVINO 安装（ubuntu20.04）
```bash
# 获取密钥
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB

# 更新验证密钥
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB

# ubuntu20密钥
echo "deb https://apt.repos.intel.com/openvino/2024 ubuntu20 main" | sudo tee /etc/apt/sources.list.d/intel-openvino-2024.list

# 更新
sudo apt update

# 寻找包
apt-cache search openvino

# 更新安装
sudo apt install openvino-2024.1.0
```
## 更新scipy
```bash
pip install --upgrade scipy
```

## start
```bash
catkin_make
```

## run
```bash
# 启动带深度点云的分辨率realsense相机
roslaunch dynamic_biped sensor_robot_enable.launch

# 启动yolo目标检测节点(结果基于camera_link坐标系)
rosrun kuavo_vision_object realsense_yolo_segment_ros.py 

# (二选一，默认选mask)根据yolo目标检测的结果 获取待抓取物体的ROI点云（根据boundingbox 推理结果取待抓取的点云）
rosrun kuavo_yolo_point2d point_cloud_bounding_node.py
# (二选一，默认选mask)根据yolo目标检测的结果 获取待抓取物体的ROI点云（根据mask像素值范围 推理结果取待抓取的点云）
rosrun kuavo_yolo_point2d point_cloud_mask_node.py

# 启动GPD抓取姿态检测节点
roslaunch gpd tutorial1.launch

# 启动moveit规划器 发布全身关节TF以及进行后续规划
roslaunch kuavo40_moveit_config demo.launch

# 启动抓取姿态转换为实际抓取pose和orientation的节点（基于camera_link坐标系）
rosrun gpd_ros grasp_pose_converter.py

# 启动抓取姿态转换为torso坐标系的节点
rosrun gpd_ros grasp_transformer_torso.py
```