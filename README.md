# GPD_moveit_Point
a simple demo

* you can find how to use gpd in [ROS workspace to get start](https://blog.csdn.net/Eeko_x/article/details/104835154)

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

## start
```bash
catkin_make
```