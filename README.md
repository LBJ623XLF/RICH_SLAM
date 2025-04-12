# RICH_SLAM

## 1. Lidar

### 1.1 Install

```bash
git clone https://github.com/RoboSense-LiDAR/rslidar_sdk.git 
cd rslidar_sdk 
git submodule init 
git submodule update


sudo apt-get update 
sudo apt-get install -y libyaml-cpp-dev

sudo apt-get install -y  libpcap-dev

cd rslidar_sdk 
mkdir build && cd build 
cmake .. && make -j4 
./rslidar_sdk_node

catkin_make 
source devel/setup.bash 
roslaunch rslidar_sdk start.launch
```

### 1.2 Run

```bash
roscore 
rosbag record /topic  
./rslidar_sdk_node 
rviz 
rosbag play filename.bag 
rviz 
```

## 2. LIO-SAM

### 2.1 Install

```bash
sudo apt-get install -y ros-noetic-navigation --allow-unauthenticated 
sudo apt-get install -y ros-noetic-robot-localization 
sudo apt-get install -y ros-noetic-robot-state-publisher 

wget -0 ~/ros1_slam_3d_ws/gtsam.zip \ 
https://github.com/borglab/gtsam/archive/4.0.2.zip  
cd ~/ros1_slam_3d_ws/ && unzip gtsam.zip -d ~/ros1_slam_3d_ws/ 
cd gtsam-4.0.2 
mkdir build && cd build 
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \ 
-DGTSAM_USE_SYSTEM_EIGEN=ON .. 
sudo make install -j4 

cd /usr/local/lib/ 
sudo mv libmetis-gtsam.so /opt/ros/noetic/lib/ 

cd ~/ros1_slam_3d_ws/src 
git clone https://github.com/TixiaoShan/LIO-SAM.git 
cd lio-sam 
cd .. 
catkin_make 

cd ~/ros1_slam_3d_ws 
gedit src/LIO-SAM/config/params.yaml 
source devel/setup.bash 
roscore 
roslaunch lio_sam run.launch  
rosbag play casual_walk.bag 
```
### 2.2 Run

```bash
cd wit/wit_ros_ws 
sudo chmod 777 /dev/ttyUSB0 
source devel/setup.bash 
roslaunch wit_ros_imu rviz_and_imu.launch 


cd ~/rslidar_ws 
source devel/setup.bash 
roslaunch rslidar_sdk start.launch 
rosbag record -O 701_v.bag /wit/imu /velodyne_points  

cd ~/catkin_ws 
source devel/setup.bash 
roslaunch lidar_imu_calib calib_exR_lidar2imu.launch 

cd ~/ros1_slam_3d_ws 
source devel/setup.bash 
roslaunch lio_sam run.launch 

rosbag play 701_v.bag /imu/data:=/imu_raw /velodyne_points:=/points_raw 
```
