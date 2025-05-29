# Project Name

**Description:** A brief description of your project and its purpose.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [ROS Melodic](#ros-melodic)
  - [Core Dependencies](#core-dependencies)
  - [Sensor Packages](#sensor-packages)
    - [Velodyne Lidar](#velodyne-lidar)
    - [USB Camera](#usb-camera)
    - [IMU Setup](#imu-setup)
  - [LVI-SAM Framework](#lvi-sam-framework)
- [Usage](#usage)
  - [Running Sensor Nodes](#running-sensor-nodes)
    - [Velodyne Lidar](#velodyne-lidar-1)
    - [USB Camera](#usb-camera-1)
    - [IMU](#imu)
  - [Running LVI-SAM](#running-lvi-sam)
  - [Running with Sample Data](#running-with-sample-data)

---

## Prerequisites
- Ubuntu 18.04 (for ROS Melodic compatibility)
- Supported hardware:
  - Velodyne VLP-16 Lidar

## Installation

### ROS Melodic
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
```

### Core Dependencies

#### GTSAM (Georgia Tech Smoothing and Mapping library)
```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

#### Ceres Solver
```bash
sudo apt-get install -y libgoogle-glog-dev libatlas-base-dev
wget -O ~/Downloads/ceres.zip https://github.com/ceres-solver/ceres-solver/archive/1.14.0.zip
cd ~/Downloads/ && unzip ceres.zip -d ~/Downloads/
cd ~/Downloads/ceres-solver-1.14.0
mkdir ceres-bin && cd ceres-bin
cmake ..
sudo make install -j4
```

### Sensor Packages

#### Velodyne Lidar
```bash
mkdir -p ~/velodyne/src
cd ~/velodyne/src
git clone -b melodic-devel https://github.com/ros-drivers/velodyne.git
cd ..
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
catkin_make -j2
```

#### USB Camera
```bash
sudo apt install ros-melodic-usb-cam
```

#### IMU Setup
```bash
mkdir -p ~/imu_ws/src
cd ~/imu_ws/src
wget https://www.syd-dynamics.com/download/transducerm_example_ros-pkg
unzip transducerm_example_ros-pkg
cd ~/imu_ws
catkin_make
```

### LVI-SAM Framework
```bash
mkdir -p ~/LVI-SAM/src
cd ~/LVI-SAM/src
git clone https://github.com/TixiaoShan/LVI-SAM.git
cd ..
catkin_make -j4
```

## Usage

### Running Sensor Nodes

#### Velodyne Lidar
1. Configure network:
   ```bash
   sudo ifconfig eth0 10.0.1.1
   sudo route add 10.0.1.7 eth0
   ```

2. Launch node:
   ```bash
   cd ~/velodyne
   source devel/setup.bash
   roslaunch velodyne_pointcloud VLP16_points.launch
   ```

#### USB Camera
1. Identify your camera:
   ```bash
   ls /dev/video*
   v4l2-ctl --device=/dev/video0 --all
   ```

2. Modify launch file (example values shown):
   ```bash
   sudo nano /opt/ros/melodic/share/usb_cam/launch/usb_cam-test.launch
   ```
   ```xml
   <param name="video_device" value="/dev/video0" />
   <param name="image_width" value="640" />
   <param name="image_height" value="480" />
   <param name="pixel_format" value="yuyv" />
   ```

3. Launch node:
   ```bash
   roslaunch usb_cam usb_cam-test.launch
   ```

#### IMU
1. Configure serial port in `TMSerial.cpp`:
   ```cpp
   std::string imu_port = "/dev/ttyUSB0";
   int baudrate = 115200;
   ```

2. Launch node:
   ```bash
   cd ~/imu_ws
   source devel/setup.bash
   rosrun TransducerM_pkg TMSerial
   ```

### Running LVI-SAM
```bash
cd ~/LVI-SAM
source devel/setup.bash
roslaunch lvi_sam run.launch
```

### Running with Sample Data
1. Download sample dataset from:
   [LVI-SAM Google Drive](https://drive.google.com/drive/folders/1q2NZnsgNmezFemoxhHnrDnp1JV_bqrgV)

2. Play the bag file:
   ```bash
   rosbag play path/to/dataset.bag
   ```
