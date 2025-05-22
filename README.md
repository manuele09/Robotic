# Project Name

**Description:** A brief description of your project and its purpose.

## Table of Contents
- [Installation](#installation)
  - [ROS Melodic Installation](#ros-melodic-installation)
  - [Dependencies Installation](#dependencies-installation)
  - [Setting up LVI-SAM](#setting-up-lvi-sam)
  - [Setting up Velodyne](#setting-up-velodyne)
  - [Setting up USB Camera](#setting-up-usb-camera)
- [Usage](#usage)
  - [Launching Velodyne Node](#launching-velodyne-node)
  - [Launching USB Camera Node](#launching-usb-camera-node)
  - [Launching LVI-SAM Node](#launching-lvi-sam-node)

---

## Installation

### ROS Melodic Installation

Install ROS Melodic on Ubuntu with these commands:

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-melodic-desktop-full

echo "source /opt/ros/melodic/setup.bash" >> ~/bashrc
source ~/bashrc

sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo rosdep init
rosdep update
```

### Dependencies Installation

#### GTSAM (Georgia Tech Smoothing and Mapping library)
```bash
sudo add-apt-repository ppa:borglab/gtsam-release-4.0
sudo apt update
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

#### Ceres Solver (Optimization library)
```bash
sudo apt-get install -y libgoogle-glog-dev libatlas-base-dev
wget -O ~/Downloads/ceres.zip https://github.com/ceres-solver/ceres-solver/archive/1.14.0.zip
cd ~/Downloads/ && unzip ceres.zip -d ~/Downloads/
cd ~/Downloads/ceres-solver-1.14.0
mkdir ceres-bin && cd ceres-bin
cmake ..
sudo make install -j4
```

### Setting up LVI-SAM

After installing dependencies, set up LVI-SAM with these commands:

```bash
mkdir -p ~/LVI-SAM/src
cd ~/LVI-SAM/src
git clone https://github.com/TixiaoShan/LVI-SAM.git
cd ..
catkin_make -j4
```

### Setting up Velodyne

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

### Setting up USB Camera

1. Install the USB camera package:
   ```bash
   sudo apt install ros-melodic-usb-cam
   ```

2. Check available video interfaces:
   ```bash
   ls /dev/video*
   ```

3. Get camera information (replace `/dev/video2` with your device):
   ```bash
   v4l2-ctl --device=/dev/video2 --all
   ```

4. Modify the launch file:
   ```bash
   sudo gedit /opt/ros/melodic/share/usb_cam/launch/usb_cam-test.launch
   ```

   The default launch file contains:
   ```xml
   <launch>
     <node name="usb_cam" pkg="usb_cam" type="usb_cam_node" output="screen" >
       <param name="video_device" value="/dev/video0" />
       <param name="image_width" value="640" />
       <param name="image_height" value="480" />
       <param name="pixel_format" value="yuyv" />
       <param name="color_format" value="yuv422p" />
       <param name="camera_frame_id" value="usb_cam" />
       <param name="io_method" value="mmap"/>
     </node>
     <node name="image_view" pkg="image_view" type="image_view" respawn="false" output="screen">
       <remap from="image" to="/usb_cam/image_raw"/>
       <param name="autosize" value="true" />
     </node>
   </launch>
   ```

   Modify these parameters according to your camera specifications:
   - `video_device`
   - `image_width`
   - `image_height` 
   - `pixel_format`
   - `color_format`

   For format conventions and additional parameters, see:
   - [ROS USB_Cam Pixel Formats Reference](http://wiki.ros.org/usb_cam#Pixel_formats.2Fencodings_reference)
   - [USB_Cam Node Parameters Documentation](http://wiki.ros.org/usb_cam/Old%20Versions)

## Usage

### Launching Velodyne Node
```bash
cd ~/velodyne
source devel/setup.bash
roslaunch velodyne_pointcloud VLP16_points.launch
```

### Launching USB Camera Node
```bash
roslaunch usb_cam usb_cam-test.launch
```

### Launching LVI-SAM Node

1. First, download one of the sample datasets (.bag files) from:
   - [LVI-SAM Google Drive Datasets](https://drive.google.com/drive/folders/1q2NZnsgNmezFemoxhHnrDnp1JV_bqrgV?usp=sharing)

2. In one terminal, navigate to your LVI-SAM workspace and launch the node:
   ```bash
   cd ~/LVI-SAM
   source devel/setup.bash
   roslaunch lvi_sam run.launch
   ```

3. In another terminal, play the downloaded dataset:
   ```bash
   rosbag play path/to/dataset.bag
   ```
