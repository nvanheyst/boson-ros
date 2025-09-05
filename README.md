# Boson ROS 2 Camera Node (ROS2 Jazzy)

This package provides a ROS 2 node (`boson_node`) for interfacing with a **FLIR Boson** thermal camera using OpenCV and `cv_bridge`. The node captures and publishes thermal images in either **raw Y16** or **RGB** formats.

---

To start with default settings:  
```bash
ros2 launch boson_ros2 boson_camera.launch.py
```

To enable **raw (Y16) mode**:  
```bash
ros2 launch boson_ros2 boson_camera.launch.py raw_video:=true
```


### Installation

mkdir -p ~/boson_ws/src
cd ~/boson_ws/src
git clone -b ros2-jazzy-devel https://github.com/nvanheyst/boson-ros.git
cd ..
colcon build --symlink-install


echo "source ~/boson_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

```bash
cd boson_ws/src/boson-ros/boson_bringup/config/
sudo cp ./99-flir-boson.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```
