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


