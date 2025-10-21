# 海康威视工业相机ROS2驱动

支持列表：

* USB相机

---

依赖项安装：

1. 在官网下载并安装MVS驱动包。

2. 运行MVS可视化工具，配置相机参数。

---

运行all:
```zsh
ros2 launch hikvision_ros2_driver camera_with_rviz.launch.py camera_name:=my_camera
```

