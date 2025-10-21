# 海康威视工业相机ROS2驱动
---
- 该项目基于 sjtu-cyberc3/hikvision_ros2_driver。
- 该项目包含了使用 MIT License 和 Apache License 2.0 的部分。
- 保留所有原始的版权声明。
---
- 运行MVS可视化工具，配置相机参数。设置相机名称(user_name, device_id)，本项目将根据相机名称寻找对应设备。

- 用** /opt/MVS/bin/MVS.sh **来打开MVS的客户端

-----------------------------------------------------------------
//source /opt/ros/jazzy/setup.zsh
//cd ~/ros2_ws_task/hikvision_ros2_driver-main
-----------------------------------------------------------------
source install/setup.zsh
ros2 launch hikvision_ros2_driver camera_with_rviz.launch.py camera_name:=my_camera
-----------------------------------------------------------------
# 切换到工作空间根目录
cd ~/ros2_ws_task/hikvision_ros2_driver-main

# 清理可能污染的变量
unset AMENT_CURRENT_PREFIX
unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset ROS_PACKAGE_PATH
export AMENT_PREFIX_PATH=""
export COLCON_PREFIX_PATH=""
export ROS_PACKAGE_PATH=""

# 使用 bash 执行 setup.bash 并将环境变量导入当前 zsh 会话
eval "$(bash -c 'source /opt/ros/jazzy/setup.bash; env | grep -E "^AMENT_PREFIX_PATH=|^COLCON_PREFIX_PATH=|^ROS_PACKAGE_PATH=|^PATH=|^PYTHONPATH=|^LD_LIBRARY_PATH=|^CMAKE_PREFIX_PATH="')"

# 现在执行构建
- colcon build --symlink-install
- 或
- colcon build --packages-select hikvision_interface hikvision_ros2_driver --symlink-install
- 若已经构建过，可以跳过或rm -rf build install log
- rm -rf build/ install/ log/
# 独立节点（standalone）运行方式：

```zsh
ros2 launch hikvision_ros2_driver standalone.launch.yaml camera_name:=<camera_name>
```

## 进程内通信（component）运行方式：

```zsh
ros2 launch hikvision_ros2_driver component.launch.yaml camera_name:=<camera_name> container_name:=<container_name>
```
- source install/setup.zsh
## 启动所有节点与Rviz2:
```zsh
ros2 launch hikvision_ros2_driver camera_with_rviz.launch.py camera_name:=my_camera
```

说明与快速启动
- 如果你传入 `camera_name`（如 `my_camera`）且该名字存在，驱动将尝试打开对应设备。
- 如果 `camera_name` 留空或名字不匹配，驱动会自动选择第一个检测到的设备并尝试打开（默认行为，避免启动报错）。
- 在没有真实相机时，可用 `simulate` 参数启动仿真发布：
```
ros2 launch hikvision_ros2_driver camera_with_rviz.launch.py simulate:=true
```

推荐一行启动流程（新终端中执行）：
```
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws_task/hikvision_ros2_driver-main
source install/setup.zsh
ros2 launch hikvision_ros2_driver camera_with_rviz.launch.py camera_name:=my_camera
```
# 操作：
- rm -rf build/ install/ log/
- # 清理并加载系统环境
unset AMENT_CURRENT_PREFIX; unset AMENT_PREFIX_PATH; unset COLCON_PREFIX_PATH; unset ROS_PACKAGE_PATH; export AMENT_PREFIX_PATH=""; export COLCON_PREFIX_PATH=""; export ROS_PACKAGE_PATH=""; eval "$(bash -c 'source /opt/ros/jazzy/setup.bash; env | grep -E "^AMENT_PREFIX_PATH=|^COLCON_PREFIX_PATH=|^ROS_PACKAGE_PATH=|^PATH=|^PYTHONPATH=|^LD_LIBRARY_PATH=|^CMAKE_PREFIX_PATH="')"

# 编译
colcon build --packages-select hikvision_interface hikvision_ros2_driver --symlink-install
- source install/setup.zsh
- ros2 launch hikvision_ros2_driver camera_with_rviz.launch.py camera_name:=my_camera

一键启动（相机+C++预处理+Python推理+RViz，仅三个可视化话题）
- 现在 camera_with_rviz.launch.py 同时启动相机驱动、推理节点和 RViz，并默认加载一个 RViz 配置，显示下面三路图像：
	- /image_raw（原图）
	- /armor_preprocessed_cpp（黑白掩码+BGR显示+绿色旋转框，仅用于可视化）
	- /armor_crop_result（裁剪后处理+模型识别结果，小字在左上角）

可选参数：
- model_path：指定 PyTorch 权重文件路径（默认脚本会找项目根目录下 best_digit_recognizer_model.pth）
	例：
	```zsh
	ros2 launch hikvision_ros2_driver camera_with_rviz.launch.py \
		camera_name:=my_camera \
		model_path:=/home/daizhijun/RobotMaster/hikvision_ros2_integration/best_digit_recognizer_model.pth
	```

依赖说明（运行推理需要）：
- Python 端需要安装 torch、torchvision、opencv-python、numpy。

验证：
- 启动后在 RViz 左侧 Displays 面板可见 3 个 Image 显示项：image_raw / armor_preprocessed_cpp / armor_crop_result。
- 命令行也可用：
	- `ros2 topic echo /image_raw`（数据较大，建议在 RViz 中查看）
	- `ros2 topic echo /armor_preprocessed_cpp`
	- `ros2 topic echo /armor_crop_result`


- 需要将camera_name替换成对应相机的名称。

- 对于进程内通信，需要提前启动`component_container`，并将container_name设置为对应container的名称。

---

发布话题：

- raw/image [sensor_msgs/msg/Image]: 相机原始数据，格式可以在MVS工具中修改，默认情况下应该是bayer格式。
- info [hikvision_interface/msg/HikImageInfo]: 图像元数据，包括时间戳、曝光长度、增益大小、白平衡参数等。

---

## 不建议对bayer格式的原始数据使用jpeg压缩。即当相机原始数据格式为bayer时，不建议订阅`raw/image/compressed`话题。

## rosbag录制时建议开启zstd压缩以节省空间。(ros2 bag record -s mcap --storage-preset-profile zstd_fast)

