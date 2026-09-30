# Task3

## 2. 系统环境与依赖
- **操作系统**: Ubuntu 22.04 LTS
- **ROS 版本**: ROS 2 Humble
- **硬件设备**: 海康机器人工业相机（型号: MV-CS016-10UC，USB3.0接口）
- **海康 MVS SDK**: 版本 5.1.0

### 关于 MVS SDK 的安装说明
MVS SDK 属于厂商提供的第三方闭源 SDK，**无法通过 `rosdep` 自动安装**。请按以下步骤手动配置：
1. 前往[海康机器人官网下载中心](https://www.hikrobotics.com/cn/machinevision/service/download)下载 Linux x86_64 版本的 MVS SDK（本项目测试版本为 5.1.0）。
2. 解压并将 SDK 安装（或移动到）标准目录 `/opt/MVS` 下。
3. 配置动态链接库路径（写入 `~/.zshrc` 或 `/etc/ld.so.conf.d/mvs.conf`）：
   ```bash
   export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/MVS/lib/64

## 编译与运行
```bash
cd ~/ros2_ws
colcon build --packages-select hik_camera_ros2
source install/stdup.zsh

ros2 run hik_camera_ros2 hik_camera_node
ros launch hik_camera_ros2 hik_camera.launch.py
```

## 发布的话题

| 话题名 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/image_raw` | `sensor_msgs/msg/Image` | 相机采集的原始图像数据 |

## ROS 2 参数

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `topic_name` | string | `/image_raw` | 图像发布的话题名称 |
| `exposure_time` | double | `10000.0` | 曝光时间（单位：微秒） |
| `gain` | double | `10.0` | 增益 |

## 关于可视化工具无法启动的说明

本项目在测试过程中，发现运行 `rviz2` 或 `rqt_image_view` 时会导致程序崩溃，报错信息通常为 `Cannot mix incompatible Qt library (5.15.10) with this library (5.15.3)` 或提示与 Wayland 环境冲突。

### 替代验证方案
虽然图形化界面无法使用，但这并不影响我们验证相机节点的工作状态。本项目使用了以下命令行工具来替代 RViz2 完成图像数据的验证：
1. **验证数据流是否正常**：`ros2 topic hz /image_raw`
   - 该命令将输出稳定的平均帧率，证明图像数据正在持续发布。
2. **验证数据内容是否有效**：`ros2 topic echo /image_raw --once`
   - 该命令会打印出一帧完整的图像字节数据，证明话题内有真实的图像内容。
3. **动态参数验证**：`ros2 param set /hik_camera exposure_time 20000.0`
   - 证明参数修改能正确同步到相机硬件。

