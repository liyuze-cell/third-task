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

## 5.1 发布的话题

| 话题名 | 消息类型 | 说明 |
| :--- | :--- | :--- |
| `/image_raw` | `sensor_msgs/msg/Image` | 相机采集的原始图像数据 |

## 5.2 ROS 2 参数

| 参数名 | 类型 | 默认值 | 说明 |
| :--- | :--- | :--- | :--- |
| `topic_name` | string | `/image_raw` | 图像发布的话题名称 |
| `exposure_time` | double | `10000.0` | 曝光时间（单位：微秒） |
| `gain` | double | `10.0` | 增益 |