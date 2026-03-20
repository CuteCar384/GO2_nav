# 机器狗摄像头 ROS2 桥接简记

## 改动说明

- 已确认机器狗侧存在原生话题：`/frontvideostream`
- 由于直接按 `unitree_go/msg/Go2FrontVideoData` 订阅时反序列化不稳定，本次改为使用主目录下的 `unitree_sdk2_python` 拉取前置相机图像
- 已将摄像头桥接从 `go2_navigation` 拆分为独立功能包：`go2_camera_bridge`
- 独立包中的桥接节点为：`go2_front_camera_bridge`
- 桥接节点会将 Go2 前置相机流发布为标准 ROS2 图像话题

## 新增话题

- `/go2/front_camera/image_raw`
- `/go2/front_camera/image_raw/compressed`

## 主要文件

- `src/go2_camera_bridge/go2_camera_bridge/front_camera_bridge.py`
- `src/go2_camera_bridge/launch/front_camera_bridge.launch.py`
- `src/go2_camera_bridge/config/go2_camera_bridge.yaml`

## 运行方式

```bash
cd /home/huang/xxx
source /opt/ros/jazzy/setup.bash
source /home/huang/xxx/install/setup.bash
ros2 launch go2_camera_bridge front_camera_bridge.launch.py network_interface:=enx00e04c123598
```

## 验证结果

- `ros2 topic list` 可见 `/go2/front_camera/image_raw/compressed`
- 已实测收到压缩图像消息
- 已实测收到原始图像消息，格式为 `bgr8`，分辨率为 `1920x1080`

## 备注

- 终端中的 `Failed to parse type hash` 多为 Unitree DDS 与 ROS2 工具混用时的兼容性警告
- 当前桥接链路已可正常工作
