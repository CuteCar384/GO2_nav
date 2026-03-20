# Go2 ROS2 工作区

这个工作区当前已经实现 4 块能力：

- 建图
- 目标点导航
- 动态避障
- 前置相机图像发布

它不是只做建图的最小仓库了，而是一个围绕 Unitree Go2 搭起来的轻量 ROS 2 工作区。当前实现更适合现场调试、闭环验证和后续功能扩展。

## 当前能力概览

| 功能 | 包 | 主要输入 | 主要输出 | 说明 |
| --- | --- | --- | --- | --- |
| 建图 | `go2_mapping_minimal` | `/utlidar/cloud_deskewed` `/utlidar/robot_odom` | `/go2_built_map` `/go2_robot_path` `output/go2_built_map.pcd` | 在线累积去畸变点云，退出时保存二进制 PCD |
| 目标点导航 | `go2_navigation` | `/utlidar/robot_odom` `/goal_pose` | `/cmd_vel_nav` `/cmd_vel` `/api/sport/request` | 基于里程计的轻量闭环控制，不是完整 Nav2 |
| 动态避障 | `go2_navigation` | `/cmd_vel_nav` `/utlidar/range_info` `/utlidar/cloud_deskewed` `/utlidar/robot_odom` | `/cmd_vel` `/go2_navigation/obstacle_filter_status` `/go2_navigation/fused_range_info` | 在导航速度和运动执行之间做急停、减速、绕行动作偏置 |
| 图像发布 | `go2_camera_bridge` | Go2 前置相机 JPEG 流 | `/go2/front_camera/image_raw` `/go2/front_camera/image_raw/compressed` | 使用 `unitree_sdk2_python` 拉流，再桥接成标准 ROS 2 图像话题 |

## 目录结构

```text
~/xxx
├── output
│   └── go2_built_map.pcd
├── README.md
├── NAVIGATION_RESEARCH.md
├── 机器狗摄像头ROS2桥接简记.md
└── src
    ├── go2_camera_bridge
    ├── go2_mapping_minimal
    └── go2_navigation
```

## 环境依赖

基础环境：

- Ubuntu 22.04
- ROS 2 Jazzy
- 机器人与本机网络连通

建图相关依赖：

- `pcl_ros`
- `pcl_conversions`

相机桥接相关依赖：

- `cv_bridge`
- `python3-opencv`
- `python3-numpy`
- 本机存在 `unitree_sdk2_python`

运动控制相关依赖：

- `unitree_api` 已经在 ROS 2 环境里可见

建议先确认现场能看到这些关键话题：

- `/utlidar/cloud_deskewed`
- `/utlidar/robot_odom`
- `/utlidar/range_info`
- `/api/sport/request`

如果还没装基础依赖，可以先执行：

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-pcl-ros \
  ros-jazzy-pcl-conversions \
  ros-jazzy-cv-bridge \
  python3-opencv \
  python3-numpy
```

如果 `unitree_api` 不在当前环境里，需要先 `source` 你自己的 Unitree ROS 2 工作区。

## 编译

```bash
cd ~/xxx
source /opt/ros/jazzy/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --symlink-install
```

如果你的 Unitree ROS 2 工作区不在 `~/unitree_ros2/cyclonedds_ws/install`，把上面的路径替换成你自己的即可。

## 1. 建图

启动：

```bash
cd ~/xxx
source /opt/ros/jazzy/setup.bash
source ~/xxx/install/setup.bash
ros2 launch go2_mapping_minimal mapping_go2_builtin.launch.py
```

不带 RViz：

```bash
ros2 launch go2_mapping_minimal mapping_go2_builtin.launch.py rviz:=false
```

默认输入：

- `/utlidar/cloud_deskewed`
- `/utlidar/robot_odom`

运行时发布：

- `/go2_built_map`
- `/go2_robot_path`

退出时保存：

```text
~/xxx/output/go2_built_map.pcd
```

常用参数：

```bash
ros2 launch go2_mapping_minimal mapping_go2_builtin.launch.py \
  save_path:=/home/huang/xxx/output/my_map.pcd
```

```bash
ros2 launch go2_mapping_minimal mapping_go2_builtin.launch.py \
  cloud_topic:=/utlidar/cloud_deskewed \
  odom_topic:=/utlidar/robot_odom
```

默认行为：

- 每 5 帧点云发布一次在线累积地图
- 每 10 帧点云做一次体素下采样
- 默认体素大小 `0.10 m`
- 收到 `Ctrl+C` 或节点退出时自动保存 PCD

## 2. 目标点导航

当前导航链路由 3 个节点组成：

- `simple_goal_controller`：用里程计做目标点闭环控制
- `dynamic_obstacle_filter`：对导航速度做动态障碍过滤
- `unitree_sport_bridge`：把 `Twist` 转成 Unitree Sport API 请求

这条链路是“轻量目标点导航”，不是完整 Nav2 全局规划/局部规划栈。

### 2.1 直接发目标点导航

启动：

```bash
cd ~/xxx
source /opt/ros/jazzy/setup.bash
source ~/xxx/install/setup.bash
ros2 launch go2_navigation simple_goal_nav.launch.py
```

默认话题流向：

```text
/goal_pose -> simple_goal_controller -> /cmd_vel_nav
                                        |
                                        v
                    dynamic_obstacle_filter -> /cmd_vel
                                                 |
                                                 v
                                  unitree_sport_bridge -> /api/sport/request
```

如果你想从命令行发一个目标点，可以这样测：

```bash
cd ~/xxx
source /opt/ros/jazzy/setup.bash
source ~/xxx/install/setup.bash
ros2 run go2_navigation publish_goal --topic /goal_pose --frame-id odom --x 1.0 --y 0.0 --yaw 0.0
```

说明：

- `simple_goal_nav.launch.py` 默认订阅目标话题是 `/goal_pose`
- `publish_goal` 工具自身默认话题是 `/go2_navigation/goal`
- 所以命令行测试时建议显式加上 `--topic /goal_pose`
- 目标点的 `frame_id` 需要和里程计帧一致，默认按 `odom` 使用

### 2.2 加载保存地图并在 RViz 点击目标导航

这个模式会把保存好的 PCD 地图发布出来，方便在 RViz 里可视化和点目标。

启动：

```bash
cd ~/xxx
source /opt/ros/jazzy/setup.bash
source ~/xxx/install/setup.bash
ros2 launch go2_navigation click_goal_nav.launch.py
```

常用参数：

```bash
ros2 launch go2_navigation click_goal_nav.launch.py \
  map_pcd:=/home/huang/xxx/output/go2_built_map.pcd
```

```bash
ros2 launch go2_navigation click_goal_nav.launch.py \
  rviz:=false
```

这个模式额外发布：

- `/go2_saved_map`

注意：

- 这里的保存地图主要用于 RViz 显示和点击目标参考
- 当前并没有做基于保存地图的定位闭环
- 控制仍然主要依赖 `/utlidar/robot_odom`

## 3. 动态避障

动态避障默认已经接在两个导航 launch 里，不需要单独插线。

传感器输入：

- `/utlidar/range_info`
- `/utlidar/cloud_deskewed`
- `/utlidar/robot_odom`

默认输出：

- `/cmd_vel`
- `/go2_navigation/obstacle_filter_status`
- `/go2_navigation/fused_range_info`

默认策略包括：

- 前向距离过近时急停
- 接近障碍时按距离比例减速
- 根据左右净空差异选择偏置方向
- 传感器或里程计超时后触发 fail-safe 停车

主要参数在：

```text
src/go2_navigation/config/go2_navigation.yaml
```

比较常用的一组阈值：

- `front_stop_distance: 0.40`
- `front_avoid_distance: 0.75`
- `front_slow_distance: 1.20`
- `side_stop_distance: 0.18`
- `side_slow_distance: 0.32`

如果你要调避障效果，优先看 `go2_dynamic_obstacle_filter` 这一组参数。

## 4. 前置相机图像发布

由于直接订阅某些 Unitree 视频消息时 ROS 2 侧反序列化不稳定，这里改成了通过 `unitree_sdk2_python` 主动拉取前置相机 JPEG，再桥接成标准图像话题。

启动：

```bash
cd ~/xxx
source /opt/ros/jazzy/setup.bash
source ~/xxx/install/setup.bash
ros2 launch go2_camera_bridge front_camera_bridge.launch.py
```

如果要手动指定网卡：

```bash
ros2 launch go2_camera_bridge front_camera_bridge.launch.py \
  network_interface:=enx00e04c123598
```

如果想调采样帧率：

```bash
ros2 launch go2_camera_bridge front_camera_bridge.launch.py \
  fps:=15.0
```

默认发布：

- `/go2/front_camera/image_raw`
- `/go2/front_camera/image_raw/compressed`

默认参数文件：

```text
src/go2_camera_bridge/config/go2_camera_bridge.yaml
```

其中需要特别确认：

- `sdk_path` 默认是 `/home/huang/unitree_sdk2_python`
- `network_interface` 为空时会尝试自动探测连接机器狗的网卡

补充说明：

- 如果系统里没有 `cv_bridge`，节点会自动关闭原始图像发布，仅保留压缩图像
- 现场调试通常优先关注 `/go2/front_camera/image_raw/compressed`

## 常用文件

- 导航调研与边界说明：`NAVIGATION_RESEARCH.md`
- 相机桥接简记：`机器狗摄像头ROS2桥接简记.md`
- 导航参数：`src/go2_navigation/config/go2_navigation.yaml`
- 相机参数：`src/go2_camera_bridge/config/go2_camera_bridge.yaml`

## 当前边界

- 建图是在线点云累积建图，不是 Point-LIO 算法本体
- 导航是基于里程计的目标点闭环，不是完整 Nav2 栈
- 动态避障是速度过滤层，不是全局路径规划器
- 保存地图当前主要用于显示和目标点参考，不等同于完整定位地图

## 已知说明

- 如果终端出现大量 `Failed to parse type hash`，通常是 Unitree DDS 话题和 ROS 2 工具混用时的兼容性噪声，未必影响当前链路
- 如果导航节点启动正常但机器人不动，优先检查 `/api/sport/request` 是否可见、`unitree_api` 是否已正确 `source`
- 如果相机桥接无图像，优先检查 `sdk_path`、网卡选择，以及 `unitree_sdk2_python` 是否可正常拉流
# GO2_nav
利用GO2内部已有topic进行建图、导航
