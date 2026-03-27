# Go2 ROS2 工作区

这个工作区当前已经实现 4 块能力：

- 建图
- 基于 EGO-Planner 的局部导航避障
- 前置相机图像发布

它不是只做建图的最小仓库了，而是一个围绕 Unitree Go2 搭起来的轻量 ROS 2 工作区。当前实现更适合现场调试、闭环验证和后续功能扩展。

## 当前能力概览

| 功能 | 包 | 主要输入 | 主要输出 | 说明 |
| --- | --- | --- | --- | --- |
| 建图 | `go2_mapping_minimal` | `/utlidar/cloud_deskewed` `/utlidar/robot_odom` | `/go2_built_map` `/go2_robot_path` `output/go2_built_map.pcd` `output/go2_named_poses.json` | 在线累积去畸变点云，退出时保存二进制 PCD；建图过程中可记录命名地点 |
| 局部导航避障 | `ego_planner` `go2_navigation` | `/utlidar/robot_odom` `/utlidar/cloud_deskewed` `/pct_path` | `planning/bspline` `/cmd_vel` `/api/sport/request` | 用 EGO-Planner 做局部占据图、重规划和避障，再把 B-spline 轨迹转换成机器狗速度命令 |
| 图像发布 | `go2_camera_bridge` | Go2 前置相机 JPEG 流 | `/go2/front_camera/image_raw` `/go2/front_camera/image_raw/compressed` | 使用 `unitree_sdk2_python` 拉流，再桥接成标准 ROS 2 图像话题 |

## 目录结构

```text
~/xxx
├── output
│   ├── go2_built_map.pcd
│   └── go2_named_poses.json
├── README.md
├── NAVIGATION_RESEARCH.md
├── 机器狗摄像头ROS2桥接简记.md
└── src
    ├── go2_camera_bridge
    ├── go2_mapping_minimal
    ├── go2_navigation
    ├── traj_utils
    ├── plan_env
    ├── path_searching
    ├── bspline_opt
    └── ego_planner
```

## 环境依赖

基础环境：

- Ubuntu 24.04
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
- `/pct_path`
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

`mapping_go2_builtin.launch.py` 现在默认就是“边建图边跑 EGO 局部规划”的组合模式。
它会同时启动：

- `go2_map_builder`：在线累积点云并保存 PCD
- `ego_planner_node`：基于点云和 odom 的局部规划与避障
- `go2_traj_server`：把 B-spline 局部轨迹转换成 `Twist`
- `unitree_sport_bridge`：把 `Twist` 发给机器狗
- `named_pose_gui`：记录命名点位到 JSON

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

运行时输出：

- `/go2_built_map`
- `/go2_robot_path`
- `/cmd_vel`

退出时保存：

```text
~/xxx/output/go2_built_map.pcd
```

运行时会弹出两个窗口：

- 命名点位窗口：输入地点名并点击“保存当前点位”
- 命名导航窗口：从 JSON 里选点并点击“发送选中目标”

点位结果默认保存到：

```text
~/xxx/output/go2_named_poses.json
```

如果地点名重复，后一次保存会覆盖同名地点的旧坐标。界面里也可以直接删除选中的点位。

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

```bash
ros2 launch go2_mapping_minimal mapping_go2_builtin.launch.py \
  named_pose_save_path:=/home/huang/xxx/output/my_named_poses.json
```

```bash
ros2 launch go2_mapping_minimal mapping_go2_builtin.launch.py \
  named_pose_gui:=false
```

```bash
ros2 launch go2_mapping_minimal mapping_go2_builtin.launch.py \
  named_goal_gui:=false
```

```bash
ros2 launch go2_mapping_minimal mapping_go2_builtin.launch.py \
  voxel_leaf_size:=0.05
```

```bash
ros2 launch go2_mapping_minimal mapping_go2_builtin.launch.py \
  voxel_leaf_size:=0.03 \
  downsample_every_n_scans:=20
```

```bash
ros2 launch go2_mapping_minimal mapping_go2_builtin.launch.py \
  voxel_leaf_size:=0 \
  downsample_every_n_scans:=0
```

默认行为：

- 每 5 帧点云发布一次在线累积地图
- 每 10 帧点云做一次体素下采样
- 默认体素大小 `0.05 m`
- 收到 `Ctrl+C` 或节点退出时自动保存 PCD
- 建图运行期间可通过图形界面记录当前 odom 点位，并生成 JSON 点位文件
- 建图运行期间可直接发送目标点或命名点位，不需要再单独启动导航 launch

建图稠密度说明：

- `voxel_leaf_size` 越小，地图越密，文件更大，内存和 CPU 占用也更高
- `downsample_every_n_scans` 越大，在线地图会保留更多原始点，但运行更吃内存
- `voxel_leaf_size:=0` 且 `downsample_every_n_scans:=0` 表示关闭体素滤波，能得到最密结果，但最容易把机器拖慢

## 2. 局部导航

这条链路现在是“参考路径 + 局部重规划 + 避障”的 EGO 本地导航，不再使用旧的 simple goal controller / obstacle filter。

### 2.1 直接启动 EGO 局部导航

启动：

```bash
cd ~/xxx
source /opt/ros/jazzy/setup.bash
source ~/xxx/install/setup.bash
ros2 launch go2_navigation simple_goal_nav.launch.py
```

默认话题流向：

```text
/pct_path + /utlidar/cloud_deskewed + /utlidar/robot_odom
                    |
                    v
              ego_planner_node -> planning/bspline
                    |
                    v
               go2_traj_server -> /cmd_vel
                    |
                    v
        unitree_sport_bridge -> /api/sport/request
```

如果你已经有上游路径规划器，保证它持续发布 `/pct_path` 即可。
如果你只是想手工测试，也可以先临时发一个 `nav_msgs/Path` 到 `/pct_path`。

常用输入：

- `/pct_path`
- `/utlidar/cloud_deskewed`
- `/utlidar/robot_odom`

### 2.2 加载保存地图并查看局部规划

这个模式会把保存好的 PCD 地图发布出来，方便在 RViz 里查看地图和局部规划。

启动：

```bash
cd ~/xxx
source /opt/ros/jazzy/setup.bash
source ~/xxx/install/setup.bash
ros2 launch go2_navigation click_goal_nav.launch.py
```

默认会同时打开 RViz。

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

- 这里的保存地图主要用于 RViz 显示参考
- 实际局部避障依赖实时 `/utlidar/cloud_deskewed` 和 `/utlidar/robot_odom`
- 当前主导航输入是 `/pct_path`，不是 `/goal_pose`
- `/utlidar/robot_odom`

默认输出：

- `/cmd_vel`
- `/go2_navigation/obstacle_filter_status`
- `/go2_navigation/fused_range_info`

默认策略包括：

- 前向距离过近时急停
- 接近障碍时按距离比例减速
- 根据左右净空差异选择偏置方向
- 根据高度图估计前方台阶、落差、坡度
- 超过地形硬阈值时触发 `terrain_stop`
- 左右地形评分不一致时优先偏向更可通行的一侧
- 传感器或里程计超时后触发 fail-safe 停车

主要参数在：

```text
src/go2_navigation/config/go2_navigation.yaml
```

比较常用的一组阈值：

- `front_stop_distance: 0.40`
- `terrain_hard_step_limit: 0.16`
- `terrain_soft_step_limit: 0.10`
- `terrain_hard_drop_limit: 0.16`
- `terrain_soft_drop_limit: 0.10`
- `terrain_hard_slope_deg: 40.0`
- `terrain_soft_slope_deg: 30.0`

高度图地形过滤当前实现说明：

- 默认订阅 `/utlidar/height_map_array`，消息类型是 `unitree_go/msg/HeightMap`
- 使用机器人脚下附近窗口作为基准高度
- 在机器人前方窗口统计最大上台阶、最大下落、局部坡度
- 当上台阶、下落或坡度超过硬阈值时停车
- 当超过软阈值但未超硬阈值时减速
- 对左右前方窗口分别打分，选择更平整、更安全的一侧施加横移和转向偏置

两个导航 launch 都支持覆盖高度图话题：

如果你要调避障效果，优先看：

- `ego_planner_node` 下的 `grid_map/*`
- `ego_planner_node` 下的 `manager/*`
- `ego_planner_node` 下的 `optimization/*`
- `go2_traj_server` 下的速度与朝向跟踪参数

## 4. MCP 点位服务

保存点位和发送点位现在也可以通过 MCP server 调用，不必依赖图形界面。

启动：

```bash
cd ~/xxx
source /opt/ros/jazzy/setup.bash
source ~/xxx/install/setup.bash
ros2 run go2_navigation navigation_mcp_server
```

这个 server 默认连接：

- odom: `/utlidar/robot_odom`
- goal: `/goal_pose`
- JSON: `/home/huang/xxx/output/go2_named_poses.json`

提供的 MCP tools：

- `go2_get_current_pose`
- `go2_list_named_poses`
- `go2_save_named_pose`
- `go2_delete_named_pose`
- `go2_send_named_goal`

## 5. 前置相机图像发布

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
