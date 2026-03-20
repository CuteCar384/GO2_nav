# Go2 导航调研与当前结论

## 先说结论

1. 可以实现“给定目标点，机器狗自主走到目标点”。
   当前仓库原本缺的是控制闭环，不是传感器。机器人在线已经有 `/utlidar/robot_odom`、`/utlidar/robot_pose`、`/api/sport/request` 等关键话题，所以我已经补了一个最小可用闭环：
   - `go2_navigation/simple_goal_controller`
   - `go2_navigation/unitree_sport_bridge`

2. 可以实现“跨楼层导航”，但不建议把它理解成“直接拿一个纯 3D Nav2 体素图就一路规划到底”。
   Nav2 官方 `Voxel Layer` 会维护 3D 环境模型，但最终还是“压扁到 2D”给规划和控制使用；更适合当前场景的做法是：
   - 每一层做稳定的局部/全局导航
   - 楼层之间用拓扑图或路由图连接
   - 电梯、楼梯、门禁做成专门的行为节点

3. 想把导航效果做得最好，推荐分三阶段：
   - 阶段 A：先跑通当前已实现的“目标点闭环”
   - 阶段 B：把 Nav2 接到 `cmd_vel -> Unitree Sport API` 桥上
   - 阶段 C：做多楼层路由图、楼梯/电梯行为、地图切换和跨层定位

## 本地现状

- 当前原始仓库只实现了在线累积点云地图：
  - [README.md](/home/huang/xxx/README.md)
  - [go2_map_builder.cpp](/home/huang/xxx/src/go2_mapping_minimal/src/go2_map_builder.cpp)
- 它订阅：
  - `/utlidar/cloud_deskewed`
  - `/utlidar/robot_odom`
- 它发布：
  - `/go2_built_map`
  - `/go2_robot_path`
- 它没有：
  - 标准 `cmd_vel` 接口
  - 目标点控制器
  - 局部规划/全局规划
  - 恢复行为
  - 多楼层图或楼层切换逻辑

## 这次新增的实现

新增包位置：
- [go2_navigation](/home/huang/xxx/src/go2_navigation)

核心文件：
- [simple_goal_controller.py](/home/huang/xxx/src/go2_navigation/go2_navigation/simple_goal_controller.py)
- [unitree_sport_bridge.py](/home/huang/xxx/src/go2_navigation/go2_navigation/unitree_sport_bridge.py)
- [publish_goal.py](/home/huang/xxx/src/go2_navigation/go2_navigation/publish_goal.py)
- [simple_goal_nav.launch.py](/home/huang/xxx/src/go2_navigation/launch/simple_goal_nav.launch.py)

### 1. `simple_goal_controller`

输入：
- `/utlidar/robot_odom`
- `/go2_navigation/goal` (`geometry_msgs/PoseStamped`)

输出：
- `/cmd_vel`

作用：
- 用里程计闭环控制机器人向目标点靠近
- 到点后自动停止
- 支持最终朝向对齐

### 2. `unitree_sport_bridge`

输入：
- `/cmd_vel`

输出：
- `/api/sport/request`

作用：
- 把标准 `geometry_msgs/Twist` 转成 Unitree 的 `Move` / `StopMove` 请求
- 保留后续接 Nav2 的标准入口

### 3. `publish_goal`

作用：
- 从命令行直接发一个 `PoseStamped` 目标点，便于现场测试

## 为什么“跨楼层”不能直接等同于“纯 3D Nav2”

Nav2 官方 `Voxel Layer` 的定位是：用 3D 传感器构建 3D 环境模型，但最终仍然压缩成 2D 给规划与控制层使用。这对障碍感知很好，但不代表它已经是“跨楼层体素导航系统”。

因此，跨楼层的推荐方案是：

1. 每层维护自己的局部导航地图。
2. 用 `Route Server` 或自定义楼层图表达楼梯、电梯、连接门等“跳转边”。
3. 在边上挂自定义行为：
   - 走到电梯门口
   - 等门开
   - 进电梯
   - 电梯到目标层后切地图/重定位
   - 出电梯后继续 Nav2
4. 楼梯场景如果要全自动，需要单独做楼梯检测和 gait/motion 切换，而不是只靠 `cmd_vel`。

## 关于 Unitree 当前能力边界

- 本机在线确实能看到：
  - `/api/sport/request`
  - `/api/slam_operate/request`
  - `/uslam/cloud_map`
  - `/uslam/localization/odom`
  - `/uslam/navigation/global_path`
- 这说明机器人侧存在更完整的 SLAM / Navigation 能力入口。
- 但当前可直接稳定依赖、且已在官方 ROS2/SDK2 中清楚暴露的高层运动接口，核心仍然是 `Move` / `StopMove` 这一类速度式控制。
- 当前官方 Go2 SDK2 `sport_client.hpp` 里没有 `TrajectoryFollow` 这样的 Go2 高层路径跟随接口。
- Unitree 官方 `unitree_ros2` 发布说明里也明确提到 `TrajectoryFollow`、`SwitchGait` 等接口在新版本中不再支持。

这意味着：
- 目标点到达：可以做，而且现在已经有基础实现。
- 多楼层自主：可以做，但要靠“路由图 + 行为节点 + 地图/定位切换”，不能指望一个现成 Go2 高层路径接口直接包办。

## 当前最推荐的落地路线

### 路线 1：先验证目标点闭环

```bash
cd ~/xxx
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/xxx/install/setup.bash
ros2 launch go2_navigation simple_goal_nav.launch.py
```

另开一个终端发目标点：

```bash
cd ~/xxx
source /opt/ros/jazzy/setup.bash
source ~/xxx/install/setup.bash
ros2 run go2_navigation publish_goal --x 1.0 --y 0.0 --yaw 0.0
```

### 路线 2：接入 Nav2

等目标点闭环稳定后，把 Nav2 的控制输出接到 `/cmd_vel`，复用当前的 `unitree_sport_bridge`。

### 路线 3：做多楼层

在 Nav2 之上补：
- 多楼层地图管理
- Route Graph
- 楼梯/电梯行为节点
- 地图切换与跨层重定位

## 外部官方资料

- Nav2 Voxel Layer:
  https://docs.nav2.org/configuration/packages/costmap-plugins/voxel.html
- Nav2 Route Server:
  https://docs.nav2.org/configuration/packages/configuring-route-server.html
- Nav2 Navigation Plugins:
  https://docs.nav2.org/plugins/
- Unitree ROS2 官方仓库:
  https://github.com/unitreerobotics/unitree_ros2
- Unitree SDK2 Go2 sport client:
  https://github.com/unitreerobotics/unitree_sdk2/blob/main/include/unitree/robot/go2/sport/sport_client.hpp
- Unitree ROS2 releases:
  https://github.com/unitreerobotics/unitree_ros2/releases
