# 导航参数说明

本文说明当前 Go2 导航链路里实际生效的参数来源与含义。

当前链路：

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

主参数文件：

```text
src/go2_navigation/config/go2_navigation.yaml
```

## 1. ego_planner_node

这一层负责局部占据图、碰撞检查、轨迹重规划和 B-spline 生成。

### 1.1 可视化与 FSM

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `vis/frame_id` | `odom` | 可视化输出使用的坐标系。 |
| `fsm/flight_type` | `3` | EGO-Planner 的运行模式编号。当前工作区里用于地面机器人局部规划链路。 |
| `fsm/thresh_replan_time` | `0.35` | 距上次规划超过该时间后，允许触发重规划。单位秒。 |
| `fsm/thresh_no_replan_meter` | `0.30` | 若当前位置与当前轨迹偏离不足该距离，可抑制不必要重规划。单位米。 |
| `fsm/planning_horizon` | `2.5` | 局部规划空间前视范围。单位米。 |
| `fsm/planning_horizen_time` | `2.5` | 局部规划时间前视范围。单位秒。 |
| `fsm/emergency_time` | `1.0` | 发生碰撞风险时预留的应急反应时间。单位秒。 |
| `fsm/realworld_experiment` | `false` | 是否启用原始工程中的实机实验分支逻辑。当前保持关闭。 |
| `fsm/fail_safe` | `true` | 打开失效保护。出现关键异常时优先停车或重规划。 |
| `fsm/waypoint_num` | `0` | 多路点任务数量。当前链路不走这套 waypoint 管理。 |

### 1.2 grid_map

这一组参数决定局部障碍地图的尺度、分辨率、障碍膨胀和地面机器人过滤逻辑。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `grid_map/resolution` | `0.10` | 体素分辨率。越小越精细，但 CPU 和内存更高。单位米。 |
| `grid_map/map_size_x` | `20.0` | 局部地图 X 尺寸。单位米。 |
| `grid_map/map_size_y` | `20.0` | 局部地图 Y 尺寸。单位米。 |
| `grid_map/map_size_z` | `1.6` | 局部地图 Z 尺寸。单位米。 |
| `grid_map/local_update_range_x` | `4.0` | 机器人周围参与实时更新的 X 半范围。单位米。 |
| `grid_map/local_update_range_y` | `4.0` | 机器人周围参与实时更新的 Y 半范围。单位米。 |
| `grid_map/local_update_range_z` | `1.0` | 机器人周围参与实时更新的 Z 半范围。单位米。 |
| `grid_map/obstacles_inflation` | `0.25` | 障碍膨胀半径。它是当前“机器人尺寸近似”的关键来源之一。数值越大越保守。单位米。 |
| `grid_map/local_map_margin` | `10` | 局部地图边界额外保留的体素边距。 |
| `grid_map/ground_height` | `-0.10` | 地面基准高度。单位米。 |
| `grid_map/ground_robot_mode` | `true` | 启用地面机器人模式。打开后会按平面避障思路处理整列体素。 |
| `grid_map/ground_obstacle_min_height` | `0.20` | 低于该高度的点不作为地面障碍。可过滤地面噪点。单位米。 |
| `grid_map/ground_obstacle_max_height` | `1.30` | 高于该高度的点不作为地面障碍。单位米。 |
| `grid_map/self_clear_radius` | `0.40` | 机器人自身附近的点云忽略半径，用于避免把腿或机身附近回波当障碍。单位米。 |
| `grid_map/fx` | `387.2` | 相机/深度投影模型内参 `fx`。当前点云模式下通常不是主要调参项。 |
| `grid_map/fy` | `387.2` | 相机/深度投影模型内参 `fy`。 |
| `grid_map/cx` | `321.0` | 相机/深度投影模型内参 `cx`。 |
| `grid_map/cy` | `243.0` | 相机/深度投影模型内参 `cy`。 |
| `grid_map/use_depth_filter` | `false` | 是否启用深度图过滤。当前主要使用点云输入，因此关闭。 |
| `grid_map/depth_filter_tolerance` | `0.15` | 深度过滤容差。单位米。 |
| `grid_map/depth_filter_maxdist` | `5.0` | 深度过滤最大距离。单位米。 |
| `grid_map/depth_filter_mindist` | `0.2` | 深度过滤最小距离。单位米。 |
| `grid_map/depth_filter_margin` | `2` | 深度图边界过滤余量。像素。 |
| `grid_map/k_depth_scaling_factor` | `1000.0` | 深度缩放系数。 |
| `grid_map/skip_pixel` | `2` | 深度图降采样步长。 |
| `grid_map/p_hit` | `0.70` | 占据更新时命中概率。 |
| `grid_map/p_miss` | `0.35` | 占据更新时未命中概率。 |
| `grid_map/p_min` | `0.12` | 占据概率下限。 |
| `grid_map/p_max` | `0.90` | 占据概率上限。 |
| `grid_map/p_occ` | `0.80` | 判定为障碍的占据阈值。 |
| `grid_map/min_ray_length` | `0.1` | 射线投射最小长度。单位米。 |
| `grid_map/max_ray_length` | `4.5` | 射线投射最大长度。单位米。 |
| `grid_map/virtual_ceil_height` | `1.20` | 虚拟天花板高度。用于限制体素地图高度范围。单位米。 |
| `grid_map/visualization_truncate_height` | `1.20` | 地图可视化截断高度。单位米。 |
| `grid_map/show_occ_time` | `false` | 是否打印占据更新耗时。 |
| `grid_map/pose_type` | `2` | 位姿消息类型编号。`2` 对应 Odometry。 |
| `grid_map/frame_id` | `odom` | 局部地图坐标系。 |
| `grid_map/odom_depth_timeout` | `1.0` | 里程计与深度/点云同步超时阈值。单位秒。 |

调参建议：

- 想让机器人更不贴边，优先增大 `grid_map/obstacles_inflation`。
- 如果发现腿部或机身近处总被当成障碍，优先检查 `grid_map/self_clear_radius`。
- 如果低矮地面噪声太多，调 `ground_obstacle_min_height`。

### 1.3 manager

这一组参数决定局部轨迹生成时的速度、加速度和控制点分布约束。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `manager/max_vel` | `0.8` | 规划器内部允许的最大速度。单位米每秒。比底盘实际发送上限更高，最终还会被后级限幅。 |
| `manager/max_acc` | `1.5` | 规划器内部允许的最大加速度。单位米每二次方秒。 |
| `manager/max_jerk` | `4.0` | 规划器内部允许的最大加加速度。 |
| `manager/control_points_distance` | `0.2` | B-spline 控制点间距。越小轨迹越灵活，但优化量更大。单位米。 |
| `manager/feasibility_tolerance` | `0.05` | 可行性容差。 |
| `manager/planning_horizon` | `2.5` | 管理器侧采用的局部规划距离视野。单位米。 |
| `manager/use_distinctive_trajs` | `false` | 是否启用区分式轨迹集合。当前关闭。 |
| `manager/drone_id` | `-1` | 原始工程用于多机编号。当前地面单机链路里基本不使用。 |

### 1.4 optimization

这一组参数决定 B-spline 优化时各成本项的权重和最小安全距离。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `optimization/lambda_smooth` | `1.0` | 平滑成本权重。越大越平滑。 |
| `optimization/lambda_collision` | `0.6` | 碰撞避障成本权重。越大越倾向远离障碍。 |
| `optimization/lambda_feasibility` | `0.1` | 动力学可行性成本权重。 |
| `optimization/lambda_fitness` | `1.0` | 轨迹贴合参考路径的成本权重。越大越不愿偏离 `/pct_path`。 |
| `optimization/dist0` | `0.40` | 轨迹到障碍的目标安全距离。它是当前“机器人尺寸近似”的另一关键来源。单位米。 |
| `optimization/swarm_clearance` | `0.5` | 原始多机避碰安全距离。当前单机基本不敏感。单位米。 |
| `optimization/max_vel` | `0.8` | 优化器内部速度约束。单位米每秒。 |
| `optimization/max_acc` | `1.5` | 优化器内部加速度约束。单位米每二次方秒。 |

调参建议：

- 轨迹太贴边：先增大 `optimization/dist0`。
- 轨迹绕障不积极：可适当增大 `optimization/lambda_collision`。
- 轨迹过于死贴参考路径：可适当减小 `optimization/lambda_fitness`。

### 1.5 bspline

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `bspline/limit_vel` | `0.8` | B-spline 速度上限。单位米每秒。 |
| `bspline/limit_acc` | `1.5` | B-spline 加速度上限。单位米每二次方秒。 |
| `bspline/limit_ratio` | `1.1` | 速度和加速度限制的放宽比例。 |

### 1.6 prediction

这组参数来自原始 EGO-Planner 的动态障碍预测能力。当前链路里保留，但不是主要调参入口。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `prediction/obj_num` | `1` | 预测对象数量上限。 |
| `prediction/lambda` | `1.0` | 预测相关权重。 |
| `prediction/predict_rate` | `1.0` | 预测更新频率系数。 |

## 2. go2_traj_server

这一层负责把 `planning/bspline` 转换成机器人机体系下的 `cmd_vel`。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `control_rate_hz` | `20.0` | 轨迹跟踪控制频率。单位 Hz。 |
| `time_forward` | `0.30` | 用于计算朝向误差的前视时间。越大越看重前方轨迹方向。单位秒。 |
| `require_initial_heading_alignment` | `true` | 收到新目标或新轨迹后，是否先原地对准朝向再前进。 |
| `initial_heading_tolerance` | `0.35` | 初始对准朝向允许误差。单位弧度。 |
| `initial_alignment_goal_change_threshold` | `0.20` | 若新目标与上次目标变化超过该值，重新进入初始对准。单位米。 |
| `heading_deadband` | `0.12` | 朝向误差死区。误差落在该范围内时不再输出角速度。单位弧度。 |
| `heading_sign_flip_tolerance` | `0.20` | 朝向误差很小时，抑制角速度正负来回翻转。单位弧度。 |
| `min_linear_speed` | `0.30` | 非零线速度的最小值。低于它会被抬到该值，避免小速度拖不动。单位米每秒。 |
| `max_linear_x` | `0.30` | 最大发送前后速度。单位米每秒。 |
| `max_linear_y` | `0.00` | 最大发送侧向速度。当前为 0，表示不允许横移。 |
| `min_angular_speed` | `0.80` | 非零角速度最小值。避免小角速度转不起来。单位弧度每秒。 |
| `max_angular_z` | `0.80` | 最大发送角速度。单位弧度每秒。 |
| `heading_kp` | `1.80` | 朝向误差比例系数。越大转向越积极。 |
| `allow_reverse_motion` | `false` | 是否允许倒车。当前关闭，轨迹落到机身后半平面时会优先停住并继续转向。 |
| `reverse_heading_tolerance` | `1.20` | 禁止倒车模式下，允许继续转向的朝向误差阈值。单位弧度。 |
| `goal_tolerance` | `0.22` | 判定到达目标的平面距离阈值。单位米。 |
| `stop_yaw_tolerance` | `0.30` | 判定到达目标时的朝向误差阈值。单位弧度。 |

调参建议：

- 机器人总想先转很多再走：可略微增大 `initial_heading_tolerance`。
- 机器人转向时来回抖：先看 `heading_deadband` 和 `heading_sign_flip_tolerance`。
- 机器人低速起步无力：保持或略增 `min_linear_speed`。
- 不希望出现后退：保持 `allow_reverse_motion: false`。

## 3. unitree_sport_bridge

这一层把 `cmd_vel` 转成 Unitree Sport API 的 Move/Stop 请求。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `control_rate_hz` | `20.0` | 向 Unitree API 发控制请求的频率。单位 Hz。 |
| `command_timeout` | `0.3` | 超过该时间没收到新 `cmd_vel`，就发送 `StopMove`。单位秒。 |
| `linear_deadband` | `0.02` | 线速度死区。小于该值视为 0。单位米每秒。 |
| `angular_deadband` | `0.02` | 角速度死区。小于该值视为 0。单位弧度每秒。 |
| `min_linear_speed` | `0.30` | 非零线速度最小值。桥接层会再次做一次下限保护。单位米每秒。 |
| `max_linear_x` | `0.30` | 发给底盘的最大前后速度。单位米每秒。 |
| `max_linear_y` | `0.25` | 发给底盘的最大横向速度。单位米每秒。当前上游 `go2_traj_server` 已经把横移关掉。 |
| `min_angular_speed` | `0.80` | 非零角速度最小值。单位弧度每秒。 |
| `max_angular_z` | `0.80` | 发给底盘的最大角速度。单位弧度每秒。 |

注意：

- `go2_traj_server` 和 `unitree_sport_bridge` 都会做一层速度限幅。
- 真正发给机器人的是两层限制后更保守的那个值。

## 4. go2_saved_map_publisher

这个节点只负责把保存的 PCD 重发出来给 RViz 查看。

| 参数 | 当前值 | 说明 |
| --- | --- | --- |
| `publish_period` | `1.0` | 重发保存地图的周期。单位秒。 |

## 5. 当前最常用的调参入口

如果你的目标是快速改善现场表现，通常优先看下面这些参数：

### 5.1 想让机器人更保守、更不贴边

- `grid_map/obstacles_inflation`
- `grid_map/self_clear_radius`
- `optimization/dist0`

### 5.2 想让机器人转向更稳

- `time_forward`
- `heading_deadband`
- `heading_sign_flip_tolerance`
- `heading_kp`

### 5.3 想让机器人走得更快或更慢

- `go2_traj_server/max_linear_x`
- `unitree_sport_bridge/max_linear_x`
- `manager/max_vel`
- `optimization/max_vel`

### 5.4 想减少“对准后又后退”

- `allow_reverse_motion`
- `reverse_heading_tolerance`
- `require_initial_heading_alignment`
- `initial_heading_tolerance`

## 6. 当前参数的整体风格

当前参数更偏：

- 地面机器人模式
- 保守避障
- 不允许横移
- 不允许倒车
- 低速稳态优先，而不是激进穿行

如果后续你要切到更激进的穿障风格，通常会从这些方向改：

- 略减 `obstacles_inflation`
- 略减 `dist0`
- 略减 `min_linear_speed`
- 略增 `max_linear_x`
- 略增 `heading_kp`
