# ROS2 Topic 逐项 Echo 推测报告

- 生成时间：2026-03-21
- 方法：先执行 `ros2 topic list` 获取全部 123 个 topic，再对每个 topic 执行一次限时 `ros2 topic echo --once` 抽样。
- 说明：`1.5 秒内未抓到样本` 不代表 topic 无效，只表示在该窗口内没有成功收到消息；这类条目的功能推测主要依赖 topic 名称和消息类型，置信度更低。

| Topic | 类型 | Echo 观察 | 推测功能 | 置信度 |
|---|---|---|---|---|
| `/api/arm/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 机械臂模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/assistant_recorder/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 助手录音模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/assistant_recorder/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | 助手录音模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/audiohub/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 音频中枢模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/audiohub/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | 音频中枢模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/bashrunner/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | Bash 执行器模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/bashrunner/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | Bash 执行器模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/config/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 配置管理模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/config/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | 配置管理模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/fourg_agent/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 4G 通信代理模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/fourg_agent/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | 4G 通信代理模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/gas_sensor/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 气体传感器模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/gas_sensor/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | 气体传感器模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/gesture/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 手势模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/gpt/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | GPT模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/gpt/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | GPT模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/motion_switcher/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 运动模式切换模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/motion_switcher/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | 运动模式切换模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/obstacles_avoid/request` | `unitree_api/msg/Request` | 抓到 API 请求头，api_id=1002 | 避障模块的 API 请求入口；样本出现 api_id=1002，说明有组件在主动查询或设置避障状态。 | 高 |
| `/api/obstacles_avoid/response` | `unitree_api/msg/Response` | 响应里携带 JSON 数据，显示 enable=true | 避障模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 中高 |
| `/api/pet/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 宠物交互/宠物动作模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/pet/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | 宠物交互/宠物动作模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/programming_actuator/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 可编程执行器模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/programming_actuator/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | 可编程执行器模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/rm_con/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | RM 连接/遥管连接模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/robot_state/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 机器人状态模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/robot_state/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | 机器人状态模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/slam_operate/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | SLAM 操作模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/slam_operate/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | SLAM 操作模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/sport/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 运动控制模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/sport/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | 运动控制模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/sport_lease/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 运动控制租约模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/sport_lease/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | 运动控制租约模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/uwbswitch/request` | `unitree_api/msg/Request` | 抓到 API 请求头，api_id=1003 | UWB 开关模块的 API 请求入口；样本出现 api_id=1003，说明有组件在查询或切换 UWB。 | 高 |
| `/api/uwbswitch/response` | `unitree_api/msg/Response` | 响应里携带 JSON 数据，显示 enable=0 | UWB 开关模块的 API 响应出口；样本显示 UWB 当前为关闭/0。 | 高 |
| `/api/videohub/request` | `unitree_api/msg/Request` | 抓到 API 请求头，api_id=1001 | 视频中枢的 API 请求入口；样本出现 api_id=1001，推测用于拉流、切流或状态查询。 | 高 |
| `/api/videohub/response` | `unitree_api/msg/Response` | 抓到 API 响应，status.code=0 | 视频中枢的 API 响应出口；样本 code=0，表示对应视频操作成功。 | 高 |
| `/api/voice/request` | `unitree_api/msg/Request` | 1.5 秒内未抓到样本 | 语音模块的 API 请求入口，用于上层向该功能下发命令或查询。 | 低 |
| `/api/voice/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | 语音模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/api/vui/request` | `unitree_api/msg/Request` | 抓到 API 请求头，api_id=1006 | 语音 UI 的 API 请求入口；样本出现 api_id=1006，说明系统存在面向语音 UI 的控制接口。 | 高 |
| `/api/vui/response` | `unitree_api/msg/Response` | 1.5 秒内未抓到样本 | 语音 UI模块的 API 响应出口，用于返回执行状态、结果数据或错误码。 | 低 |
| `/arm/action/state` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 机械臂动作状态广播，适合给上层显示当前动作阶段或结果。 | 低 |
| `/arm_Command` | `unitree_arm/msg/ArmString` | 1.5 秒内未抓到样本 | 机械臂字符串命令通道，用于发送机械臂动作/控制指令。 | 低 |
| `/arm_Feedback` | `unitree_arm/msg/ArmString` | 1.5 秒内未抓到样本 | 机械臂反馈通道，用于返回机械臂执行结果或状态。 | 低 |
| `/audio_msg` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 音频相关的字符串消息通道，可能用于文本化音频事件或调试信息。 | 低 |
| `/audiohub/player/state` | `std_msgs/msg/String` | 抓到播放器状态 JSON，含 play_state/is_playing/current_audio_* | 音频播放器状态广播，直接反映播放状态、当前音频 ID 和自定义名称。 | 高 |
| `/audioreceiver` | `unitree_go/msg/AudioData` | 1.5 秒内未抓到样本 | 音频接收流通道，按消息类型看用于接收音频帧。 | 低 |
| `/audiosender` | `unitree_go/msg/AudioData` | 抓到音频帧字节流 | 音频发送流通道；样本是原始音频字节帧，推测用于麦克风/对讲上行。 | 高 |
| `/config_change_status` | `unitree_go/msg/ConfigChangeStatus` | 1.5 秒内未抓到样本 | 配置变更状态通道，用于通知配置下发是否生效。 | 低 |
| `/frontvideostream` | `unitree_go/msg/Go2FrontVideoData` | 抓到前视视频字节流，字段为 video720p | 前视视频裸流通道，样本包含 video720p 字节块。 | 高 |
| `/gas_sensor` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 气体传感器字符串输出，可能封装浓度或报警文本。 | 低 |
| `/gesture/result` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 手势识别结果输出。 | 低 |
| `/gnss` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | GNSS 定位字符串输出，可能封装经纬度或定位状态。 | 低 |
| `/go2/front_camera/image_raw` | `sensor_msgs/msg/Image` | 抓到前相机原始图像，1920x1080，bgr8 | Go2 前相机原始图像话题。 | 高 |
| `/go2/front_camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | 抓到前相机压缩图像，JPEG | Go2 前相机压缩图像话题，便于低带宽传输或 Web 可视化。 | 高 |
| `/gpt_cmd` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | GPT 模块命令输入或转发通道。 | 低 |
| `/gpt_state` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | GPT 模块状态广播。 | 低 |
| `/gptflowfeedback` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | GPT 工作流反馈通道，用于回传阶段性执行信息。 | 低 |
| `/lf/battery_alarm` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 低层/底盘侧电池告警通道。 | 低 |
| `/lf/lowstate` | `unitree_go/msg/LowState` | 抓到底层状态帧，含协议头/head 与 level_flag | 低层状态通道，和 /lowstate 类似，但命名看起来属于 lf 命名空间。 | 中高 |
| `/lf/sportmodestate` | `unitree_go/msg/SportModeState` | 抓到运动状态，含 IMU 四元数和 error_code | lf 命名空间下的运动状态通道，包含 IMU 与运动模式状态。 | 高 |
| `/lio_sam_ros2/mapping/odometry` | `nav_msgs/msg/Odometry` | 1.5 秒内未抓到样本 | LIO-SAM 建图里程计输出，用于 SLAM / 建图定位链路。 | 低 |
| `/lowcmd` | `unitree_go/msg/LowCmd` | 抓到底层控制帧，含协议头/head 与 level_flag | 机器人底层控制命令总线，用于向电机/底盘发送低级控制帧。 | 高 |
| `/lowstate` | `unitree_go/msg/LowState` | 抓到底层状态帧，含协议头/head 与 level_flag | 机器人底层状态总线，用于回传硬件/电机/IMU 等底层状态。 | 高 |
| `/multiplestate` | `std_msgs/msg/String` | 抓到全局状态 JSON，含 brightness/obstaclesAvoidSwitch/uwbSwitch/volume | 多项系统状态聚合通道；样本显示亮度、避障、UWB、音量等总开关。 | 高 |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | 1.5 秒内未抓到样本 | ROS 2 参数变更事件标准话题。 | 高 |
| `/pctoimage_local` | `unitree_interfaces/msg/PcToImage` | 1.5 秒内未抓到样本 | 点云转图像的本地处理结果，用于把三维感知映射到图像平面。 | 低 |
| `/pet/flowfeedback` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 宠物交互/宠物动作流程反馈。 | 低 |
| `/programming_actuator/command` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 可编程执行器的命令通道。 | 低 |
| `/programming_actuator/feedback` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 可编程执行器的反馈通道。 | 低 |
| `/public_network_status` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 公网连通性或外网状态广播。 | 低 |
| `/qt_add_edge` | `unitree_interfaces/msg/QtEdge` | 1.5 秒内未抓到样本 | Qt 侧新增图边指令/数据，推测用于地图图结构编辑。 | 低 |
| `/qt_add_node` | `unitree_interfaces/msg/QtNode` | 1.5 秒内未抓到样本 | Qt 侧新增图节点指令/数据。 | 低 |
| `/qt_command` | `unitree_interfaces/msg/QtCommand` | 1.5 秒内未抓到样本 | Qt 界面到后端的命令通道。 | 低 |
| `/qt_notice` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | Qt 界面通知消息通道。 | 低 |
| `/query_result_edge` | `unitree_interfaces/msg/QtEdge` | 1.5 秒内未抓到样本 | 图边查询结果返回。 | 低 |
| `/query_result_node` | `unitree_interfaces/msg/QtNode` | 1.5 秒内未抓到样本 | 图节点查询结果返回。 | 低 |
| `/rosout` | `rcl_interfaces/msg/Log` | 1.5 秒内未抓到样本 | ROS 2 标准日志输出话题。 | 高 |
| `/rtc/state` | `std_msgs/msg/String` | 抓到 RTC 状态 JSON，connection_state=not_connected | RTC 连接状态广播；样本显示当前未连接。 | 高 |
| `/rtc_status` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | RTC 相关状态补充通道，可能给 UI 或上层同步更细状态。 | 低 |
| `/selftest` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 系统自检结果或自检进度通道。 | 低 |
| `/servicestate` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 服务整体状态通道。 | 低 |
| `/servicestateactivate` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 服务激活状态通道。 | 低 |
| `/slam_info` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | SLAM 普通状态信息。 | 低 |
| `/slam_key_info` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | SLAM 关键状态信息，可能面向 UI 显示关键节点/关键帧状态。 | 低 |
| `/sportmodestate` | `unitree_go/msg/SportModeState` | 抓到运动状态，含 IMU 四元数和 error_code | 运动模式状态总线，包含姿态和错误码。 | 高 |
| `/uslam/client_command` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | uSLAM 客户端命令通道。 | 低 |
| `/uslam/cloud_map` | `sensor_msgs/msg/PointCloud2` | 1.5 秒内未抓到样本 | uSLAM 全局地图点云。 | 低 |
| `/uslam/frontend/cloud_world_ds` | `sensor_msgs/msg/PointCloud2` | 1.5 秒内未抓到样本 | uSLAM 前端降采样世界点云。 | 低 |
| `/uslam/frontend/odom` | `nav_msgs/msg/Odometry` | 1.5 秒内未抓到样本 | uSLAM 前端里程计。 | 低 |
| `/uslam/localization/cloud_world` | `sensor_msgs/msg/PointCloud2` | 1.5 秒内未抓到样本 | uSLAM 定位阶段使用的世界坐标点云。 | 低 |
| `/uslam/localization/odom` | `nav_msgs/msg/Odometry` | 1.5 秒内未抓到样本 | uSLAM 定位里程计。 | 低 |
| `/uslam/map_file_pub` | `sensor_msgs/msg/PointCloud2` | 1.5 秒内未抓到样本 | uSLAM 地图文件发布通道。 | 低 |
| `/uslam/map_file_sub` | `sensor_msgs/msg/PointCloud2` | 1.5 秒内未抓到样本 | uSLAM 地图文件订阅/导入通道。 | 低 |
| `/uslam/navigation/global_path` | `sensor_msgs/msg/PointCloud2` | 1.5 秒内未抓到样本 | uSLAM 导航全局路径；虽然类型是 PointCloud2，但命名明确指向全局路径可视化/交换。 | 低 |
| `/uslam/server_log` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | uSLAM 服务端日志通道。 | 低 |
| `/utlidar/client_command` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | UT 激光雷达客户端命令通道。 | 低 |
| `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | 抓到点云，frame_id=utlidar_lidar，width=4166 | UT 激光雷达原始点云。 | 高 |
| `/utlidar/cloud_base` | `sensor_msgs/msg/PointCloud2` | 抓到点云，frame_id=base_link，width=1626 | 变换到 base_link 的激光雷达点云。 | 高 |
| `/utlidar/cloud_deskewed` | `sensor_msgs/msg/PointCloud2` | 抓到点云，frame_id=odom，width=11232 | 经过去畸变/运动补偿后的点云。 | 高 |
| `/utlidar/grid_map` | `sensor_msgs/msg/PointCloud2` | 1.5 秒内未抓到样本 | 基于雷达生成的栅格地图。 | 低 |
| `/utlidar/height_map` | `sensor_msgs/msg/PointCloud2` | 1.5 秒内未抓到样本 | 基于雷达生成的高度点云/高度地图。 | 低 |
| `/utlidar/height_map_array` | `unitree_go/msg/HeightMap` | 抓到高度图，分辨率约 0.05999999865889549，尺寸 128x128 | 二维高度图数组，用于地形/可通行性分析。 | 高 |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | 抓到 IMU 姿态数据 | 雷达配套 IMU 数据。 | 高 |
| `/utlidar/lidar_state` | `unitree_go/msg/LidarState` | 抓到雷达状态，含版本、转速、error_state、cloud_frequency | 雷达设备状态与健康信息。 | 高 |
| `/utlidar/mapping_cmd` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 雷达建图命令通道。 | 低 |
| `/utlidar/range_info` | `geometry_msgs/msg/PointStamped` | 抓到单点范围信息 | 局部量测范围/目标点信息。 | 中高 |
| `/utlidar/range_map` | `sensor_msgs/msg/PointCloud2` | 1.5 秒内未抓到样本 | 范围图/距离图点云。 | 低 |
| `/utlidar/robot_odom` | `nav_msgs/msg/Odometry` | 抓到里程计，child_frame_id=base_link | 基于雷达或融合结果的机器人里程计。 | 高 |
| `/utlidar/robot_pose` | `geometry_msgs/msg/PoseStamped` | 抓到位姿消息，frame_id=odom | 基于雷达或融合结果的机器人位姿。 | 高 |
| `/utlidar/server_log` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 雷达服务端日志。 | 低 |
| `/utlidar/switch` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 雷达功能开关控制通道。 | 低 |
| `/utlidar/voxel_map` | `sensor_msgs/msg/PointCloud2` | 1.5 秒内未抓到样本 | 体素地图点云。 | 低 |
| `/utlidar/voxel_map_compressed` | `unitree_go/msg/VoxelMapCompressed` | 1.5 秒内未抓到样本 | 压缩后的体素地图。 | 低 |
| `/uwbstate` | `unitree_go/msg/UwbState` | 1.5 秒内未抓到样本 | UWB 状态通道，可能包含定位/设备状态。 | 低 |
| `/uwbswitch` | `unitree_go/msg/UwbSwitch` | 1.5 秒内未抓到样本 | UWB 开关状态或控制消息通道。 | 低 |
| `/videohub/inner` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 视频中枢内部字符串消息，可能用于内部路由或状态同步。 | 低 |
| `/webrtcreq` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | WebRTC 请求通道。 | 低 |
| `/webrtcres` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | WebRTC 响应通道。 | 低 |
| `/wirelesscontroller` | `unitree_go/msg/WirelessController` | 1.5 秒内未抓到样本 | 无线手柄/遥控器处理后输入。 | 低 |
| `/wirelesscontroller_unprocessed` | `unitree_go/msg/WirelessController` | 1.5 秒内未抓到样本 | 无线手柄/遥控器原始未处理输入。 | 低 |
| `/xfk_webrtcreq` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 另一套 WebRTC 请求通道，可能面向 xfk 子系统。 | 低 |
| `/xfk_webrtcres` | `std_msgs/msg/String` | 1.5 秒内未抓到样本 | 另一套 WebRTC 响应通道，可能面向 xfk 子系统。 | 低 |