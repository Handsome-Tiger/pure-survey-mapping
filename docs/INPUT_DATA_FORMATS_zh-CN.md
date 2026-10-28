# 实地测试输入数据规范

## 1. 唯一主输入

流程主输入为 **ROS1 bag v2.0**（`.bag`），同一文件内同步记录 LiDAR、两路 IMU、RTK/导航与相机。代码以 ROS 原始序列化字节读取消息，因此 topic 名可以在 YAML 中改，但消息类型和字段布局必须与下表一致。

| 配置键 / 推荐 topic | ROS 消息类型 | 必需内容与单位 | 基线频率 |
|---|---|---|---:|
| `lidar_topic` `/livox/lidar` | `livox_ros_driver/CustomMsg` | `timebase` ns；每点 `offset_time` ns、`x/y/z` m、`reflectivity` 0–255、`tag`、`line` | 10 Hz |
| `livox_imu_topic` `/livox/imu` | `sensor_msgs/Imu` | `header.stamp`；角速度 rad/s；线加速度 m/s² | 约 200 Hz |
| `imu_topic` `/dji_osdk_ros/imu` 或 `/gnss/imu200` | `sensor_msgs/Imu` | `header.stamp`；角速度 rad/s；线加速度 m/s² | 约 200–400 Hz |
| `attitude_topic` `/dji_osdk_ros/attitude` 或 `/gnss/imu` | `geometry_msgs/QuaternionStamped` 或 `sensor_msgs/Imu` | `header.stamp`；有效、归一化的姿态四元数 `x,y,z,w`，与项目 ENU/机体系约定一致 | 1–100 Hz |
| `gnss_topic` `/dji_osdk_ros/rtk_position` | `sensor_msgs/NavSatFix` | WGS84 纬度/经度（度）、椭球高（m）、status/service | 5 Hz |
| `rtk_velocity_topic` `/dji_osdk_ros/rtk_velocity` 或 `/gnss/odom` | `geometry_msgs/Vector3Stamped` 或 `nav_msgs/Odometry` | `vector` 或 `twist.twist.linear`，m/s；在 YAML 中声明 NED/ENU | 1–5 Hz |
| `rtk_yaw_topic` `/dji_osdk_ros/rtk_yaw`（可选） | `std_msgs/Int16` | 原始偏航角，当前代码按百分之一度记录 | 5 Hz |
| `rtk_quality_topic` `/dji_osdk_ros/rtk_info_position`（可选） | `std_msgs/UInt8` | RTK 解状态；基线中数值 `50` 代表 fixed | 5 Hz |
| `camera.topic` `/left_camera/image/compressed` | `sensor_msgs/CompressedImage` | `header.stamp`、`format`、JPEG/PNG 压缩字节；不得二次改变分辨率 | 10 Hz |

“基线频率”是已跑通数据的参考值，不是解析器硬编码值。实际采集可以略有不同，但必须保证连续覆盖和足够的运动激励。时间戳用 ROS `secs+nsecs` 表示 Unix 秒；Livox 点时刻为 `timebase × 1e-9 + offset_time × 1e-9`。

姿态消息类型不同于默认值时，设置 `bag.attitude_message_type: sensor_msgs/Imu`。速度来自里程计时，设置 `bag.rtk_velocity_message_type: nav_msgs/Odometry`；若速度已经是 ENU，则同时设置 `bag.rtk_velocity_frame: enu`。`rtk_yaw_topic` 和 `rtk_quality_topic` 可以不配置，但 LiDAR、两路 IMU、姿态、GNSS、相机仍是生产流程必需输入。

`NavSatFix` 必须检查 `position_covariance` 和接收机解状态。只有确认 fixed RTK 且协方差可信时，才能使用厘米级 `fixed_rtk_sigma_m` 和高平移先验权重；普通单点 GNSS 应使用与其实际误差相符的米级 sigma，并把成果标为 GNSS 辅助相对建图，不能宣称厘米级绝对精度。

## 2. 强烈建议同时记录的时间诊断 topic

| topic | 类型 | 用途 |
|---|---|---|
| `/dji_osdk_ros/time_sync_gps_utc` | `dji_osdk_ros/GPSUTC` | GPS UTC 与 ROS 时间审计 |
| `/dji_osdk_ros/time_sync_fc_time_utc` | `dji_osdk_ros/FCTimeInUTC` | 飞控微秒时钟与 UTC 审计 |
| `/dji_osdk_ros/time_sync_pps_source` | `std_msgs/String` | PPS 来源存档，基线为 RTK |

这些 topic 生成硬件同步报告。算法主时差仍由 DJI IMU 与 Livox IMU 的三轴陀螺相关估计；生产阈值为相关系数不低于 0.80。

## 3. 必须单独提供的标定数据

把以下值填写进每套设备专属 YAML，数值均使用 SI 单位：

- `mapping.extrinsic.rotation_body_lidar`：3×3 旋转矩阵，LiDAR 坐标到机体坐标。
- `mapping.extrinsic.translation_body_lidar_m`：3 元平移，单位 m。
- `mapping.livox_imu_extrinsic.rotation_imu_lidar` 和 `translation_imu_lidar_m`：LiDAR/内置 IMU 刚体外参。
- `filter.lever_arm_body_m`：IMU/机体系原点到 GNSS 天线相位中心的杆臂，单位 m；不能长期保留零值冒充已标定。
- `camera.intrinsics`：`[[fx,0,cx],[0,fy,cy],[0,0,1]]`，对应 bag 中图像原始分辨率。
- `camera.distortion`：Brown-Conrady `[k1,k2,p1,p2]`。
- `camera.rotation_camera_lidar`、`translation_camera_lidar_m`：LiDAR 到相机的刚体变换。
- `camera.time_offset_s`：相机 header 修正到 Livox 时间域，定义为 `t_livox_camera = t_camera_header + offset`。

矩阵方向不可凭名称猜测后转置；应使用已知标定靶和投影叠加图验证。只要相机分辨率、焦距、安装角度、减震架或传感器位置改变，就重新标定。

## 4. 采集质量要求

- 开始前等待 RTK fixed 和 IMU 稳定，结束前后各保留一段静止数据。
- 规划相邻航带重叠与交叉航带/闭环；纯直线且无几何结构会使点面配准退化。
- 避免强烈振动、过曝、滚动快门拖影和 LiDAR 镜面/雨雾污染。
- 相机与 LiDAR 视场必须有足够重叠；相机帧应覆盖全部有效 LiDAR 时段。
- 不得在录包后独立重写某一路时间戳；若必须转换数据，要保留转换程序、原始时间和校验报告。
- 保留完整 bag，不要只导出点云。单独的 PCD/PLY、JPEG 文件夹、SBET/POS 或 CSV 不能直接替代本流程所需的逐点时间和 ROS 消息语义。

## 5. 入场自检命令

```bash
rosbag info --yaml /data/flight.bag
python3 scripts/run_production_pipeline.py \
  --config config/my_rig.yaml \
  --bag /data/flight.bag \
  --output /data/results/preflight \
  --dry-run
```

预检会核对配置指定的核心 topic、已启用可选 topic 及其消息类型、完整包限制、相机标定字段和 g2o。它不替代对时间戳单调性、频率、图像曝光和实际外参质量的人工/样本检查。


