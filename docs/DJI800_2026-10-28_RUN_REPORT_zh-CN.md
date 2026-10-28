# DJI800 2026-10-28 实跑记录

## 数据与结论

- 输入：`dji800_run_2026-10-28-10-59-32_1.bag`，1.88 GB，约 253 秒。
- 配置：`config/dji800_2026-10-28_relative_color.yaml`。
- 完整流程：GNSS/INS 初始轨迹、双 IMU 时间对齐、Livox 逐点去畸变、关键帧配准、g2o 位姿图优化、全帧几何点云、多视角相机着色、文件级验收。
- 结果：流程成功结束，所有自动质量门通过。

该包的 `NavSatFix.position_covariance` 对应约 7–15 m 的位置不确定度，不是 fixed RTK。因此本次成果定位为“普通 GNSS 辅助的相对优化彩色点云”，不能作为厘米级绝对测绘成果。

## 实跑输入话题

| 作用 | topic | 类型 | 数量 |
|---|---|---|---:|
| GNSS 坐标 | `/gnss/navsat` | `sensor_msgs/NavSatFix` | 253 |
| 高频导航 IMU | `/gnss/imu200` | `sensor_msgs/Imu` | 50,597 |
| 姿态四元数 | `/gnss/imu` | `sensor_msgs/Imu` | 253 |
| ENU 速度/姿态 | `/gnss/odom` | `nav_msgs/Odometry` | 253 |
| Livox IMU | `/livox/imu` | `sensor_msgs/Imu` | 51,447 |
| Livox 点云 | `/livox/lidar` | `livox_ros_driver/CustomMsg` | 2,529 |
| 相机 | `/left_camera/image/compressed` | `sensor_msgs/CompressedImage` | 2,530 |

`/gnss/imu.orientation` 和 `/gnss/odom.pose.pose.orientation` 均提供非零四元数；本次流水线使用 `/gnss/imu` 作为外部姿态源，使用 `/gnss/odom.twist.twist.linear` 作为 ENU 速度观测。

## 质量指标

- 双 IMU 时间偏移：`t_navigation = t_livox - 1.188 s`。
- 时间相关系数：0.96165，未使用固定偏移兜底。
- 雷达帧：2,529；关键帧：253。
- 配准候选：717；接受：316；接受约束平均点到面 RMSE：0.1783 m。
- 位姿图平均/最大平移修正：2.108 m / 6.487 m。
- 位姿图平均/最大旋转修正：1.171° / 7.646°。
- 原始雷达点：60,696,000；有效完整点：56,545,926。
- 真彩点：55,712,525；真彩覆盖率：98.526%。
- 雷达帧相机配对率：99.605%；平均最近图像时差：3.014 ms。
- 完整彩色 PLY：904,735,037 bytes；严格真彩 PLY：835,688,069 bytes。
- 文件验收：声明点数、实际点数、有限坐标数一致。

## 标定说明

本包图像分辨率为 1440×1080，而既有相机标定来自 2448×2048。本次按分辨率比例缩放内参，并沿用同一机架的 LiDAR—相机外参，用于验证完整工程链路。正式实地测绘前必须针对当前分辨率和实际安装状态重新完成内参、畸变、LiDAR—相机外参、GNSS 杆臂和投影叠加复核。

## 复现命令

```bash
python3 scripts/run_production_pipeline.py \
  --config config/dji800_2026-10-28_relative_color.yaml
```

Windows 成果目录：

```text
E:\桌面\三维建模\fast\outputs\dji800_run_2026-10-28-10-59-32_1
```

