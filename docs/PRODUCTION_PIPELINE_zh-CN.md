# 正式生产流程与方案落档

## 1. 固化基线

目标成果为 `cloud_optimized_color_complete.ply`。已验证基线使用 `config/HKisland_GNSS02_joint_all_livox_deskew_color.yaml`，处理 4,349 个有效 Livox 扫描、74,367,651 个过滤后点；73,125,606 点获得真实相机 RGB，覆盖率 98.3299%。完整 PLY 的声明点数、实际记录数和有限坐标数均为 74,367,651。

算法只使用 bag 内原始观测，不使用 FAST-LIO、FAST-LIVO2、mapping-fusion、DJI L1 颜色迁移或三维近邻补色。

## 2. 七阶段生产链

1. **导航与关键帧**：读取 RTK 位置/质量/速度、DJI IMU/姿态、Livox IMU；运行误差状态 GNSS/INS；由双 IMU 陀螺相关估计 `t_navigation = t_livox + offset_s`；逐点去畸变并每 10 帧建关键帧。
2. **硬件时间诊断**：导出 GPS UTC、飞控 UTC 和 PPS 来源，与相关估计结果共同存档。
3. **LiDAR 约束**：以 GNSS/INS 相对位姿为初值做点到面配准；按重叠率、RMSE、平移和旋转改正阈值剔除退化边。
4. **位姿图优化**：组合 RTK 平移先验、姿态旋转先验和 LiDAR 边，使用 Huber 核及 g2o 优化，并执行最大位姿改正安全门。
5. **几何重建**：把连续插值的优化改正应用到全部去畸变 LiDAR 点，写入 binary little-endian PLY。
6. **相机赋色**：每个扫描选择时间窗内最多 3 帧图像，应用相机畸变模型、外参、z-buffer 可见性和双线性取色；最近有效观测胜出，不跨点传播颜色。
7. **独立校验**：重新读取 PLY 头与二进制载荷，检查声明/实际记录数、有限 XYZ、`color_valid` 取值，以及 strict 点数是否等于 complete 的有效颜色点数。

统一入口：

```bash
python3 scripts/run_production_pipeline.py \
  --config config/production.example.yaml \
  --bag /data/flight.bag \
  --output /data/results/flight_001
```

等价的人工分步命令为：

```bash
python3 -m pytest tests -q
python3 scripts/build_lidar_keyframes.py --config CONFIG.yaml
python3 scripts/analyze_time_sync.py --config CONFIG.yaml
python3 scripts/build_lidar_constraints.py --config CONFIG.yaml
python3 scripts/optimize_lidar_pose_graph.py --config CONFIG.yaml
python3 scripts/build_optimized_cloud.py --config CONFIG.yaml
python3 scripts/build_optimized_color_cloud.py --config CONFIG.yaml
python3 scripts/verify_color_cloud.py --config CONFIG.yaml
```

## 3. 生产前检查

- 使用刚性固定且完成热机的整套传感器；任何拆装都视为外参失效。
- 相机内参、Brown-Conrady 畸变、LiDAR→相机、LiDAR→机体和 LiDAR→Livox IMU 外参必须对应本次硬件。
- 所有传感器时间戳应单调且覆盖同一采集区间；必须记录 Livox 内置 IMU，禁止生产配置回退到固定时差。
- 完整运行中 `start_lidar_frame=0`，不得设置 `max_lidar_frames` 或 `max_points_per_frame`。
- 输出目录应为空或为本次采集专用；建议至少预留原始 bag 大小加 8 GiB 的空间。
- 先用短样本验证坐标轴、时间偏移和颜色投影，再执行全包。

## 4. 运行后验收

`full_color_report.json` 的所有 `acceptance_gates` 必须为 `true`；`color_cloud_verification.json` 的 `counts_consistent` 必须为 `true`。同时人工检查：

- 建筑边缘和杆状物是否出现双影；
- 往返航带是否分层；
- 颜色是否沿运动方向拖影或错贴；
- `optimization_report.json` 的最大改正是否逼近安全阈值；
- `hardware_time_sync_report.json` 和 `time_alignment_report.json` 是否相互矛盾。

基线内部一致性指标（GNSS 创新 3D RMSE 0.276585 m、接受约束点面 RMSE 0.200841 m）不可宣称为绝对精度。正式测绘报告必须使用未参与滤波/优化的独立检查点，分别报告水平、垂直和三维 RMSE、最大误差及样本数。

## 5. 可追溯性

每次交付需保存：原始 bag 的 SHA-256、Git commit、`resolved_production_config.yaml`、全部 JSON/CSV 报告、三份 PLY、检查点成果及人工验收记录。原始数据和大点云不进入普通 Git；如需发布样例，使用独立下载地址并明确许可、隐私与坐标脱敏策略。

