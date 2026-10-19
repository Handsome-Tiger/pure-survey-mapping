# 纯测绘 GNSS/INS/LiDAR 建图与彩色点云说明

## 1. 项目边界

本版本只使用原始 ROS bag 内的 GNSS、IMU、姿态、Livox Avia LiDAR 和左相机数据，不读取或依赖 mapping-fusion 的地图、轨迹或点云。

测试数据：`/home/shuai/code/datasets/HKisland_GNSS02.bag`，完整有效 LiDAR 帧数为 4,349。

## 2. 建图流程

1. 将 RTK 经纬高转换到局部 ENU 坐标。
2. 使用误差状态 INS 融合 IMU、DJI 姿态、RTK位置和RTK速度；RTK质量决定位置观测噪声。
3. 按 Livox 点内 `offset_time` 插值载体位姿，对每个扫描逐点运动补偿。
4. 每10帧建立一个去畸变关键帧，以GNSS/INS相对位姿为初值执行鲁棒点到面配准。
5. 只接受重叠率、点面RMSE、平移校正和旋转校正均达标的激光约束。
6. 在G2O位姿图中联合激光约束、RTK位置先验和DJI姿态先验，使用Huber核优化。
7. 将关键帧校正连续插值到每个雷达点时刻，重新投影全部有效点，输出完整几何地图。

核心脚本：

- `scripts/build_lidar_keyframes.py`
- `scripts/build_lidar_constraints.py`
- `scripts/optimize_lidar_pose_graph.py`
- `scripts/build_optimized_cloud.py`
- `scripts/build_optimized_color_cloud.py`

完整配置：`config/HKisland_GNSS02_joint_all.yaml`。

## 3. 彩色点云

赋色遵循“世界坐标变换、相机投影、双线性插值”思路，并增加了运动补偿、镜头畸变和遮挡处理：

1. 每个LiDAR点按自身时间变换到初始世界坐标。
2. 再把静态世界点运动补偿到相机曝光时刻的LiDAR坐标系。
3. 使用实测 `R_cl`、`P_cl` 变换到相机坐标系。
4. 使用相机内参及 `k1,k2,p1,p2` Brown-Conrady畸变投影。
5. 通过像素深度缓冲去除被遮挡点。
6. 对可见投影位置进行双线性RGB采样。
7. 彩色PLY只写真实取到图像颜色的点，不用强度伪彩或灰色回退。

相机时间戳加 `0.1 s` 后与LiDAR对齐。完整包实测平均绝对时间差为 `0.107 ms`，最大 `0.222 ms`。

## 4. 完整包结果

- 关键帧：435
- 候选激光约束：756
- 接受激光约束：557，其中相邻385、航带/闭环172
- 接受边平均初始最近邻RMSE：1.012 m
- 接受边平均最终点面RMSE：0.259 m
- 位姿图平均/最大位置校正：0.103 m / 1.705 m
- 位姿图平均/最大姿态校正：0.926 deg / 5.160 deg
- 几何点云：74,367,651点
- 彩色点云：72,613,437点，真实RGB覆盖率97.64%
- RTK更新前创新3D RMSE：0.277 m，P95为0.525 m，2247次更新全部接受
- 自动测试：35项通过

## 5. 推荐成果

CloudCompare优先打开：

- `outputs_joint_kfgins_all/cloud_optimized_colorized_clean.ply`
- `outputs_joint_kfgins_all/cloud_optimized_geometry_clean.ply`

`clean`版本只删除高度低于-50 m或高于80 m的明显离群回波。全量版本仍保留：

- `outputs_joint_kfgins_all/cloud_optimized_colorized.ply`
- `outputs_joint_kfgins_all/cloud_optimized_geometry.ply`

所有成果均为标准 `binary_little_endian PLY`，文件头声明点数与实际载荷严格一致。彩色PLY包含 `red/green/blue` 字段；几何PLY包含 `intensity` 字段。

快速预览：`outputs_joint_kfgins_all/cloud_optimized_colorized_clean_preview.png`。

## 6. 复现命令

```bash
cd /home/shuai/cehui
python3 scripts/build_lidar_keyframes.py --config config/HKisland_GNSS02_joint_all.yaml
python3 scripts/build_lidar_constraints.py --config config/HKisland_GNSS02_joint_all.yaml
python3 scripts/optimize_lidar_pose_graph.py --config config/HKisland_GNSS02_joint_all.yaml
python3 scripts/build_optimized_cloud.py --config config/HKisland_GNSS02_joint_all.yaml
python3 scripts/build_optimized_color_cloud.py --config config/HKisland_GNSS02_joint_all.yaml
python3 -m pytest -q
```

## 7. 精度指标的正确解释

`0.259 m` 是已接受重叠关键帧之间的点到面配准残差，不是整幅地图的绝对RMSE。`0.277 m` 是RTK更新前的滤波创新RMSE，也不是独立真值误差。它们可用于发现内部不一致、轨迹跳变和配准退化。

要给出可审计的绝对平面/高程精度，仍需布设独立控制点或检查点，并用未参与建图的全站仪/RTK测量坐标计算水平RMSE、垂直RMSE和三维RMSE。没有独立真值时，不应把上述内部残差写成测图绝对精度。
