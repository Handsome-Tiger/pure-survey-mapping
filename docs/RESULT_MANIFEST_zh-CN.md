# 最终成果清单

生成日期：2026-07-21  
方案归档日期：2026-10-19

## 主成果

```text
outputs_joint_kfgins_all_livox_deskew_color/cloud_optimized_color_complete.ply
```

- 格式：PLY `binary_little_endian 1.0`
- 字段：`float x, float y, float z, uchar red, uchar green, uchar blue, uchar color_valid`
- 点数：74,367,651
- 文件大小：1,189,882,637 bytes
- SHA-256：`53a7a670872537084d052074736981c39e2d3d2f9142979d0904082b923d335b`
- 有限坐标点：74,367,651
- 真实相机颜色点：73,125,606（98.3299%）
- 未观测点：1,242,045，以强度灰度保存且 `color_valid=0`
- XYZ 最小值（m）：`[-349.708679, -579.766479, -351.223999]`
- XYZ 最大值（m）：`[135.255005, 174.413391, 84.202904]`

## 关键运行证据

- 有效 LiDAR 扫描：4,349
- 原始回波：313,128,000
- LiDAR→导航时差：-0.572 s
- 双 IMU 相关系数：0.929582
- 相机配对率：100%
- 最近相机平均/最大时间差：0.0004269 s / 0.0123277 s
- 关键帧：435
- 接受/拒绝 LiDAR 约束：639 / 88
- 接受约束平均点面 RMSE：0.200841 m
- 最大平移/旋转优化改正：1.274467 m / 2.613691°

`full_color_report.json` 的五项验收门均通过，`color_cloud_verification.json` 的声明点数、实际二进制记录数、有限坐标数以及 strict/complete 颜色数关系全部一致。

这些是内部一致性和文件完整性证据，不是绝对测绘精度认证。对外报告绝对精度时，必须使用未参与建图的独立检查点。

大体积成果不进入 GitHub 普通 Git 历史；本清单的 SHA-256 用于核对离线交付文件。

