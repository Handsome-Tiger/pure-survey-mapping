# HKisland 项目学习与组会汇报执行计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to execute this plan task-by-task. 本任务需要学生逐章互动复述，因此在当前会话内执行。

**Goal:** 从零理解 HKisland 多传感器测绘建图项目，并完成一套约 15 分钟的项目进展 PPT、逐页讲稿和答辩题库。

**Architecture:** 采用“基础原理—项目证据—复述检查—PPT 沉淀”的循环。八章学习内容逐章推进，最后把已经确认的材料组合成汇报文件，并通过内容核查、视觉核查和模拟答辩验证。

**Tech Stack:** Markdown 学习笔记、项目 Python/ROS/YAML/JSON/CSV/PLY 文件、PowerPoint、项目现有 PNG 预览图。

## Global Constraints

- 主汇报时长约 15 分钟，PPT 控制在 12–14 页。
- 主 PPT 讲项目进展，完整技术原理放在学习笔记、演讲者备注和答辩题库中。
- 按“学过但已基本遗忘”讲解，不默认掌握 Python、ROS、线性代数或概率统计。
- 先讲物理直觉，再讲术语和公式，最后对应项目代码与结果。
- 不虚构学生独立手写 AI 生成代码的经历。
- 明确区分已验证事实、基于结果的推断和仍需验证的事项。
- 不把点云视觉效果直接等同于测绘精度，也不把插值新点当作独立测量。

---

## 文件结构

- Create: `docs/learning/01-project-overview.md` — 项目输入、输出、主线和术语地图。
- Create: `docs/learning/02-pointcloud-pose-coordinates.md` — 点云、坐标系与位姿基础。
- Create: `docs/learning/03-sensors.md` — GNSS、IMU、LiDAR、相机基础。
- Create: `docs/learning/04-time-and-extrinsics.md` — 时间同步、外参和杆臂。
- Create: `docs/learning/05-gnss-ins-fusion.md` — EKF 与 15 状态误差模型。
- Create: `docs/learning/06-lidar-optimization.md` — 去畸变、关键帧、约束和姿态图。
- Create: `docs/learning/07-color-pointcloud.md` — 相机投影、补色、裁剪和加密。
- Create: `docs/learning/08-evaluation-and-limitations.md` — 结果评价、失败分析和局限。
- Create: `docs/learning/defense-questions.md` — 老师追问题库与参考答案。
- Create: `docs/learning/presentation-script.md` — 逐页口语化讲稿。
- Create: `HKisland_15min_progress_report.pptx` — 最终 12–14 页汇报文件。

### Task 1: 建立项目全景

**Evidence:** `README.md`、`PROJECT_SUMMARY.md`、`config/*.yaml`、`scripts/run_mapping*.py`、各输出目录的报告文件。

- [ ] 用一句话定义项目目标，并区分原始输入、中间结果和最终输出。
- [ ] 列出 ROS bag 中关键 topic，解释每个 topic 的物理含义。
- [ ] 绘制从 ROS bag 到彩色三维点云的总体数据流。
- [ ] 对照项目文件确认流程中的真实脚本与输出，标记已验证、推断和待验证内容。
- [ ] 让学生不看答案复述完整流程；纠正遗漏后写入 `docs/learning/01-project-overview.md`。
- [ ] 形成 PPT 第 2–4 页素材和对应答辩问题。

### Task 2: 补齐点云、坐标系与位姿基础

**Evidence:** `survey_mapping/` 中坐标与姿态相关模块、轨迹 CSV、点云 PLY 头信息。

- [ ] 从二维地图类比解释三维点、点云、坐标系和位姿。
- [ ] 解释平移、旋转、旋转矩阵与四元数分别解决什么问题。
- [ ] 解释经纬高、ECEF、局部 ENU 与机体系之间的关系。
- [ ] 用项目轨迹和点云字段验证坐标含义。
- [ ] 完成复述题并写入 `docs/learning/02-pointcloud-pose-coordinates.md`。

### Task 3: 理解四类传感器

**Evidence:** bag topic、`scripts/extract_bag.py`、配置中的 topic 与噪声参数。

- [ ] 分别解释 GNSS/RTK、IMU、LiDAR、相机实际测量什么。
- [ ] 比较四类传感器的频率、长期稳定性、短期连续性和主要误差。
- [ ] 解释为何任何单一传感器都不足以完成该项目目标。
- [ ] 将传感器输出对应到项目字段和配置项。
- [ ] 完成复述题并写入 `docs/learning/03-sensors.md`，形成 PPT 第 3 页素材。

### Task 4: 理解时间同步与空间外参

**Evidence:** `scripts/analyze_time_sync.py`、时间同步 CSV/JSON、相机和 LiDAR 配置。

- [ ] 用“不同手表”和“不同安装位置”解释时间偏差与空间外参。
- [ ] 解释杆臂、旋转外参、相机内参和畸变参数。
- [ ] 对照时间同步报告说明项目做了哪些修正。
- [ ] 区分时间错误、姿态错误和外参错误在点云中的典型表现。
- [ ] 完成复述题并写入 `docs/learning/04-time-and-extrinsics.md`，形成 PPT 第 5 页素材。

### Task 5: 理解 GNSS/INS 融合

**Evidence:** `survey_mapping/ekf.py`、`survey_mapping/error_state_ins.py`、`scripts/run_mapping_full_stream.py`、轨迹和残差 CSV。

- [ ] 从“IMU 高频但会漂、GNSS 低频但有绝对位置”建立融合直觉。
- [ ] 解释状态、预测、观测、残差、协方差和卡尔曼增益。
- [ ] 逐项解释 6 状态 MVP 和 15 状态误差模型的状态含义。
- [ ] 讲解必要公式，并把每个符号对应到真实物理量。
- [ ] 对照轨迹、GNSS 更新残差和 IMU 零偏输出验证理解。
- [ ] 完成复述题并写入 `docs/learning/05-gnss-ins-fusion.md`，形成 PPT 第 6 页素材。

### Task 6: 理解 LiDAR 建图与姿态图优化

**Evidence:** `scripts/build_lidar_keyframes.py`、`build_lidar_constraints.py`、`optimize_lidar_pose_graph.py`、`build_optimized_cloud.py` 及对应报告。

- [ ] 解释扫描期间运动为什么会使点云弯曲或重影。
- [ ] 解释点级去畸变、关键帧和点云配准。
- [ ] 把姿态图解释为节点、边、测量约束和残差最小化问题。
- [ ] 解释鲁棒核、拒绝约束和错误约束的后果。
- [ ] 对照关键帧、约束、优化报告和轨迹文件验证流程。
- [ ] 完成复述题并写入 `docs/learning/06-lidar-optimization.md`，形成 PPT 第 7–8 页素材。

### Task 7: 理解彩色点云与后处理

**Evidence:** `scripts/colorize_cloud_from_camera.py`、`repair_colorized_geom.py`、`densify_rgb_ply.py`、高密度裁剪脚本和颜色报告。

- [ ] 解释三维点如何通过相机模型找到图像像素并取得 RGB。
- [ ] 解释内参、外参、畸变、遮挡和时间偏差为何会造成错贴色。
- [ ] 解释灰点补色的近邻策略及其限制。
- [ ] 解释漂移块裁剪和局部中点加密，强调二者不会提高测量精度。
- [ ] 对比各候选点云，解释为何推荐 `ColorizedGeom_recolored_dense`。
- [ ] 完成复述题并写入 `docs/learning/07-color-pointcloud.md`，形成 PPT 第 9–11 页素材。

### Task 8: 建立结果评价与局限意识

**Evidence:** 各 `report.json`、结果预览图、点数统计、残差和轨迹文件。

- [ ] 区分定性评价、内部一致性指标和外部绝对精度评价。
- [ ] 整理已成功、部分成功和失败实验及证据。
- [ ] 明确真实内外参、杆臂、PPK、控制点和精度评定方面的不足。
- [ ] 提出按优先级排序的后续工作，并说明每一步解决什么风险。
- [ ] 完成复述题并写入 `docs/learning/08-evaluation-and-limitations.md`，形成 PPT 第 12–14 页素材。

### Task 9: 制作并验证汇报材料

**Inputs:** 八章学习笔记、项目结果图片、已确认的 PPT 页面素材。

- [ ] 汇总老师可能追问的问题到 `docs/learning/defense-questions.md`，每题包含短答、展开答和项目证据。
- [ ] 编写 `docs/learning/presentation-script.md`，将总时长控制在约 13–14 分钟，预留提问或停顿时间。
- [ ] 制作 `HKisland_15min_progress_report.pptx`，控制为 12–14 页并保持一页一个核心结论。
- [ ] 渲染全部页面并检查文字溢出、字体、图片清晰度、对齐和颜色对比度。
- [ ] 逐页核对 PPT 结论是否能追溯到项目证据，删除无法支持的断言。
- [ ] 完成至少两轮模拟汇报：第一轮纠正内容缺口，第二轮控制时间并强化薄弱问题。

## 完成标准

- 学生能在不看稿的情况下，用 3 分钟讲清项目全流程。
- 学生能解释每个核心模块的输入、输出、作用和主要误差来源。
- 学生能正确回答“为什么融合”“为什么去畸变”“为什么优化”“为什么会错贴色”“为什么点更多不等于更准”。
- PPT 页数和演讲时长满足约束，所有核心结论有项目文件或实验结果支持。
- 最终材料明确说明项目局限和 AI 辅助边界。

