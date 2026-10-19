# 项目最终摘要

当前正式成果为：

```text
outputs_joint_kfgins_all_livox_deskew_color/cloud_optimized_color_complete.ply
```

它由纯 GNSS/INS/Livox/相机流水线生成，不依赖 FAST-LIO、FAST-LIVO2、mapping-fusion、DJI L1 颜色迁移或三维近邻补色。完整点数 74,367,651，真实相机 RGB 点数 73,125,606，颜色覆盖率 98.3299%；文件 SHA-256 为 `53a7a670872537084d052074736981c39e2d3d2f9142979d0904082b923d335b`。

正式入口、输入要求和验收方式分别见：

- [README.md](README.md)
- [docs/PRODUCTION_PIPELINE_zh-CN.md](docs/PRODUCTION_PIPELINE_zh-CN.md)
- [docs/INPUT_DATA_FORMATS_zh-CN.md](docs/INPUT_DATA_FORMATS_zh-CN.md)
- [docs/RESULT_MANIFEST_zh-CN.md](docs/RESULT_MANIFEST_zh-CN.md)

历史实验方案仍保留在 `docs/`，但不应覆盖上述正式基线。

