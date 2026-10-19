# DJI Terra Reference Accuracy Evaluation Design

## Goal

Measure the current pure-survey map against the DJI Terra L1 point cloud in
two distinct ways:

1. Absolute georeferenced agreement without ICP.
2. Relative geometric agreement after a scale-fixed rigid alignment.

The result is an accuracy comparison against DJI Terra, not an independently
surveyed control-point certificate.

## Inputs

### Evaluated map

- Geometry PLY:
  `/home/shuai/cehui/outputs_joint_kfgins_all_livox_deskew_color/cloud_optimized_geometry_livox_deskew.ply`
- Point count: 74,367,651
- Coordinate frame: local ENU
- ENU origin: latitude 22.20785650776563 deg, longitude
  114.26042957568009 deg, ellipsoidal height 5.64432430267334 m

The complete color PLY has identical XYZ records, but the intensity geometry
PLY is the accuracy authority because color coverage must not affect geometry
statistics.

### DJI Terra reference

- Root:
  `E:/文档/DJI/DJITerra/19834212827/香港岛/lidars/terra_point_ply`
- Point blocks: `cloud0.ply` through `cloud7.ply`
- Total point count: 32,980,951
- CRS: EPSG:32650, WGS 84 / UTM zone 50N
- Local SRS origin:
  `(217552.60937532573, 2458384.1928950348, 95.392542085959832)`
- DJI Terra sample distance: 0.1 m
- DJI Terra reported average flight-strip elevation difference: 0.21138 m

The Terra reference is produced by the independently mounted DJI L1 rather
than by color transfer or mapping-fusion in the evaluated project.

## Coordinate Unification

The evaluated ENU points will be transformed through ECEF and geodetic
coordinates into EPSG:32650. The Terra points will be converted from their
local coordinates by adding the SRS origin from `metadata.xml`.

No fitted transform is allowed in the absolute evaluation. The coordinate
conversion preserves scale and uses the stated ellipsoidal heights. The
initial horizontal and vertical offsets implied by the two metadata origins
are recorded in the report.

Before distance computation, the evaluated cloud is cropped to the Terra
bounding box plus a 10 m margin. Points outside the expanded box are reported
as non-overlapping coverage rather than silently discarded.

## Sampling and Spatial Split

Both clouds are reduced to deterministic voxel centroids:

- Coarse registration levels: 2.0 m, 1.0 m, and 0.5 m.
- Final evaluation level: 0.25 m.

The final overlap is divided into 20 m XY tiles. Tiles are assigned by checkerboard
parity:

- Fit tiles: even `(tile_x + tile_y)` parity.
- Holdout tiles: odd parity.

ICP uses fit tiles only. All published relative-accuracy metrics use holdout
tiles only. Absolute metrics are reported for all overlap and separately for
the same holdout tiles so the two modes remain comparable.

## Absolute Evaluation

The metadata-only transform is applied and then frozen. Distances are computed
in both directions:

- Evaluated map to DJI Terra.
- DJI Terra to evaluated map.

For each nearest-neighbor vector, report:

- 3D Euclidean distance.
- Horizontal magnitude `sqrt(dx^2 + dy^2)`.
- Signed and absolute vertical difference `dz`.
- Point-to-plane distance where a valid Terra normal exists.

Metrics include count, mean, MAE, RMSE, bias, standard deviation, P50, P90,
P95, P99, maximum, and coverage within 0.2 m, 0.5 m, 1.0 m, and 2.0 m.

Core robust metrics use correspondences within 2.0 m. The complete untrimmed
distribution and outlier fraction remain in the report, so the threshold does
not conceal gross disagreement.

## Relative Evaluation

Start from the metadata-only transform and estimate one six-degree-of-freedom
rigid transform with scale fixed to 1.0. Registration proceeds coarse to fine
with robust point-to-plane ICP on fit tiles:

- 2.0 m voxel, 8.0 m maximum correspondence.
- 1.0 m voxel, 4.0 m maximum correspondence.
- 0.5 m voxel, 2.0 m maximum correspondence.

Normals are estimated from the Terra reference. Correspondences with invalid
normals or robust residuals beyond the current stage threshold receive zero
weight. The solver must reject reflections and non-rigid scale.

The fitted transform is reported as a 4x4 matrix, XYZ translation, rotation
vector, and roll/pitch/yaw. Relative metrics are then computed on independent
0.25 m holdout tiles using the same symmetric and component-wise definitions
as the absolute mode.

## Quality and Failure Gates

The evaluation stops with an explicit failure if any of these conditions
occur:

- Required Terra blocks or metadata are missing.
- Either CRS cannot be resolved.
- Fewer than 100,000 final overlap voxels exist in either direction.
- Absolute overlap within 2.0 m is below 20 percent.
- ICP produces scale other than 1, a reflection, non-finite coordinates, more
  than 20 m translation, or more than 15 deg rotation from metadata alignment.
- Holdout coverage within 2.0 m is below 50 percent after ICP.

Failures preserve diagnostics and must not be labeled as a successful accuracy
result.

## Outputs

Output root:

`/home/shuai/cehui/outputs_joint_kfgins_all_livox_deskew_color/accuracy_against_dji_terra`

Required artifacts:

- `accuracy_report.json`: machine-readable configuration, transforms, metrics,
  gates, and reference provenance.
- `accuracy_report.md`: concise Chinese results and interpretation.
- `absolute_metrics.csv` and `relative_holdout_metrics.csv`.
- `tile_accuracy.csv`: per-tile coverage, bias, RMSE, and percentiles.
- `metadata_transform.npy` and `relative_icp_transform.npy`.
- `absolute_error_colored.ply` and `relative_error_colored.ply`: deterministic
  sampled evaluated points colored by 3D error.
- `error_histograms.png`, `accuracy_cdf.png`, and `tile_error_heatmap.png`.
- `run_manifest.json`: input sizes, hashes of small metadata/config files,
  software versions, voxel sizes, and random seed.

Original DJI Terra and project PLY files remain read-only and are never
modified.

## Accuracy Interpretation

The primary absolute result is the metadata-only holdout/all-overlap result.
It retains global translation, rotation, datum, and lever-arm disagreement.

The relative result answers a different question: how closely the map shape
matches DJI Terra after removing one global rigid bias. It must always be shown
with the removed transform and cannot replace the absolute result.

Vegetation, water, temporal scene differences, occlusion, and different sensor
viewpoints affect nearest-neighbor cloud distances. Symmetric metrics,
point-to-plane distances, spatial tiles, coverage rates, and robust/untrimmed
statistics are all retained to make those effects visible.

