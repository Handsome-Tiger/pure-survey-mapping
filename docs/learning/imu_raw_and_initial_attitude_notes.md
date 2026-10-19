# IMU Raw Data and Initial Attitude Notes

## 1. Current Data Situation

In the current dataset, the IMU topic only provides raw measurements:

- angular velocity
- linear acceleration

The `orientation` field is invalid, shown as an all-zero quaternion. This means the IMU driver or upstream system does not provide a ready-to-use attitude estimate.

Therefore, the project cannot directly use the IMU message's `orientation` field as the drone attitude.

## 2. Does Invalid Orientation Mean the Project Cannot Continue?

No. Invalid `orientation` does not mean the project must stop.

It means the IMU should be treated as a raw inertial measurement source rather than a direct pose source.

Raw IMU data can still be used for high-frequency motion prediction:

- Angular velocity can be integrated to estimate attitude change.
- Linear acceleration can be used to predict velocity and position after gravity compensation.

However, raw IMU integration cannot provide a reliable long-term trajectory by itself, because IMU noise and bias accumulate over time.

In this project, the reasonable role of raw IMU data is:

> IMU raw data provides high-frequency short-term prediction, while GNSS and LiDAR constraints are needed to correct long-term drift.

## 3. Why Can We Not Use Only IMU?

In theory:

```text
angular velocity integration -> attitude
acceleration integration -> velocity
velocity integration -> position
```

In practice, this is unstable for long-term navigation.

The main reasons are:

- Gyroscope bias causes attitude drift.
- Accelerometer bias is amplified by double integration.
- Acceleration measurements include gravity.
- Accurate gravity removal requires accurate attitude.
- Small IMU errors can quickly grow into large position errors.

Therefore, IMU alone can provide short-term continuity, but it cannot act as a stable absolute pose source.

## 4. Why GNSS Is Still Needed

GNSS provides low-frequency but global position observations. It can be used to correct the accumulated drift from IMU integration.

A simple explanation is:

> IMU predicts the motion at high frequency, and GNSS periodically pulls the predicted trajectory back toward the global position observation.

This is the basic intuition behind GNSS/INS fusion.

In a filter-based system:

- IMU is used in the prediction step.
- GNSS is used in the correction/update step.

This combines the strengths of both sensors:

- IMU: high frequency, continuous, good for short-term motion.
- GNSS: global position constraint, useful for suppressing long-term drift.

## 5. What If There Is No Initial Attitude?

The absence of initial attitude is an important limitation, but it is not necessarily fatal.

Initial attitude includes:

- roll
- pitch
- yaw

The difficulty is different for each component.

Roll and pitch can often be estimated from the gravity direction if the system has a short static segment at the beginning. When the drone is static, the accelerometer mainly observes gravity, so the system can infer whether the body is tilted.

Yaw is much harder. A single ordinary IMU cannot determine yaw from gravity while static, because gravity only gives the vertical direction. It does not tell whether the drone is facing east, west, north, or south.

Therefore:

- roll/pitch may be initialized from static accelerometer data.
- yaw needs extra information or an assumption.

Possible yaw sources include:

- GNSS course over ground after the drone starts moving.
- LiDAR geometric matching.
- magnetometer or flight controller attitude, if available.
- dual-antenna GNSS, if available.
- manual initial yaw assumption.

## 6. Can the Current Project Still Be Done?

Yes, but it should be positioned as a simplified MVP or a system with known limitations.

If the dataset contains:

- raw IMU,
- GNSS position,
- LiDAR point cloud,
- usable timestamps,
- approximate extrinsic calibration,

then the project can still proceed as a GNSS/IMU/LiDAR mapping pipeline.

However, because there is no direct attitude topic and no reliable initial attitude, the system should not be described as a fully validated high-precision GNSS/INS solution.

A more accurate project statement is:

> The current system uses raw IMU data for high-frequency motion prediction, GNSS for global position correction, and LiDAR constraints for geometric refinement. Since the IMU orientation field is invalid and the initial attitude is not directly available, the current implementation should be regarded as a simplified multisensor mapping MVP. Initial attitude estimation, yaw initialization, sensor calibration, and accuracy validation remain important future work.

## 7. Suggested Explanation for Group Meeting

The following paragraph can be used in the presentation:

> In the current dataset, the IMU topic only provides raw angular velocity and linear acceleration, while the orientation field is invalid. Therefore, the IMU cannot be directly used as a pose source. Instead, raw IMU data is used for high-frequency inertial prediction. Since pure IMU integration suffers from bias and drift, GNSS position observations are needed to periodically correct the trajectory, and LiDAR geometric constraints are further used to improve mapping consistency. In addition, because there is no direct initial attitude, roll and pitch may be estimated from gravity if a static segment exists, while yaw requires external information such as GNSS motion direction, LiDAR matching, flight controller attitude, or an assumed initial value. Therefore, the current project can continue as a simplified multisensor mapping MVP, but strict attitude initialization and survey-grade accuracy validation remain future work.

## 8. Possible Teacher Questions and Answers

Question: If the IMU orientation field is all zeros, can the IMU still be used?

Answer:

> Yes. The invalid orientation field only means there is no ready-to-use attitude output. The raw angular velocity and acceleration can still be used for inertial prediction, but the predicted state must be corrected by GNSS, LiDAR, or other observations.

Question: Can raw IMU data alone estimate the full trajectory?

Answer:

> Only for a very short time. In theory, angular velocity and acceleration can be integrated to estimate attitude, velocity, and position, but bias and noise accumulate quickly. Therefore, IMU alone cannot provide a reliable long-term trajectory.

Question: Why is initial attitude important?

Answer:

> Initial attitude defines the starting orientation of the body frame relative to the map frame. Without it, IMU integration does not know the correct initial direction. Roll and pitch can often be estimated from gravity during a static segment, but yaw cannot be determined by a single static IMU alone.

Question: Does the absence of initial attitude mean the project is impossible?

Answer:

> Not necessarily. The project can still proceed with additional assumptions or external constraints. However, the system should be described as a simplified MVP, and yaw initialization plus accuracy validation should be listed as future work.

Question: What is the relationship between IMU and GNSS in this project?

Answer:

> IMU provides high-frequency short-term prediction, while GNSS provides low-frequency global correction. In simple terms, IMU predicts continuously, and GNSS periodically corrects the accumulated drift.

