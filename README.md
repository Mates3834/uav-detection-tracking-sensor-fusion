# UAV Detection, Tracking and Sensor Fusion

A modular simulation framework for **UAV detection, multi-sensor perception, target-state estimation, and tracking** using radar-like and camera-like measurements.

The project investigates how heterogeneous sensor information can be combined to improve the estimation of a maneuvering aerial target. A synthetic UAV trajectory is observed using noisy **radar range/bearing measurements** and **camera bearing measurements**. The measurements are combined through a lightweight sensor-fusion layer and processed using a **Kalman-based target tracker**.

The framework is designed as a research-oriented implementation for studies in **UAV perception, target tracking, sensor fusion, state estimation, autonomous systems, and airspace monitoring**.

> **Note:** The public implementation uses synthetic target trajectories and generic sensor models. It does not contain operational surveillance data, platform-specific sensor specifications, threat-assessment logic, or restricted information.

---

## System Architecture

The framework follows a modular detection-to-tracking architecture:

```text
                  UAV Target
                      |
              True Target State
                      |
          +-----------+-----------+
          |                       |
          v                       v
   Radar-Like Sensor        Camera-Like Sensor
   Range + Bearing               Bearing
          |                       |
          v                       v
    Radar Measurement      Camera Measurement
          |                       |
          +-----------+-----------+
                      |
                      v
             Sensor Fusion
                      |
                      v
          Cartesian Measurement
                      |
                      v
              Kalman Filter
                      |
                      v
          Estimated Target State
             [x, y, vx, vy]
                      |
                      v
             Tracking Metrics
```

The architecture separates **measurement generation, sensor fusion, state estimation, and performance evaluation**, allowing each component to be modified independently.

---

## 1. UAV Target Model

A synthetic maneuvering UAV trajectory is generated to evaluate the tracking framework.

The target position is represented by

```text
p_target = [x, y]^T
```

and follows a smooth time-varying trajectory.

The public simulation uses a generic trajectory of the form

```text
x(t) = x0 + Vx t
```

```text
y(t) = y0 + A sin(omega t)
```

which produces a target moving forward while performing a smooth lateral maneuver.

This provides a simple but useful scenario for evaluating state estimation under non-constant target motion.

---

## 2. Radar-Like Measurement Model

The first sensor produces generic **range and bearing measurements**.

For a sensor located at

```text
p_s = [x_s, y_s]^T
```

and a target located at

```text
p_t = [x_t, y_t]^T
```

the relative position is

```text
Delta p = p_t - p_s
```

The ideal range measurement is

```text
r = sqrt(
    (x_t - x_s)^2 +
    (y_t - y_s)^2
)
```

and the bearing is

```text
theta =
atan2(
    y_t - y_s,
    x_t - x_s
)
```

Measurement noise is added to both quantities:

```text
r_measured = r + n_r
```

```text
theta_measured = theta + n_theta
```

where `n_r` and `n_theta` represent synthetic sensor noise.

---

## 3. Camera-Like Bearing Measurement

The second sensor represents a simplified vision-based angular measurement.

Unlike the radar-like sensor, the camera-like model provides only target bearing:

```text
theta_camera =
atan2(
    y_t - y_s,
    x_t - x_s
) + n_camera
```

This represents a simplified angular observation that could conceptually originate from an image-based detector.

The current public implementation does **not** perform real image detection. Instead, it focuses on the estimation and fusion layer following detection.

This distinction is important:

```text
Current implementation:

Synthetic Target
      ↓
Synthetic Sensor Measurements
      ↓
Fusion
      ↓
Tracking

Possible future implementation:

RGB / Thermal Image
      ↓
UAV Detector
      ↓
Bounding Box / Bearing
      ↓
Fusion
      ↓
Tracking
```

---

## 4. Radar Measurement Conversion

Radar measurements are converted from polar coordinates

```text
[r, theta]
```

to Cartesian coordinates

```text
[x, y]
```

using

```text
x = x_s + r cos(theta)
```

```text
y = y_s + r sin(theta)
```

The resulting Cartesian measurement can then be processed by the target-state estimator.

---

## 5. Multi-Sensor Fusion

The project includes a lightweight radar-camera fusion method.

Radar provides:

```text
Range
+
Bearing
```

while the camera-like sensor provides:

```text
Bearing
```

The angular measurements are combined using a weighted circular mean.

Conceptually,

```text
Radar Bearing ----\
                   \
                    >---- Fused Bearing
                   /
Camera Bearing ---/
```

The fused bearing is then combined with the radar range to produce a fused Cartesian target-position measurement.

The public implementation intentionally uses a simple fusion architecture so that the complete perception-to-tracking pipeline remains easy to understand and extend.

---

## 6. Kalman-Based Target Tracking

The fused measurements are processed by a discrete **constant-velocity Kalman filter**.

The estimated target state is

```text
x_hat =
[x, y, vx, vy]^T
```

where

```text
x, y   = estimated target position
vx, vy = estimated target velocity
```

The discrete state model is

```text
x_k = F x_(k-1) + w_k
```

with

```text
F =
[1  0  dt  0 ]
[0  1  0   dt]
[0  0  1   0 ]
[0  0  0   1 ]
```

and measurement model

```text
z_k = H x_k + v_k
```

where

```text
H =
[1  0  0  0]
[0  1  0  0]
```

---

## 7. Kalman Prediction

The state prediction is calculated as

```text
x_hat(k|k-1) =
F x_hat(k-1|k-1)
```

and covariance prediction as

```text
P(k|k-1) =
F P(k-1|k-1) F^T + Q
```

where `Q` represents process uncertainty.

---

## 8. Kalman Measurement Update

When a new fused measurement becomes available, the innovation is

```text
y_k =
z_k - H x_hat(k|k-1)
```

The innovation covariance is

```text
S_k =
H P(k|k-1) H^T + R
```

and the Kalman gain becomes

```text
K_k =
P(k|k-1) H^T S_k^-1
```

The estimated state is updated using

```text
x_hat(k|k) =
x_hat(k|k-1) + K_k y_k
```

and the covariance becomes

```text
P(k|k) =
(I - K_k H) P(k|k-1)
```

This recursive architecture enables the target trajectory to be reconstructed from noisy measurements.

---

## 9. Detection-to-Tracking Pipeline

The complete conceptual pipeline can be represented as

```text
Target
   ↓
Detection / Measurement
   ↓
Sensor Observation
   ↓
Measurement Fusion
   ↓
State Estimation
   ↓
Target Track
```

In the current public implementation, the detection stage is represented using synthetic sensor observations.

This allows the tracking and fusion algorithms to be evaluated independently from a particular computer-vision or radar-detection algorithm.

---

## 10. Radar-Only Tracking

The first evaluation configuration uses only radar-like measurements:

```text
UAV Target
    ↓
Radar Range + Bearing
    ↓
Polar-to-Cartesian Conversion
    ↓
Kalman Filter
    ↓
Target Track
```

This configuration provides a baseline for evaluating the benefit of additional sensor information.

---

## 11. Radar-Camera Fusion Tracking

The second configuration combines radar-like and camera-like observations:

```text
                  UAV Target
                      |
             +--------+--------+
             |                 |
             v                 v
           Radar             Camera
       Range/Bearing         Bearing
             |                 |
             +--------+--------+
                      |
                      v
               Sensor Fusion
                      |
                      v
                Kalman Filter
                      |
                      v
                 UAV Track
```

The same target trajectory and random seed can be used for both configurations to support a consistent comparison.

---

## 12. Evaluation Metrics

The framework calculates several target-tracking metrics.

### Position RMSE

The position error is

```text
e_k =
||p_hat_k - p_k||
```

and the root-mean-square position error is

```text
RMSE =
sqrt(
    (1/N) *
    sum(e_k^2)
)
```

---

### Mean Position Error

```text
e_mean =
(1/N) *
sum(e_k)
```

provides the average target-position estimation error.

---

### Final Position Error

```text
e_final =
||p_hat_N - p_N||
```

shows the estimation accuracy at the end of the simulation.

---

## 13. Comparative Evaluation

The included example allows two configurations to be compared:

| Configuration | Range | Bearing | Multi-Sensor Fusion | Kalman Tracking |
|---|---:|---:|---:|---:|
| Radar Only | ✓ | ✓ | — | ✓ |
| Radar + Camera | ✓ | ✓ + camera bearing | ✓ | ✓ |

This provides a basic framework for investigating whether additional angular information improves target-state estimation.

---

## 14. Repository Structure

```text
uav_detection_tracking_sensor_fusion/
│
├── README.md
├── requirements.txt
├── .gitignore
│
├── src/
│   │
│   ├── detection/
│   │   ├── __init__.py
│   │   └── synthetic_sensors.py
│   │
│   ├── tracking/
│   │   ├── __init__.py
│   │   └── kalman_tracker.py
│   │
│   ├── fusion/
│   │   ├── __init__.py
│   │   └── sensor_fusion.py
│   │
│   └── simulation/
│       ├── __init__.py
│       └── tracking_simulation.py
│
├── examples/
│   ├── run_tracking_demo.py
│   └── compare_sensors.py
│
└── data/
    └── README.md
```

---

## 15. Module Description

| Module | Purpose |
|---|---|
| `synthetic_sensors.py` | Radar-like and camera-like measurement generation |
| `sensor_fusion.py` | Radar-camera angular measurement fusion |
| `kalman_tracker.py` | Constant-velocity Kalman target tracker |
| `tracking_simulation.py` | Integrated target tracking simulation |
| `run_tracking_demo.py` | Main visualization and tracking demonstration |
| `compare_sensors.py` | Radar-only vs fused-sensor comparison |

---

## 16. Running the Project

Install the dependencies:

```bash
pip install -r requirements.txt
```

Run the main sensor-fusion tracking demonstration:

```bash
python examples/run_tracking_demo.py
```

Run the sensor comparison:

```bash
python examples/compare_sensors.py
```

---

## 17. Example Outputs

The project can generate several useful visualization and evaluation outputs:

```text
results/
├── target_tracking.png
├── tracking_error.png
├── radar_vs_fusion.png
└── kalman_estimation.png
```

Recommended figures for the repository are:

### Target Tracking

```text
True UAV trajectory
        +
Sensor measurements
        +
Kalman estimated trajectory
```

### Tracking Error

```text
Position Error [m]
        |
        |\
        | \___
        |     \____
        +--------------> Time
```

### Sensor Comparison

```text
Radar Only
     vs.
Radar + Camera Fusion
```

These three figures would be enough to make the repository visually much stronger.

---

## Technologies

- Python
- NumPy
- Matplotlib
- Kalman Filtering
- Multi-Sensor Fusion
- Target Tracking
- State Estimation

---

## Research Areas

The project is related to:

- UAV Detection
- UAV Tracking
- Target-State Estimation
- Sensor Fusion
- Kalman Filtering
- Autonomous Systems
- Multi-Sensor Perception
- Airspace Monitoring
- Computer Vision Integration
- Guidance, Navigation and Control

---

## Project Motivation

Reliable autonomous systems require more than detecting an object at a single instant.

A complete perception architecture should maintain an estimate of the object's motion over time:

```text
Detection
    ↓
Measurement
    ↓
Tracking
    ↓
State Estimation
    ↓
Prediction
```

Combining heterogeneous sensors can improve robustness because different sensing modalities provide complementary information.

In this framework:

```text
Radar
  → range + bearing

Camera
  → bearing

Kalman Filter
  → position + velocity estimate
```

The project therefore provides a modular foundation for more advanced **multi-sensor UAV tracking and autonomous airspace-monitoring research**.

---

## Future Extensions

The current architecture can be extended with:

- Real RGB UAV detection
- Thermal-camera detection
- YOLO-based UAV detection
- Image-based bearing extraction
- Extended Kalman Filter (EKF)
- Unscented Kalman Filter (UKF)
- Nonlinear radar measurement models
- Radar-camera calibration
- Track-to-track fusion
- Multiple-target tracking
- Data association
- Multiple Hypothesis Tracking
- Joint Probabilistic Data Association
- 3-D target tracking
- Target-motion prediction
- Real-time video processing

A future version could therefore implement:

```text
RGB / Thermal Camera
          |
          v
      UAV Detector
          |
          v
    Image Tracking
          |
          +----------------+
                           |
Radar Detection ----------+
                           |
                           v
                    Sensor Fusion
                           |
                           v
                    Kalman / EKF
                           |
                           v
                    Target Track
```

---

## Public Implementation Notice

The source code in this repository contains **generic and sanitized implementations** intended to demonstrate the underlying perception, state-estimation, and sensor-fusion concepts.

The public version intentionally excludes:

- Operational radar specifications
- Platform-specific sensor characteristics
- Restricted surveillance data
- Real-world target signatures
- Operational detection thresholds
- Threat classification
- Threat-assessment logic
- Engagement logic
- Restricted airspace information
- Unpublished datasets
- Sensitive implementation parameters

The repository should therefore be interpreted as a **research and educational UAV detection, tracking, and sensor-fusion framework** rather than an operational surveillance system.

---

## Status

**Research-oriented project / active development**

The current implementation focuses on synthetic radar-camera measurements, sensor fusion, and Kalman-based target tracking.

Integration of real computer-vision detection and more advanced nonlinear/multi-target tracking algorithms is considered future work.

---

## Author

**Mehmet Ateş**

Research interests:

- Autonomous Systems
- UAV Autonomy
- Guidance, Navigation and Control (GNC)
- Target Tracking
- State Estimation
- Sensor Fusion
- Computer Vision
- Path Planning
- Reinforcement Learning
- Marine and Aerial Robotics
