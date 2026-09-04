# UAV Detection, Tracking and Sensor Fusion

Generic and sanitized Python framework for studying aerial-target detection,
tracking, state estimation, and multi-sensor fusion.

The project demonstrates:

- Synthetic UAV target generation
- Camera-like bearing measurements
- Radar-like range/bearing measurements
- Measurement noise modeling
- Kalman-based target tracking
- Simple multi-sensor fusion
- Track quality evaluation
- Detection-to-tracking simulation

The public implementation is designed for research and educational use in
autonomous systems, perception, target tracking, and airspace monitoring.

> This repository does not include operational surveillance data, restricted
> sensor parameters, threat-assessment logic, or real-world targeting data.

---

## Architecture

```text
           UAV Target
               |
       +-------+--------+
       |                |
       v                v
 Camera-like        Radar-like
 Measurement        Measurement
       |                |
       +-------+--------+
               |
               v
      Measurement Fusion
               |
               v
        Kalman Tracker
               |
               v
 Estimated Target State
       [x, y, vx, vy]
               |
               v
   Tracking / Error Metrics
```

---

## Main Modules

### 1. Synthetic Target Motion
A maneuvering UAV target is generated in 2-D using a generic kinematic model.

### 2. Camera-Like Measurements
The camera model provides noisy bearing observations.

### 3. Radar-Like Measurements
The radar model provides noisy range and bearing observations.

### 4. Sensor Fusion
Radar and camera information can be fused into a Cartesian position estimate.

### 5. Kalman Tracking
A constant-velocity Kalman filter estimates target position and velocity.

### 6. Evaluation
The framework reports position-estimation RMSE and tracking-error history.

---

## Repository Structure

```text
uav_detection_tracking_sensor_fusion/
├── README.md
├── requirements.txt
├── .gitignore
├── src/
│   ├── detection/
│   │   └── synthetic_sensors.py
│   ├── tracking/
│   │   └── kalman_tracker.py
│   ├── fusion/
│   │   └── sensor_fusion.py
│   └── simulation/
│       └── tracking_simulation.py
├── examples/
│   ├── run_tracking_demo.py
│   └── compare_sensors.py
└── data/
    └── README.md
```

---

## Evaluation Metrics

- Position RMSE
- Mean absolute tracking error
- Final position error
- Sensor-wise tracking comparison
- Fused-estimate performance

---

## Installation

```bash
pip install -r requirements.txt
```

Run the main demo:

```bash
python examples/run_tracking_demo.py
```

Compare sensor configurations:

```bash
python examples/compare_sensors.py
```

---

## Technologies

- Python
- NumPy
- Matplotlib
- Kalman Filtering
- Sensor Fusion
- Target Tracking

---

## Research Areas

- UAV Detection
- Target Tracking
- State Estimation
- Sensor Fusion
- Autonomous Systems
- Airspace Monitoring
- Multi-Sensor Perception

---

## Public Implementation Notice

This repository contains generic and sanitized implementations only.

It intentionally excludes:

- Operational sensor specifications
- Restricted surveillance data
- Real-world target signatures
- Threat-assessment logic
- Platform-specific detection thresholds
- Classified or mission-specific parameters

## Status

Research-oriented educational implementation.
