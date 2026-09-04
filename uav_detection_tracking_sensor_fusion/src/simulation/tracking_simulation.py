import numpy as np

from src.detection.synthetic_sensors import (
    radar_measurement,
    camera_bearing_measurement,
    radar_to_cartesian,
)
from src.fusion.sensor_fusion import angular_weighted_fusion
from src.tracking.kalman_tracker import ConstantVelocityKalmanTracker


def target_trajectory(t):
    """
    Generic maneuvering UAV trajectory.
    """
    x = 120.0 + 12.0 * t
    y = 180.0 + 80.0 * np.sin(0.035 * t)
    return np.array([x, y], dtype=float)


def run_tracking_simulation(
    mode="fused",
    duration=50.0,
    dt=0.1,
    seed=5,
):
    rng = np.random.default_rng(seed)
    sensor_xy = np.array([0.0, 0.0])

    tracker = ConstantVelocityKalmanTracker(
        dt=dt,
        process_var=3.0,
        measurement_var=30.0,
    )

    truth_log = []
    measurement_log = []
    estimate_log = []
    error_log = []

    times = np.arange(0.0, duration + dt, dt)

    for t in times:
        truth = target_trajectory(t)

        radar_range, radar_bearing = radar_measurement(
            truth,
            sensor_xy=sensor_xy,
            rng=rng,
        )
        camera_bearing = camera_bearing_measurement(
            truth,
            sensor_xy=sensor_xy,
            rng=rng,
        )

        if mode == "radar":
            measurement = radar_to_cartesian(
                radar_range,
                radar_bearing,
                sensor_xy,
            )
        elif mode == "fused":
            measurement, _ = angular_weighted_fusion(
                radar_range,
                radar_bearing,
                camera_bearing,
                sensor_xy=sensor_xy,
            )
        else:
            raise ValueError("mode must be 'radar' or 'fused'")

        estimate = tracker.step(measurement)

        truth_log.append(truth)
        measurement_log.append(measurement)
        estimate_log.append(estimate[:2])
        error_log.append(np.linalg.norm(estimate[:2] - truth))

    error = np.asarray(error_log)

    metrics = {
        "rmse_position_m": float(np.sqrt(np.mean(error**2))),
        "mean_position_error_m": float(np.mean(error)),
        "final_position_error_m": float(error[-1]),
    }

    return {
        "time": times,
        "truth": np.asarray(truth_log),
        "measurements": np.asarray(measurement_log),
        "estimates": np.asarray(estimate_log),
        "error": error,
        "metrics": metrics,
    }
