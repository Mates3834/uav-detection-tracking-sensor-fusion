import numpy as np


def wrap_angle(angle):
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def radar_measurement(target_xy, sensor_xy=(0.0, 0.0),
                      range_std=6.0, bearing_std_deg=1.2, rng=None):
    """
    Simulate a generic radar-like range/bearing observation.
    """
    rng = np.random.default_rng() if rng is None else rng
    target_xy = np.asarray(target_xy, dtype=float)
    sensor_xy = np.asarray(sensor_xy, dtype=float)

    delta = target_xy - sensor_xy
    true_range = np.linalg.norm(delta)
    true_bearing = np.arctan2(delta[1], delta[0])

    measured_range = true_range + rng.normal(0.0, range_std)
    measured_bearing = wrap_angle(
        true_bearing + rng.normal(0.0, np.deg2rad(bearing_std_deg))
    )

    return float(measured_range), float(measured_bearing)


def camera_bearing_measurement(target_xy, sensor_xy=(0.0, 0.0),
                               bearing_std_deg=0.6, rng=None):
    """
    Simulate a generic camera-like angular measurement.
    """
    rng = np.random.default_rng() if rng is None else rng
    target_xy = np.asarray(target_xy, dtype=float)
    sensor_xy = np.asarray(sensor_xy, dtype=float)

    delta = target_xy - sensor_xy
    true_bearing = np.arctan2(delta[1], delta[0])

    measured_bearing = wrap_angle(
        true_bearing + rng.normal(0.0, np.deg2rad(bearing_std_deg))
    )
    return float(measured_bearing)


def radar_to_cartesian(range_value, bearing, sensor_xy=(0.0, 0.0)):
    sensor_xy = np.asarray(sensor_xy, dtype=float)
    return sensor_xy + np.array([
        range_value * np.cos(bearing),
        range_value * np.sin(bearing),
    ])
