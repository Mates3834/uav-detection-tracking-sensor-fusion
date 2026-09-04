import numpy as np


def angular_weighted_fusion(radar_range, radar_bearing, camera_bearing,
                            sensor_xy=(0.0, 0.0),
                            radar_weight=0.35, camera_weight=0.65):
    """
    Fuse radar range with radar/camera bearing using a weighted circular mean.

    This compact educational fusion block is intentionally simple.
    """
    radar_weight = float(radar_weight)
    camera_weight = float(camera_weight)

    sx = (
        radar_weight * np.cos(radar_bearing)
        + camera_weight * np.cos(camera_bearing)
    )
    sy = (
        radar_weight * np.sin(radar_bearing)
        + camera_weight * np.sin(camera_bearing)
    )
    fused_bearing = np.arctan2(sy, sx)

    sensor_xy = np.asarray(sensor_xy, dtype=float)
    fused_xy = sensor_xy + np.array([
        radar_range * np.cos(fused_bearing),
        radar_range * np.sin(fused_bearing),
    ])

    return fused_xy, float(fused_bearing)
