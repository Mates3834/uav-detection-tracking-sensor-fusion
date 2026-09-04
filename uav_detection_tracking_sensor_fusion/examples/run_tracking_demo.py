import matplotlib.pyplot as plt

from src.simulation.tracking_simulation import run_tracking_simulation


result = run_tracking_simulation(mode="fused")

print("Tracking metrics")
for key, value in result["metrics"].items():
    print(f"{key}: {value:.3f}")

truth = result["truth"]
meas = result["measurements"]
est = result["estimates"]

plt.figure()
plt.plot(truth[:, 0], truth[:, 1], label="True UAV trajectory")
plt.scatter(
    meas[::8, 0],
    meas[::8, 1],
    s=12,
    alpha=0.6,
    label="Fused measurements",
)
plt.plot(est[:, 0], est[:, 1], linestyle="--", label="Kalman estimate")
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("UAV Detection, Tracking and Sensor Fusion")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()

plt.figure()
plt.plot(result["time"], result["error"])
plt.xlabel("Time [s]")
plt.ylabel("Position error [m]")
plt.title("Tracking Error")
plt.grid(True)
plt.show()
