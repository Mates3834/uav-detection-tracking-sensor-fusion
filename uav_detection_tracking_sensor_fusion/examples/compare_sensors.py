from src.simulation.tracking_simulation import run_tracking_simulation


for mode in ("radar", "fused"):
    result = run_tracking_simulation(mode=mode, seed=5)

    print(f"\n{mode.upper()} MODE")
    for key, value in result["metrics"].items():
        print(f"{key}: {value:.3f}")
