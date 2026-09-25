# PX4 precision-agriculture multi-UAV simulation

This project runs a configurable 1 km × 1 km farm in Gazebo Sim with four independent PX4 X500 vehicles. Each vehicle receives a deterministic agricultural survey, records native PX4 ULog and CSV telemetry, and can carry a downward RGB and depth camera. The experiment logger records Gazebo world poses, weather, nine soil probes, synthetic crop spectral truth and NDVI, inter-UAV separation, and synchronized image manifests.

The implementation was built and exercised natively on Apple Silicon macOS. PX4 officially documents Gazebo multi-vehicle simulation as Linux-only, so this macOS route is an experimentally verified configuration for the versions in [docs/environment.json](docs/environment.json), rather than an upstream-supported platform guarantee.

## Verified host

- macOS 14.6.1 arm64
- PX4 `v1.18.0-alpha1-461-ga7a6f4c4d7`
- Gazebo Sim 10.4.0 / SDFormat 16.0.1
- Python 3.14.6, CMake 4.3.4, Ninja 1.13.2
- ROS 2 is not installed on this host

The native design uses PX4's Gazebo bridge plus separate MAVLink 2 links for control and logging. ROS 2 is therefore not required and no ROS bag is claimed. PX4's IMU, GPS, barometer, magnetometer, estimator and actuator data remain in each `.ulg`; selected lower-rate fields are also written to CSV. The future DDS namespace assignments are `px4_1` through `px4_4` and avoid deprecated microRTPS architecture.

## Setup

The project expects a compiled PX4 checkout at `~/PX4-Autopilot`. Override that with `PX4_ROOT=/path/to/PX4-Autopilot` when needed.

```bash
./launch/setup_macos.sh
```

The setup script checks local tools, creates `.venv`, installs the pinned Python dependencies, and writes the inspected version manifest. It does not modify the PX4 checkout.

## Run

One command starts the complete graphical scenario: the farm, four sensor-equipped PX4 vehicles, physical wind, environment logger, and full field missions.

```bash
./launch/start_simulation.sh
```

A full survey crosses all four large fields and can take well over an hour at the default 5 m/s. Use the compact acceptance mission for a much shorter end-to-end run:

```bash
./launch/start_simulation.sh --mission compact
```

Useful variants:

```bash
./launch/start_simulation.sh --headless --mission compact
./launch/start_simulation.sh --world-only
./launch/start_simulation.sh --no-sensors --no-wind --mission compact
./launch/start_simulation.sh --count 1 --mission compact
```

Stop any run with Ctrl-C in its launcher terminal or from another terminal:

```bash
./launch/stop_simulation.sh
```

The supervisor signals only process groups it created, lets the mission attempt a controlled recovery, closes the recorder, copies ULogs, runs analysis and acceptance checks, and removes its runtime state. It does not broadly kill unrelated PX4 or Gazebo processes.

## Configuration

- [simulation.yaml](config/simulation.yaml): map scale, seed, quality, altitude, velocity, separation and output rates
- [drones.yaml](config/drones.yaml): instances, system IDs, launch positions, roles and assigned fields
- [fields.yaml](config/fields.yaml): field bounds, crop state and idealized spectral values
- [weather.yaml](config/weather.yaml): normal, moderate-wind, strong-wind, hot/dry and humid profiles
- [sensors.yaml](config/sensors.yaml): camera switches, resolution and update rate
- [soil.yaml](config/soil.yaml): nine soil-probe positions and values

Set `quality` to `low`, `medium`, or `high`. This changes terrain and crop-row density. Cameras can be disabled on the command line, and their rates and sizes can be reduced independently. The fixed `random_seed` controls generated crop gaps and Gazebo randomness.

Weather separates physics from metadata. `wind_speed`, direction and sinusoidal gust amplitude configure Gazebo's wind-effects system when `--wind` is active. Temperature, humidity, pressure, sunlight, rainfall and visibility are recorded metadata. They do not affect vehicle physics. The weather CSV calls the time-varying field `wind_target_speed` because it is the configured forcing, not a measured airflow sample.

The Red, Green, Blue, Red Edge and NIR values are synthetic region labels. NDVI is `(NIR - Red) / (NIR + Red)`. These data are ground truth for dataset experiments; the renderer does not perform physically based spectral simulation.

## Output

Each run creates a UTC-named folder under `experiments/` containing:

```text
metadata.json                 configuration/run identity
environment/weather.csv      weather metadata and wind target
environment/soil.csv         nine virtual soil probes
environment/crop_ground_truth.csv
environment/synthetic_observations.csv
ground_truth_poses.csv        Gazebo ENU poses
separation.csv                all six UAV pairs and warnings
uav1..uav4/telemetry.csv
uav1..uav4/camera/*.png       RGB images
uav1..uav4/depth/*.npy        float32 depth in meters
uav1..uav4/*/manifest.csv     simulation time and associated pose
uav1..uav4/ulog/**/*.ulg      native PX4 logs
analysis/metrics.json
analysis/summary.png
acceptance.json
```

The image manifests retain Gazebo simulation time, host receipt time, closest world-pose time and pose delta. A missing pose is written as empty fields rather than fabricated. PX4 CSV rows include PX4 boot time, host monotonic and Unix time, and the closest current Gazebo simulation time. Raw high-rate sensor values remain in ULog.

## Safety and extensibility

The logger computes all pairwise distances from Gazebo's shared ENU world frame. It rejects stale or poorly aligned samples. Below the configured threshold, the mission controller pauses the higher-numbered UAV and resumes it after the conflict clears. This deterministic hold response is intentionally simple; the planner and safety monitor are separate modules so an RL policy, task allocator, velocity-obstacle method or distributed coordinator can replace them.

`scripts/planner.py` is the policy boundary for future AI controllers. Crop truth and observations are independent of mission control. Communication simulation can later sit between controller and MAVLink/DDS without changing the farm model or dataset schema.

Run unit checks with:

```bash
.venv/bin/python -m unittest discover -s tests -v
```

Analyze any completed run again with:

```bash
.venv/bin/python analysis/analyze_experiment.py experiments/<experiment-id>
.venv/bin/python scripts/validate_experiment.py experiments/<experiment-id>
```

See [architecture.md](docs/architecture.md) for frame, port, namespace and phase-gate details, and [acceptance-tests.md](docs/acceptance-tests.md) for the verified results and remaining platform limits.
