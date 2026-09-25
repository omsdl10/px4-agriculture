# Simulation reference

## Purpose and scope

This is a deterministic PX4 SITL / Gazebo Sim precision-agriculture scenario. Four X500 multicopters survey separate crop fields, record flight telemetry and environmental truth, and use a simple collision-separation response. It is a simulation dataset and control testbed, not a calibrated representation of a specific real farm or airframe.

## Software and execution environment

| Item | Value |
| --- | --- |
| Host used for verification | Apple Silicon macOS 14.6.1 (arm64) |
| PX4 | `v1.18.0-alpha1-461-ga7a6f4c4d7` |
| Simulator | Gazebo Sim 10.4.0 / SDFormat 16.0.1 |
| Python | 3.14.6 |
| ROS 2 | Not installed or required for this implementation |
| Vehicle interface | PX4 Gazebo bridge and independent MAVLink 2 UDP links |
| Random seed | 42 |

PX4 documents Gazebo multi-vehicle use as Linux-only. The Mac setup is an experimentally verified configuration for the versions above, rather than an upstream support guarantee.

## Coordinate systems and time

- Gazebo world coordinates are **ENU**: east (`x`), north (`y`), up (`z`), in metres.
- Geographic origin: **47.397742° N, 8.545594° E, 488 m AMSL**, heading 0°.
- PX4 local positions are vehicle-local NED coordinates; they are not directly compared across vehicles. Inter-UAV separation uses Gazebo world poses.
- Simulator clock, PX4 boot time, host monotonic time, and host Unix time are recorded separately where available.
- The world uses gravity **0, 0, -9.80665 m/s²**, 0.004 s maximum physics step, and a real-time-factor target of 1.

## Farm layout

The world is **1,000 m × 1,000 m** centred on `(0, 0)`. Terrain is a deterministic gently undulating mesh with a maximum configured amplitude of **3 m**. Low quality uses a 51 × 51 terrain grid (2,601 vertices) and 8 m crop-row spacing.

| Field | Bounds `[xmin, xmax, ymin, ymax]` m | Crop | Declared condition | Density | Soil moisture |
| --- | --- | --- | --- | ---: | ---: |
| A | `[-380, -40, 40, 380]` | Wheat | Healthy | 0.95 | 41 |
| B | `[40, 380, 40, 380]` | Wheat | Water stressed | 0.75 | 17 |
| C | `[-380, -40, -380, -40]` | Maize | Uneven growth | 0.70 | 29 |
| D | `[40, 380, -380, -40]` | Maize | Diseased patches | 0.60 | 24 |

Each field is divided into four synthetic crop-truth regions, for 16 total. Region 3 in fields B and D is deliberately made more stressed. Field D also contains deterministically generated row gaps. The farm includes two 12 m-wide central roads, perimeter fencing with a south-side 100 m entrance, four launch pads, farmhouse, barn, storage, base station, greenhouse, pond, water tank, tractor, orchard, irrigation pipes and sprinklers, a 340 m irrigation channel, electric poles, weather mast, and solar panels.

## Fleet and networking

| Vehicle | PX4 instance | MAV_SYS_ID | Spawn ENU m | Assigned field | Role |
| --- | ---: | ---: | --- | --- | --- |
| UAV1 | 0 | 1 | `[-6, -6, 0.3]` | A | RGB mapping |
| UAV2 | 1 | 2 | `[-2, -6, 0.3]` | B | Crop health |
| UAV3 | 2 | 3 | `[2, -6, 0.3]` | C | Irrigation |
| UAV4 | 3 | 4 | `[6, -6, 0.3]` | D | Anomaly inspection |

Vehicles use receive ports **14540–14543**, PX4 API ports **14580–14583**, and local GCS ports **18570–18573**. Future DDS namespaces are `px4_1` through `px4_4`; no ROS/DDS traffic is required for the current scenario.

## Flight and mission parameters

| Parameter | Value |
| --- | ---: |
| Vehicle model | PX4 X500 |
| Survey altitude | 15 m AGL |
| Commanded survey speed | 5 m/s |
| Survey-lane spacing | 12 m |
| Safety separation threshold | 3 m |
| Telemetry logging rate | 5 Hz |
| Default mission | Full field lawnmower survey |
| Compact regression mission | 20 m × 20 m patch in each assigned field |

The planner creates alternating-end lawnmower lanes inside each field. UAV3 adds irrigation waypoints at `(-100, -22)`, `(0, -22)`, and `(440, 130)` m. UAV4 adds anomaly-inspection waypoints at `(435, -200)` and `(455, -100)` m. Each mission uploads takeoff, speed change, survey waypoints, return-to-launch waypoint, and land commands.

If a world-frame distance drops below 3 m, the controller pauses the higher-numbered vehicle and resumes it after the conflict clears. This is a deterministic hold response, not a distributed avoidance planner.

At 5 m/s, the full 1 km² survey takes well over one hour. A five-minute run records partial coverage; it is not expected to meet the full-route completion check.

## Cameras and vehicle-state sensors

| Sensor | Enabled | Rate | Resolution | Horizontal FOV |
| --- | --- | ---: | --- | ---: |
| Downward RGB camera | Yes | 2 Hz | 320 × 240 | 1.047 rad (60°) |
| Downward depth camera | Yes | 2 Hz | 160 × 120 | 1.047 rad (60°) |
| IMU, magnetometer, barometer, GPS | Native PX4 | Native PX4 rate | — | — |

RGB frames are PNG files. Depth frames are float32 NumPy arrays in metres. Camera manifests record simulation time, host receipt time, nearest pose timestamp, pose delta, ENU pose, resolution, and pixel format. Native high-rate PX4 state is retained in ULog; selected flight state is mirrored to CSV.

## Weather and environment assumptions

The active profile is `normal`:

| Variable | Active value |
| --- | ---: |
| Mean wind speed | 1 m/s |
| Wind direction | 0° ENU |
| Peak gust speed | 1 m/s |
| Gust period | 20 s |
| Temperature | 25 °C |
| Relative humidity | 50% |
| Pressure | 101,325 Pa |
| Sunlight factor | 1.0 |
| Rainfall | 0 |
| Visibility | 10,000 m |

Other selectable profiles are moderate wind (5 m/s, 45°, 8 m/s gust), strong wind (10 m/s, 90°, 15 m/s gust), hot/dry, and humid. Gazebo's wind-effects system applies configured wind dynamics. Temperature, humidity, pressure, sunlight, rainfall, and visibility are recorded metadata; they do not drive additional vehicle or crop physics. The logged `wind_target_speed` is a sinusoidal configured forcing, not an airflow measurement.

Nine static virtual soil probes form a 3 × 3 grid at `x,y ∈ {-240, 0, 240}` m. They provide soil moisture (19–38), temperature (25–30 °C), and pH (6.1–6.8).

## Crop-truth assumptions

Crop Red, Green, Blue, Red Edge, and NIR values are idealised labels, not spectra rendered by a physical multispectral camera. NDVI is calculated as:

`NDVI = (NIR − Red) / (NIR + Red)`

The crop health, density, moisture, water-stress, disease state, spectral values, and NDVI are known ground truth. They support dataset and controller experiments but must not be interpreted as field-calibrated agronomic measurements.

## Outputs

Each run makes `experiments/<UTC-run-id>/` containing:

- `uav1`–`uav4/telemetry.csv`: position, velocity, attitude, IMU values when available, battery values when available, flight mode, mission item, and nearest UAV.
- `ground_truth_poses.csv` and `separation.csv`: Gazebo ENU poses and all six vehicle-pair distances.
- `environment/weather.csv`, `soil.csv`, `crop_ground_truth.csv`, and `synthetic_observations.csv`.
- RGB/depth image data and manifest CSVs when sensors are enabled.
- PX4 `.ulg` logs, mission events/results, shutdown status, analysis metrics, and validation results.

Battery energy remains blank when the PX4 simulated battery stream does not report current; the analysis intentionally does not manufacture energy use.

## Verification boundary

A compact end-to-end run passed all mission items, landing checks, cameras, weather, environment data, ULogs, metrics, and clean shutdown. It recorded 214 RGB and 214 depth frames per UAV, zero dropped frames, and 3.342 m minimum separation. Full-field missions and fixed-duration runs have different completion expectations because their route length exceeds a short test window.
