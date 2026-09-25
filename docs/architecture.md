# Architecture and phase gates

This project targets the user's native Apple Silicon macOS environment experimentally.
PX4 officially supports Gazebo multi-vehicle simulation only on Linux:
https://docs.px4.io/main/en/sim_gazebo_gz/multi_vehicle_simulation

Inspected 2026-09-25:
- macOS 14.6.1, arm64
- PX4 source: v1.18.0-alpha1-461-ga7a6f4c4d7, existing ARM64 SITL binary
- PX4 checkout has existing untracked runtime files and a dirty Gazebo submodule; it is not modified by this project.
- Gazebo Sim 10.4.0; SDFormat 16.0.1
- Python 3.14.6; CMake 4.3.4; Ninja 1.13.2
- ROS 2 not found on PATH or /opt/ros; Docker CLI present, engine unavailable

Native architecture: Gazebo Sim -> PX4 gz_bridge -> independent MAVLink UDP links -> Python fleet controller and logger. Gazebo Transport provides simulation clock, world poses and sensor images. PX4 ULog preserves native high-frequency sensor and estimator data. No deprecated microRTPS bridge. No ROS bag is claimed when ROS 2 is absent.

Instances 0..3 use MAV_SYS_ID 1..4, API receive ports 14540..14543, PX4 API ports 14580..14583, and GCS local ports 18570..18573. Shared GCS destination 14550 is deliberate MAVLink multiplexing. DDS namespace environment variables px4_1..px4_4 are assigned for future XRCE-DDS use, but no ROS communication is required for this native implementation.

World frame: east/north/up in meters, geographic origin 47.397742 N / 8.545594 E / 488 m AMSL. PX4 local positions are north/east/down relative to each vehicle's estimator origin. They must never be directly subtracted across vehicles; world poses or translated origins are required for separation. Sensor frame timestamps and simulation clock must be retained separately from host receipt timestamps.

Development gates: (1) valid SDF and live physics world; (2) one vehicle takeoff and landing; (3) four unique working instances; (4) mission execution; (5) sensors; (6) weather physics; (7) soil/crop truth; (8) synchronized logging; (9) separation warning; (10) analysis. A test is not passed merely because files exist.
