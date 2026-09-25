# Acceptance test record

Test host and component revisions are recorded in `environment.json`. These tests were performed on 2026-09-25.

| Phase | Result | Evidence |
|---|---|---|
| Farm world | Passed | SDFormat validation and 100 live physics iterations; terrain normals were corrected after the first DART failure |
| One PX4 UAV | Passed | System ID 1 armed normally, climbed to 8.16 m, landed and disarmed; `phase2-result.json` |
| Four PX4 UAVs | Passed | IDs 1–4 independently armed, climbed above 5 m, landed and disarmed; `phase3-result.json` |
| Deterministic missions | Passed | Four compact field surveys reached all planned sequence numbers, returned, landed and disarmed; `phase4-result.json` |
| RGB camera | Passed | Native Gazebo transport produced timestamped PNG frames with matching world poses |
| Depth camera | Passed | Native Gazebo transport produced float32 depth arrays with matching world poses |
| Recorder shutdown | Passed | Corrected subscriptions were explicitly removed; isolated RGB/depth run exited with code 0, zero drops and zero errors |
| Unit checks | Passed | Planner, mission frames, IDs, ports, terrain AGL, local assets, NDVI, soil and separation tests |
| Complete integration | Passed | `experiments/20260925T061916_078915Z/acceptance.json`: all four missions, wind, 1,712 total image/depth frames, environment data, four ULogs, metrics and clean shutdown |

The final compact integration run recorded 214 RGB and 214 depth frames for each UAV with zero dropped frames and no recorder errors. All vehicles reached mission items 1–7, returned, landed and disarmed. The minimum observed separation was 3.342 m against a 3 m threshold, so the safety monitor raised no collision warning. The run produced `analysis/metrics.json`, `analysis/summary.png`, four telemetry CSVs, four ULogs, weather/soil/crop CSVs and a passing `acceptance.json`.

Coverage percentages in that run are intentionally low because the compact regression mission surveys only a 20 m × 20 m patch inside each 340 m × 340 m field. The default launcher uses the full field missions. Energy consumption remains `null` when PX4's simulated battery stream does not provide current; the analyzer does not invent a value.

ROS 2 and an XRCE-DDS agent are absent from this Mac, so ROS namespaces and rosbag recording were not runtime-tested and are not presented as completed. This project does not require ROS 2 for its current control or datasets. The PX4 documentation states that its official Gazebo multi-vehicle workflow is Linux-only; the native macOS results above apply to the inspected versions only.
