# ros2_ws — Workspace ROS 2 Jazzy

| Paquet | Type | Rôle |
|---|---|---|
| `isimm_robot` | `ament_python` | Nœuds du robot, launch, configuration, URDF, RViz, règles udev |
| `rf2o_laser_odometry` | `ament_cmake` (C++17) | Odométrie LiDAR (GPL v3, voir `THIRD_PARTY.md`) |

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
```

## Lancements

| Commande | Mode | Remarques |
|---|---|---|
| `ros2 launch isimm_robot robot.launch.py` | base | Diagnostic ; `mode:=base\|mapping\|navigation` |
| `ros2 launch isimm_robot mapping.launch.py` | mapping | SLAM Toolbox asynchrone |
| `ros2 launch isimm_robot navigation.launch.py map_file:=<chemin sans extension>` | navigation | `ISIMM_MAP_FILE` possible ; `use_rviz:=true` |

Ne jamais lancer *mapping* et *navigation* ensemble (garde-fou `check_duplicates`).

## Configuration

| Fichier | Contenu |
|---|---|
| `config/robot_params.yaml` | LiDAR, RF2O, moteurs, sécurité, mux, statut |
| `config/nav2_params.yaml` | Nav2, costmaps, Collision Monitor |
| `config/slam_params.yaml` / `slam_localization_params.yaml` | Mapping / localisation |
| `urdf/robot.urdf.xacro` | Modèle (0,57 × 0,40 m, LiDAR centré) |

## Topics principaux

| Topic | Type | Producteur |
|---|---|---|
| `/scan` | `LaserScan` | `rplidar_node` |
| `/odom` + TF `odom→base_link` | `Odometry` | RF2O |
| `/imu` | `Imu` | `esp32_sensor_node` |
| `/cmd_vel_smoothed` → `/cmd_vel_mux` → `/cmd_vel` → `/safe_cmd_vel` | `Twist` | smoother → mux → collision monitor → safety |
| `/cmd_vel_teleop`, `/manual_session` | `Twist`, `Bool` | `operator_ui` |
| `/cmd_vel_mode` | `String` | `cmd_vel_mux_node` (AUTO, AUTO_WAIT, MANUAL, ESTOP) |
| `/emergency_stop` | `Bool` | IA / interface |
| `/safety_state`, `/motor_status` | `String` | `safety_node`, `motor_controller_node` |
| `/robot_status` | `DiagnosticArray` | `robot_status_node` |

## Tests

```bash
cd ros2_ws/src/isimm_robot && PYTHONPATH=. python -m pytest test/test_cmd_vel_arbiter.py -q
colcon test --packages-select isimm_robot
```

Documentation : [`docs/architecture.md`](../docs/architecture.md) · [`docs/safety.md`](../docs/safety.md).
