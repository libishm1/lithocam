# Robot meshes

Collision meshes from the robots' official descriptions, unchanged:

| Folder | Source | Licence |
|---|---|---|
| `ur10e/` | Universal Robots, [Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) `meshes/ur10e/collision` | BSD-3-Clause (`ur10e/LICENSE`) |
| `kr120/` | ROS-Industrial, [kuka_experimental](https://github.com/ros-industrial/kuka_experimental) `kuka_kr120_support/meshes/kr120r2500pro/collision` | Apache-2.0 (`kr120/LICENSE`) |
| `irb6700/` | ROS-Industrial, [abb](https://github.com/ros-industrial/abb) `abb_irb6700_support/meshes` (collision and visual STL, IRB 6700-200/2.60) | Apache-2.0 (`irb6700/LICENSE`, credit in `irb6700/NOTICE`) |

`make_cells.py` places them with the mesh offsets from the official URDFs (UR: `config/ur10e/visual_parameters.yaml`;
KR120: zero offsets in `kr120r2500pro.urdf`).
