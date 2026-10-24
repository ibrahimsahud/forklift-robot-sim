# articubot_one simulation package

This package contains the Gazebo, RViz, SLAM, Nav2, camera, QR, line-following
and forklift mission components used by the Tekno robot simulation.

The package is intentionally simulation-only. It contains no firmware upload
scripts, serial motor drivers, physical LiDAR launch files or hardware plugins.

Main entry points:

- `launch/launch_sim.launch.py`: Gazebo simulation and optional mission nodes.
- `launch/localization_navigation.launch.py`: saved-map localization and Nav2.
- `launch/bringup_all.launch.py`: mapping workflow.
- `launch/autonomous_competition.launch.py`: complete simulated route workflow.

Worlds, maps, QR textures and the robot Xacro model are installed with the
package, so the workspace can be rebuilt from a clean clone.
