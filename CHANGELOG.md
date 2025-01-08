# Changelog

## [Branch: ignition-gazebo] - 2025-01-08
- Added `CHANGELOG.md` file.
- Set different planners for `kuka_arm` (`pilz_industrial_motion_planner`) and `robotiq_2f140` (`ompl`).
- Modified the calculation of `TARGET_POSE` in `kuka_control/control_scripts/src/move_xyzw.cpp`.
- Added empty world as parameter for simulation in `/kuka_gazebo/launch/multi_kuka_bringup.launch.py`