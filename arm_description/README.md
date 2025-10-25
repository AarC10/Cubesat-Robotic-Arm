# Arm_description

This package contains the URDF/xacro, meshes, launch files and RViz config for the robotic arm.

Changes made:
- Converted package to ROS2 / `ament_cmake` style (updated `package.xml` and `CMakeLists.txt`).
- Added ROS2 Python launch files: `launch/display.launch.py` and `launch/gazebo.launch.py`.
- Install rules added so `urdf`, `meshes`, `launch` and `rviz` are installed to `share/Arm_description`.

Quick build & run (on a ROS2 workspace with the correct distro sourced):

```bash
# from workspace root (where src/ is)
colcon build --packages-select Arm_description
source install/setup.zsh  # or install/setup.bash

# Display in RViz2
ros2 launch Arm_description display.launch.py

# Spawn into Gazebo (make sure gazebo server is running or include an empty_world launch)
ros2 launch Arm_description gazebo.launch.py
```

Notes:
- The original ROS1 XML launch files are left in `launch/` as backups. Use the new `*.launch.py` files for ROS2.
- If you want the package name lowercased (recommended for ROS conventions) we can rename the package, but that will require updating references across the repo.
