colcon build --packages-select CartStateManager SensorManager ActuatorManager --symlink-install
colcon build --packages-select robot_localization pinky_msgs SoloBringup

source install/setup.bash
