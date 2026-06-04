# Navigate to workspace
cd ~/ros2_ws
colcon build --packages-select linear_rail_controller
source install/setup.bash

# Run the Node
ros2 launch linear_rail_controller rail_left.launch.py

# Flash Pico
ros2 service call /absolute_homer_left/flash_pico_config std_srvs/srv/Trigger

# Homing
ros2 service call /absolute_homer_left/home_motor_side std_srvs/srv/Trigger
ros2 service call /absolute_homer_left/home_end_side std_srvs/srv/Trigger

# Topics
ros2 topic echo /absolute_homer_left/current_position_mm
ros2 topic echo /absolute_homer_left/in_alarm
ros2 topic echo /absolute_homer_left/is_moving

# Clearing Alarm
ros2 service call /absolute_homer_left/clear_alarm std_srvs/srv/Trigger

# Message
ros2 topic pub /absolute_homer_left/absolute_target std_msgs/msg/Float64 "{data: -300.0}" -1
ros2 topic pub /absolute_homer_left/relative_jog std_msgs/msg/Float64 "{data: 50.0}" -1

# Demo
ros2 run linear_rail_controller demo

# Demo Message
ros2 topic pub /demo/absolute_target std_msgs/msg/Float64 "{data: -200.0}" -1
ros2 topic pub /demo/relative_jog std_msgs/msg/Float64 "{data: 50.0}" -1

# Demo Alarm Clear
ros2 service call /standalone_rail_demo/clear_alarm std_srvs/srv/Trigger

# Demo Topics
ros2 topic echo /demo/current_position_mm
ros2 topic echo /demo/is_moving
ros2 topic echo /demo/in_alarm
