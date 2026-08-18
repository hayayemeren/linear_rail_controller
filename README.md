# Navigate to workspace
cd ~/ros2_ws

colcon build --packages-select linear_rail_controller

source install/setup.bash

# Flash Pico Left
ros2 service call /absolute_homer_left/flash_pico_config std_srvs/srv/Trigger

# Clearing Alarm Left
ros2 service call /absolute_homer_left/clear_alarm std_srvs/srv/Trigger

# Message Left
ros2 topic pub /absolute_homer_left/absolute_target std_msgs/msg/Float64 "{data: -300.0}" -1

ros2 topic pub /absolute_homer_left/relative_jog std_msgs/msg/Float64 "{data: 50.0}" -1

# Run the Node Left
ros2 launch linear_rail_controller rail_left.launch.py

# Homing Left
ros2 service call /absolute_homer_left/home_motor_side std_srvs/srv/Trigger

ros2 service call /absolute_homer_left/home_end_side std_srvs/srv/Trigger

# Topics Left
ros2 topic echo /absolute_homer_left/current_position_mm

ros2 topic echo /absolute_homer_left/in_alarm

ros2 topic echo /absolute_homer_left/is_moving

# Flash Pico Right
ros2 service call /absolute_homer_right/flash_pico_config std_srvs/srv/Trigger

# Clearing Alarm Right
ros2 service call /absolute_homer_right/clear_alarm std_srvs/srv/Trigger

# Message Right
ros2 topic pub /absolute_homer_right/absolute_target std_msgs/msg/Float64 "{data: -300.0}" -1

ros2 topic pub /absolute_homer_right/relative_jog std_msgs/msg/Float64 "{data: 50.0}" -1

# Run the Node Right
ros2 launch linear_rail_controller rail_right.launch.py

# Homing Right
ros2 service call /absolute_homer_right/home_motor_side std_srvs/srv/Trigger

ros2 service call /absolute_homer_right/home_end_side std_srvs/srv/Trigger

# Topics Right
ros2 topic echo /absolute_homer_right/current_position_mm

ros2 topic echo /absolute_homer_right/in_alarm

ros2 topic echo /absolute_homer_right/is_moving

# Demo
ros2 launch linear_rail_controller demo.launch.py

# Demo Message x Axis
ros2 topic pub /demo/x/absolute_target std_msgs/msg/Float64 "{data: -200.0}" -1

ros2 topic pub /demo/x/relative_jog std_msgs/msg/Float64 "{data: 50.0}" -1

# Demo Set Current x Position
ros2 topic pub --once /demo/x/set_current_position std_msgs/msg/Float64 "{data: 150.0}"

# Listen Topics x
ros2 topic echo /demo/x/current_position_mm

ros2 topic echo /demo/x/is_moving

# Demo Message y Axis
ros2 topic pub /demo/y/absolute_target std_msgs/msg/Float64 "{data: -200.0}" -1

ros2 topic pub /demo/y/relative_jog std_msgs/msg/Float64 "{data: 50.0}" -1

# Demo Set Current y Position
ros2 topic pub --once /demo/y/set_current_position std_msgs/msg/Float64 "{data: 150.0}"

# Listen Position y
ros2 topic echo /demo/y/current_position_mm

ros2 topic echo /demo/y/is_moving

# Demo Alarm Clear
ros2 service call /standalone_rail_demo/clear_alarm std_srvs/srv/Trigger

ros2 topic echo /demo/in_alarm

# Demo Start Homing
ros2 service call /standalone_rail_demo/x/home std_srvs/srv/Trigger

ros2 service call /standalone_rail_demo/y/home std_srvs/srv/Trigger

ros2 service call /standalone_rail_demo/home_all std_srvs/srv/Trigger

# Flash Pico
ros2 service call /standalone_rail_demo/flash_pico_config std_srvs/srv/Trigger