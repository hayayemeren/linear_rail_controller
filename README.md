# Navigate to workspace
cd ~/ros2_ws

colcon build --packages-select linear_rail_controller

source install/setup.bash

# Demo
ros2 launch linear_rail_controller demo.launch.py
*(Launches the standalone rail demo node which controls both X and Y axes on a single Pico)*

# Demo Message x Axis
ros2 topic pub /demo/x/absolute_target std_msgs/msg/Float64 "{data: -200.0}" -1
*(Moves the X axis to the absolute coordinate -200.0 mm. Will be rejected if it violates safety constraints.)*

ros2 topic pub /demo/x/relative_jog std_msgs/msg/Float64 "{data: 50.0}" -1
*(Jogs the X axis 50.0 mm from its current position.)*

# Demo Set Current x Position
ros2 topic pub --once /demo/x/set_current_position std_msgs/msg/Float64 "{data: 150.0}"
*(Artificially sets the current X position to 150.0 mm without physically moving the motors.)*

# Listen Topics x
ros2 topic echo /demo/x/current_position_mm
*(Streams the live X position in mm from the Pico.)*

ros2 topic echo /demo/x/is_moving
*(Streams a boolean indicating if the X axis is currently moving.)*

# Demo Message y Axis
ros2 topic pub /demo/y/absolute_target std_msgs/msg/Float64 "{data: -200.0}" -1
*(Moves the Y axis to the absolute coordinate -200.0 mm. Will be rejected if it violates safety constraints.)*

ros2 topic pub /demo/y/relative_jog std_msgs/msg/Float64 "{data: 50.0}" -1
*(Jogs the Y axis 50.0 mm from its current position.)*

# Demo Set Current y Position
ros2 topic pub --once /demo/y/set_current_position std_msgs/msg/Float64 "{data: 150.0}"
*(Artificially sets the current Y position to 150.0 mm without physically moving the motors.)*

# Listen Position y
ros2 topic echo /demo/y/current_position_mm
*(Streams the live Y position in mm from the Pico.)*

ros2 topic echo /demo/y/is_moving
*(Streams a boolean indicating if the Y axis is currently moving.)*

# Demo Alarm Clear
ros2 service call /standalone_rail_demo/clear_alarm std_srvs/srv/Trigger
*(Sends a Soft Reset to instantly halt all movement and unlocks the Pico from an ALARM state.)*
*(Can be used as failsw'tch to halt dangerous operations but use the physical button if you can.)*

ros2 topic echo /demo/in_alarm
*(Streams a boolean indicating if the Pico is currently in a locked ALARM state.)*

# Demo Start Homing
ros2 service call /standalone_rail_demo/x/home std_srvs/srv/Trigger
*(Homes only the X axis and sets its absolute position to 5 mm.)*

ros2 service call /standalone_rail_demo/y/home std_srvs/srv/Trigger
*(Homes only the Y axis and sets its absolute position to 2790 mm.)*

ros2 service call /standalone_rail_demo/home_all std_srvs/srv/Trigger
*(Homes both axes simultaneously and sets X=5, Y=2790.)*

# Flash Pico
ros2 service call /standalone_rail_demo/flash_pico_config std_srvs/srv/Trigger
*(Flashes the GRBL EEPROM settings (speeds, scales, limits) to match your custom configuration.)*

# Demo Simultaneous XY Movement
ros2 topic pub --once /demo/xy_absolute_target geometry_msgs/msg/Point "{x: 50.0, y: 250.0}"
*(Moves both X and Y axes simultaneously to the specified absolute coordinates.)*

ros2 topic pub --once /demo/xy_relative_jog geometry_msgs/msg/Point "{x: 10.0, y: -10.0}"
*(Jogs both axes simultaneously relative to their current positions.)*

# X-Y Collision Safety Logic
*(The demo script includes a built-in safety constraint to prevent physical collisions on the rail. It strictly enforces the rule `X <= Y - 150`. If any movement command (absolute, relative, or simultaneous) or position override would result in this condition being broken, the command is instantly rejected, the motors will not move, and an `x-y collusion` error will be printed to the node's terminal log.)*

------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------

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
