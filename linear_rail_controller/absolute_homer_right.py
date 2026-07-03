import rclpy
from rclpy.node import Node
import serial
import time
import threading
import re
from gpiozero import Button
from std_srvs.srv import Trigger
from std_msgs.msg import Float64, Bool

class AbsoluteHomerRight(Node):
    def __init__(self):
        super().__init__('absolute_homer_right')

        self.serial_lock = threading.Lock()
        self.is_connected = False
        
        # --- INTERNAL STATE VARIABLES ---
        self.in_alarm = False
        self.is_moving = False
        self.current_position_mm = 0.0
        self.position_initialized = False

        # --- DECLARE & FETCH ROS 2 PARAMETERS ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rail_length_mm', 1000.0), ('steps_per_mm', 320.0),
                ('max_velocity_mm_s', 300.0), ('max_acceleration_mm_s2', 150.0),
                ('invert_direction', False), ('serial_port', '/dev/ttyACM1'),
                ('baud_rate', 115200), ('status_poll_rate_hz', 50.0),
                ('search_velocity_mm_s', 15.0), ('jog_velocity_mm_s', 50.0),
                ('sensor_pin', 17), 
                ('sheet_positions_mm', [0.0, -100.0, -250.0, -450.0]),
                ('gap_tolerance_mm', 5.0),
                ('sensor_bounce_time', 0.01)
            ]
        )

        self.rail_length_mm = self.get_parameter('rail_length_mm').value
        self.steps_per_mm = self.get_parameter('steps_per_mm').value
        self.max_velocity_mm_s = self.get_parameter('max_velocity_mm_s').value
        self.max_acceleration_mm_s2 = self.get_parameter('max_acceleration_mm_s2').value
        self.invert_direction = self.get_parameter('invert_direction').value
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.status_poll_rate_hz = self.get_parameter('status_poll_rate_hz').value
        self.search_velocity_mm_s = self.get_parameter('search_velocity_mm_s').value
        self.jog_velocity_mm_s = self.get_parameter('jog_velocity_mm_s').value
        
        # New Right-Side specific parameters
        self.sheet_positions = sorted(self.get_parameter('sheet_positions_mm').value, reverse=True)
        self.gap_tolerance = self.get_parameter('gap_tolerance_mm').value

        # --- ROS 2 INTERFACE ---
        self.pos_pub = self.create_publisher(Float64, '~/current_position_mm', 10)
        self.alarm_pub = self.create_publisher(Bool, '~/in_alarm', 10)
        self.moving_pub = self.create_publisher(Bool, '~/is_moving', 10)
        
        self.rel_sub = self.create_subscription(Float64, '~/relative_jog', self.relative_jog_callback, 10)
        self.abs_sub = self.create_subscription(Float64, '~/absolute_target', self.absolute_target_callback, 10)
        
        self.flash_srv = self.create_service(Trigger, '~/flash_pico_config', self.flash_config_callback)
        self.clear_alarm_srv = self.create_service(Trigger, '~/clear_alarm', self.clear_alarm_callback)
        self.home_end_side_srv = self.create_service(Trigger, '~/home_end_side', self.home_end_side_callback)
        self.home_motor_side_srv = self.create_service(Trigger, '~/home_motor_side', self.home_motor_side_callback)

        self.is_homing = False
        self.awaiting_sync = False
        self.sync_position_mm = 0.0
        
        # Gap matching variables
        self.homing_direction = 0
        self.trigger_positions = []

        self.keep_running = True
        self.watchdog_thread = threading.Thread(target=self.connection_watchdog_loop, daemon=True)
        self.watchdog_thread.start()

        self.poll_timer = self.create_timer(1.0 / self.status_poll_rate_hz, self.poll_pico_status)
        self.state_pub_timer = self.create_timer(0.05, self.publish_states_callback)

        self.setup_sensors()
        self.get_logger().info("Node 'absolute_homer_right' initialized. Gap-based Homing Enabled.")

    def setup_sensors(self):
        pin = self.get_parameter('sensor_pin').value
        bounce = self.get_parameter('sensor_bounce_time').value

        # Single moving sensor reading stationary sheets
        self.sensor = Button(pin, pull_up=False, bounce_time=bounce)
        self.sensor.when_pressed = self.sensor_triggered_callback

    def publish_states_callback(self):
        pos_msg = Float64()
        pos_msg.data = self.current_position_mm
        self.pos_pub.publish(pos_msg)

        alarm_msg = Bool()
        alarm_msg.data = self.in_alarm
        self.alarm_pub.publish(alarm_msg)

        moving_msg = Bool()
        moving_msg.data = self.is_moving
        self.moving_pub.publish(moving_msg)

    # ==========================================
    # MOVEMENT LOGIC & BOUNDARIES
    # ==========================================

    def relative_jog_callback(self, msg):
        distance_mm = msg.data
        projected_target = self.current_position_mm + distance_mm
        self._execute_jog(distance_mm, projected_target)

    def absolute_target_callback(self, msg):
        if not self.position_initialized:
            self.get_logger().warn("Cannot calculate absolute target: Waiting for initial position.")
            return
        target_mm = msg.data
        distance_mm = target_mm - self.current_position_mm
        self._execute_jog(distance_mm, target_mm)

    def _execute_jog(self, distance_mm, projected_target):
        if self.in_alarm or not self.is_connected or self.is_homing:
            return

        if abs(distance_mm) < 0.1:
            return

        lower_bound = -self.rail_length_mm
        upper_bound = 0.0
        
        if projected_target > upper_bound or projected_target < lower_bound:
            self.get_logger().warn(f"REJECTED: Target {projected_target:.2f} mm is out of bounds.")
            return

        feed_rate = self.jog_velocity_mm_s * 60.0 
        self.send_gcode(f"$J=G21G91X{distance_mm}F{feed_rate}")

    # ==========================================
    # PICO CONFIGURATION & HOMING
    # ==========================================

    def configure_pico_eeprom(self):
        # Implementation remains exactly the same as left side
        self.get_logger().info("Flashing Pico EEPROM...")
        max_vel = self.max_velocity_mm_s * 60.0 
        invert_mask = 1 if self.invert_direction else 0

        config_commands = [
            "$0=5.0", "$1=25", "$2=0", f"$3={invert_mask}", 
            "$4=1", "$5=6", "$6=1", "$10=510", "$14=70", 
            "$20=0", "$21=1", "$22=0", 
            f"$100={self.steps_per_mm}", f"$110={max_vel}", 
            f"$120={self.max_acceleration_mm_s2}", f"$130={self.rail_length_mm}"
        ]

        for cmd in config_commands:
            self.send_gcode(cmd)
            time.sleep(0.05)
            
        self.send_gcode("\x18") 
        time.sleep(1.0)
        self.send_gcode("$X")
        time.sleep(0.2)

    def flash_config_callback(self, request, response):
        self.configure_pico_eeprom()
        response.success = True
        return response

    def home_end_side_callback(self, request, response):
        return self._execute_homing(direction_multiplier=1, target_name="END SIDE", response=response)

    def home_motor_side_callback(self, request, response):
        return self._execute_homing(direction_multiplier=-1, target_name="MOTOR SIDE", response=response)

    def _execute_homing(self, direction_multiplier, target_name, response):
        if self.in_alarm or not self.is_connected:
            response.success = False
            return response

        self.send_gcode("G92 X0")
        time.sleep(0.1)

        self.is_homing = True
        self.trigger_positions = []
        self.homing_direction = direction_multiplier

        feed_rate = self.search_velocity_mm_s * 60.0 
        jog_dist = 1500.0 * direction_multiplier 
        
        self.send_gcode(f"$J=G21G91X{jog_dist}F{feed_rate}")

        response.success = True
        response.message = f"Homing towards {target_name}. Awaiting two sheet triggers."
        self.get_logger().info(response.message)
        return response

    def clear_alarm_callback(self, request, response):
        if not self.is_connected:
            response.success = False
            return response
        self.send_gcode("\x18") 
        time.sleep(1.0)
        self.send_gcode("$X") 
        time.sleep(0.5)
        self.in_alarm = False
        response.success = True
        return response

    # ==========================================
    # UNIQUE GAP HOMING ALGORITHM
    # ==========================================

    def sensor_triggered_callback(self):
        if self.is_homing:
            self.trigger_positions.append(self.current_position_mm)
            
            if len(self.trigger_positions) == 1:
                self.get_logger().info("First sheet detected. Continuing to next for gap measurement...")
            
            elif len(self.trigger_positions) == 2:
                # Stop the motor immediately
                self.is_homing = False
                self.send_gcode("!")    
                time.sleep(0.05)
                self.send_gcode("\x85") 
                
                # Calculate the distance traveled between triggers
                gap = abs(self.trigger_positions[1] - self.trigger_positions[0])
                self.get_logger().info(f"Second sheet detected. Measured gap: {gap:.2f} mm")
                
                absolute_pos = self.match_gap_to_position(gap)
                
                if absolute_pos is not None:
                    self.sync_position_mm = absolute_pos
                    self.awaiting_sync = True
                else:
                    self.get_logger().error(f"Gap {gap:.2f}mm not recognized in sheet_positions! Homing failed.")

    def match_gap_to_position(self, measured_gap):
        sheets = self.sheet_positions
        for i in range(len(sheets) - 1):
            known_gap = abs(sheets[i] - sheets[i+1])
            
            if abs(known_gap - measured_gap) <= self.gap_tolerance:
                # Match found! Calculate which sheet we are currently stopped on
                if self.homing_direction == -1: 
                    # Moving towards negative, second trigger is the smaller (more negative) coordinate
                    return min(sheets[i], sheets[i+1])
                else:
                    # Moving towards positive, second trigger is the larger coordinate
                    return max(sheets[i], sheets[i+1])
        return None

    # ==========================================
    # HARDWARE COMMUNICATION
    # ==========================================

    def connect_to_pico(self):
        try:
            self.pico_serial = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.is_connected = True
            time.sleep(1.0)
            self.send_gcode("$X") 
            return True
        except Exception:
            return False

    def connection_watchdog_loop(self):
        while self.keep_running and rclpy.ok():
            if not self.is_connected:
                self.connect_to_pico()
                time.sleep(2.0)
                continue
            try:
                if self.pico_serial.in_waiting > 0:
                    line = self.pico_serial.readline().decode('utf-8', errors='ignore').strip()
                    if line.startswith('<'):
                        if "Alarm" in line and not self.in_alarm:
                            self.in_alarm = True
                            self.is_homing = False
                        self.update_internal_states(line)
                    elif "ALARM" in line.upper() and not self.in_alarm:
                        self.in_alarm = True
                        self.is_homing = False
            except Exception:
                self.is_connected = False
            time.sleep(0.005)

    def send_gcode(self, command):
        if self.is_connected:
            with self.serial_lock:
                try:
                    self.pico_serial.write((command + '\n').encode('utf-8'))
                except Exception:
                    pass

    def poll_pico_status(self):
        if self.is_connected:
            with self.serial_lock:
                try:
                    self.pico_serial.write(b'?')
                except Exception:
                    pass

    def update_internal_states(self, status_line):
        if self.awaiting_sync and "Idle" in status_line:
            self.send_gcode(f"G92 X{self.sync_position_mm}")
            self.awaiting_sync = False
            self.get_logger().info(f"Position Synced to {self.sync_position_mm} mm")

        self.is_moving = "Run" in status_line or "Jog" in status_line

        match = re.search(r'(?:MPos|WPos):([-\d.]+)', status_line)
        if match:
            self.current_position_mm = float(match.group(1))
            self.position_initialized = True

def main(args=None):
    rclpy.init(args=args)
    node = AbsoluteHomerRight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.keep_running = False
        rclpy.shutdown()

if __name__ == '__main__':
    main()