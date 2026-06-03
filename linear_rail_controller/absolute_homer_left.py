import rclpy
from rclpy.node import Node
import serial
import time
import threading
import re
from gpiozero import Button
from std_srvs.srv import Trigger
from std_msgs.msg import Float64, Bool

class AbsoluteHomerLeft(Node):
    def __init__(self):
        super().__init__('absolute_homer_left')

        self.serial_lock = threading.Lock()
        self.is_connected = False
        
        # --- INTERNAL STATE VARIABLES ---
        self.in_alarm = False
        self.is_moving = False
        self.current_position_mm = 0.0

        # --- DECLARE & FETCH ROS 2 PARAMETERS ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rail_length_mm', 1000.0), ('steps_per_mm', 320.0),
                ('max_velocity_mm_s', 300.0), ('max_acceleration_mm_s2', 150.0),
                ('invert_direction', False), ('serial_port', '/dev/ttyACM0'),
                ('baud_rate', 115200), ('status_poll_rate_hz', 50.0),
                ('search_velocity_mm_s', 5.0), ('jog_velocity_mm_s', 50.0),
                ('sensor_1_pin', 17), ('sensor_1_pos', 0.0),
                ('sensor_2_pin', 27), ('sensor_2_pos', -300.0),
                ('sensor_3_pin', 22), ('sensor_3_pos', -600.0),
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
        
        self.sensor_1_pos = self.get_parameter('sensor_1_pos').value
        self.sensor_2_pos = self.get_parameter('sensor_2_pos').value
        self.sensor_3_pos = self.get_parameter('sensor_3_pos').value

        # --- ROS 2 INTERFACE ---
        self.pos_pub = self.create_publisher(Float64, '~/current_position_mm', 10)
        self.alarm_pub = self.create_publisher(Bool, '~/in_alarm', 10)
        self.moving_pub = self.create_publisher(Bool, '~/is_moving', 10)
        
        self.jog_sub = self.create_subscription(Float64, '~/jog_distance', self.jog_callback, 10)
        
        self.flash_srv = self.create_service(Trigger, '~/flash_pico_config', self.flash_config_callback)
        self.clear_alarm_srv = self.create_service(Trigger, '~/clear_alarm', self.clear_alarm_callback)
        self.home_end_side_srv = self.create_service(Trigger, '~/home_end_side', self.home_end_side_callback)
        self.home_motor_side_srv = self.create_service(Trigger, '~/home_motor_side', self.home_motor_side_callback)

        self.is_homing = False
        self.awaiting_sync = False
        self.sync_position_mm = 0.0

        self.keep_running = True
        self.watchdog_thread = threading.Thread(target=self.connection_watchdog_loop, daemon=True)
        self.watchdog_thread.start()

        # Timer to ask Pico for its status
        self.poll_timer = self.create_timer(1.0 / self.status_poll_rate_hz, self.poll_pico_status)
        
        # Dedicated timer to continuously publish states to ROS network at 20Hz
        self.state_pub_timer = self.create_timer(0.05, self.publish_states_callback)

        self.setup_sensors()
        self.get_logger().info("Node 'absolute_homer_left' initialized. Soft Limits Enabled. Publishing states continuously.")

    def setup_sensors(self):
        s1_pin = self.get_parameter('sensor_1_pin').value
        s2_pin = self.get_parameter('sensor_2_pin').value
        s3_pin = self.get_parameter('sensor_3_pin').value
        bounce = self.get_parameter('sensor_bounce_time').value

        self.sensor_1 = Button(s1_pin, pull_up=False, bounce_time=bounce)
        self.sensor_2 = Button(s2_pin, pull_up=False, bounce_time=bounce)
        self.sensor_3 = Button(s3_pin, pull_up=False, bounce_time=bounce)

        self.sensor_1.when_pressed = lambda: self.sensor_triggered_callback(self.sensor_1_pos)
        self.sensor_2.when_pressed = lambda: self.sensor_triggered_callback(self.sensor_2_pos)
        self.sensor_3.when_pressed = lambda: self.sensor_triggered_callback(self.sensor_3_pos)

    # --- CONTINUOUS PUBLISHER ---
    def publish_states_callback(self):
        # Publish Position
        pos_msg = Float64()
        pos_msg.data = self.current_position_mm
        self.pos_pub.publish(pos_msg)

        # Publish Alarm State
        alarm_msg = Bool()
        alarm_msg.data = self.in_alarm
        self.alarm_pub.publish(alarm_msg)

        # Publish Moving State
        moving_msg = Bool()
        moving_msg.data = self.is_moving
        self.moving_pub.publish(moving_msg)

    def jog_callback(self, msg):
        if self.in_alarm:
            self.get_logger().warn("Cannot jog: Pico is in ALARM state.")
            return
        if not self.is_connected:
            self.get_logger().warn("Cannot jog: Pico not connected.")
            return
        if self.is_homing:
            self.get_logger().warn("Cannot jog: Homing in progress.")
            return

        distance_mm = msg.data 
        
        # --- SOFTWARE LIMIT CHECK ---
        # Calculate where the rail will end up after this jog
        projected_target = self.current_position_mm + distance_mm
        
        # Upper bound is 0.0, lower bound is negative rail length (e.g., -1000.0)
        lower_bound = -self.rail_length_mm
        upper_bound = 0.0
        
        if projected_target > upper_bound or projected_target < lower_bound:
            self.get_logger().warn(
                f"REJECTED: Target {projected_target:.2f} mm is out of bounds "
                f"({lower_bound} to {upper_bound}). Command ignored to prevent crash."
            )
            return
        # ----------------------------

        feed_rate = self.jog_velocity_mm_s * 60.0 
        self.send_gcode(f"$J=G21G91X{distance_mm}F{feed_rate}")
        self.get_logger().info(f"Jogging {distance_mm} mm... (Target: {projected_target:.2f} mm)")

    def configure_pico_eeprom(self):
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
            
        self.get_logger().info("Restarting GRBL to apply parameters...")
        self.send_gcode("\x18") 
        time.sleep(1.0)
        self.send_gcode("$X")
        time.sleep(0.2)
        self.get_logger().info("Pico Flash Complete and Unlocked.")

    def flash_config_callback(self, request, response):
        self.configure_pico_eeprom()
        response.success = True
        response.message = "Pico configuration flashed successfully."
        return response

    def home_end_side_callback(self, request, response):
        return self._execute_homing(direction_multiplier=1, target_name="END SIDE", response=response)

    def home_motor_side_callback(self, request, response):
        return self._execute_homing(direction_multiplier=-1, target_name="MOTOR SIDE", response=response)

    def _execute_homing(self, direction_multiplier, target_name, response):
        if self.in_alarm:
            response.success = False
            response.message = "REJECTED: Pico in ALARM state. Call ~/clear_alarm first."
            self.get_logger().warn("Attempted to home while in ALARM state.")
            return response

        if not self.is_connected:
            response.success = False
            response.message = "Pico not connected."
            return response

        self.send_gcode("G92 X0")
        time.sleep(0.1)

        self.is_homing = True
        feed_rate = self.search_velocity_mm_s * 60.0 
        jog_dist = 1500.0 * direction_multiplier 
        
        self.send_gcode(f"$J=G21G91X{jog_dist}F{feed_rate}")

        response.success = True
        response.message = f"Homing sequence started moving towards {target_name}."
        self.get_logger().info(response.message)
        return response

    def clear_alarm_callback(self, request, response):
        if not self.is_connected:
            response.success = False
            response.message = "Pico not connected."
            return response
            
        self.get_logger().info("Sending Soft Reset to Pico...")
        self.send_gcode("\x18") 
        time.sleep(1.0)
        self.send_gcode("$X") 
        time.sleep(0.5)
        
        self.in_alarm = False
        self.get_logger().info("User manually cleared ALARM. Pico Soft-Reset and Unlocked.")
        
        response.success = True
        response.message = "Alarm cleared. CRITICAL: Use the manual jogger to move away from the sensor before homing!"
        return response

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
                        if "Alarm" in line:
                            if not self.in_alarm:
                                self.get_logger().error(f"HARD LIMIT OR CRASH! Status: {line}")
                                self.in_alarm = True
                                self.is_homing = False
                        self.update_internal_states(line)
                    elif "ALARM" in line.upper():
                        if not self.in_alarm:
                            self.get_logger().error(f"HARD LIMIT OR CRASH! {line}")
                            self.in_alarm = True
                            self.is_homing = False
                    elif "error" in line.lower():
                        self.get_logger().warn(f"PICO ERROR: {line}")
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

        # Update internal moving state silently
        self.is_moving = "Run" in status_line or "Jog" in status_line

        # Update internal position silently
        match = re.search(r'(?:MPos|WPos):([-\d.]+)', status_line)
        if match:
            self.current_position_mm = float(match.group(1))

    def sensor_triggered_callback(self, pos):
        if self.is_homing:
            self.is_homing = False
            self.send_gcode("!")    
            time.sleep(0.05)
            self.send_gcode("\x85") 
            self.sync_position_mm = pos
            self.awaiting_sync = True

def main(args=None):
    rclpy.init(args=args)
    node = AbsoluteHomerLeft()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.keep_running = False
        rclpy.shutdown()

if __name__ == '__main__':
    main()
