import rclpy
from rclpy.node import Node
import serial
import time
import threading
import re
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

class MonolithicRailDemo(Node):
    def __init__(self):
        super().__init__('standalone_rail_demo')

        # --- HARDWARE CONNECTION STATES ---
        self.serial_lock = threading.Lock()
        self.is_connected = False
        self.in_alarm = False
        # --- PARAMETERS ---
        self.serial_port = self.declare_parameter('serial_port', '/dev/ttyACM0').value
        self.baud_rate = self.declare_parameter('baud_rate', 115200).value
        self.jog_velocity_mm_s = self.declare_parameter('jog_velocity_mm_s', 50.0).value
        self.grbl_scale = self.declare_parameter('grbl_scale', 0.5).value
        
        # --- HARDWARE CONFIG PARAMETERS ---
        self.rail_length_mm = self.declare_parameter('rail_length_mm', 2900.0).value
        self.steps_per_mm = self.declare_parameter('steps_per_mm', 320.0).value
        self.max_velocity_mm_s = self.declare_parameter('max_velocity_mm_s', 300.0).value
        self.max_acceleration_mm_s2 = self.declare_parameter('max_acceleration_mm_s2', 150.0).value
        self.invert_direction = self.declare_parameter('invert_direction', False).value
        self.enable_soft_limits = self.declare_parameter('enable_soft_limits', False).value
        self.enable_hard_limits = self.declare_parameter('enable_hard_limits', True).value
        self.enable_homing = self.declare_parameter('enable_homing', False).value

        self.current_position_x_mm = 0.0
        self.current_position_y_mm = 0.0
        self.last_position_x_mm = 0.0
        self.last_position_y_mm = 0.0
        self.x_is_moving = False
        self.y_is_moving = False
        self.position_initialized = False

        self.homing_state = 0 # 0: idle, 1: waiting for <Home, 2: waiting for <Idle
        self.pending_x_offset = None
        self.pending_y_offset = None

        # --- ROS 2 INTERFACE (DUAL MODE) ---
        # 1. Listen for Relative commands
        self.x_rel_sub = self.create_subscription(
            Float64, '/demo/x/relative_jog', self.x_relative_jog_callback, 10)
        self.y_rel_sub = self.create_subscription(
            Float64, '/demo/y/relative_jog', self.y_relative_jog_callback, 10)
            
        # 2. Listen for Absolute commands
        self.x_abs_sub = self.create_subscription(
            Float64, '/demo/x/absolute_target', self.x_absolute_target_callback, 10)
        self.y_abs_sub = self.create_subscription(
            Float64, '/demo/y/absolute_target', self.y_absolute_target_callback, 10)
            
        # 2.5 Listen for Simultaneous XY commands
        self.xy_rel_sub = self.create_subscription(
            Point, '/demo/xy_relative_jog', self.xy_relative_jog_callback, 10)
        self.xy_abs_sub = self.create_subscription(
            Point, '/demo/xy_absolute_target', self.xy_absolute_target_callback, 10)
            
        # 3. Listen for Set Position commands
        self.x_set_pos_sub = self.create_subscription(
            Float64, '/demo/x/set_current_position', self.x_set_current_position_callback, 10)
        self.y_set_pos_sub = self.create_subscription(
            Float64, '/demo/y/set_current_position', self.y_set_current_position_callback, 10)
        
        # 4. Publish current state
        self.x_pos_pub = self.create_publisher(Float64, '/demo/x/current_position_mm', 10)
        self.y_pos_pub = self.create_publisher(Float64, '/demo/y/current_position_mm', 10)
        self.x_moving_pub = self.create_publisher(Bool, '/demo/x/is_moving', 10)
        self.y_moving_pub = self.create_publisher(Bool, '/demo/y/is_moving', 10)
        self.alarm_pub = self.create_publisher(Bool, '/demo/in_alarm', 10)
        
        # 5. Safety & Configuration services
        self.clear_alarm_srv = self.create_service(Trigger, '~/clear_alarm', self.clear_alarm_callback)
        self.home_all_srv = self.create_service(Trigger, '~/home_all', self.home_all_callback)
        self.x_home_srv = self.create_service(Trigger, '~/x/home', self.x_home_callback)
        self.y_home_srv = self.create_service(Trigger, '~/y/home', self.y_home_callback)
        self.flash_config_srv = self.create_service(Trigger, '~/flash_pico_config', self.flash_config_callback)

        # --- HARDWARE THREADS & TIMERS ---
        self.keep_running = True
        
        self.watchdog_thread = threading.Thread(target=self.connection_watchdog_loop, daemon=True)
        self.watchdog_thread.start()

        self.poll_timer = self.create_timer(1.0 / 50.0, self.poll_pico_status)
        self.state_timer = self.create_timer(0.1, self.evaluate_movement_state)

        self.get_logger().info("Dual-Mode Demo initialized. Connecting directly to Pico...")
        self.get_logger().info("Listening on /demo/x/... and /demo/y/... topics")
        self.get_logger().info("Services available: ~/clear_alarm, ~/home_rail, ~/flash_pico_config")

    # ==========================================
    # ROS 2 LOGIC (Movement Control)
    # ==========================================

    def _pre_move_checks_pass(self):
        """Helper to check safety states before moving."""
        if self.in_alarm:
            self.get_logger().warn("Cannot move: Pico is in ALARM state. Call ~/clear_alarm first.")
            return False
        if not self.is_connected:
            self.get_logger().warn("Cannot move: Pico not connected via USB.")
            return False
        return True

    def _execute_jog(self, axis, jog_distance):
        """Helper to format and send the raw G-code."""
        if abs(jog_distance) < 0.1:
            self.get_logger().info("Jog distance too small, ignoring.")
            return

        # Apply scaling and formatting safely so GRBL doesn't reject it
        feed_rate = (self.jog_velocity_mm_s * 60.0) * self.grbl_scale
        scaled_jog = jog_distance * self.grbl_scale
        
        if axis == 'X':
            self.send_gcode(f"$J=G21G91X{scaled_jog:.3f}F{feed_rate:.1f}")
        elif axis == 'Y':
            self.send_gcode(f"$J=G21G91Y{scaled_jog:.3f}F{feed_rate:.1f}")
            
        self.get_logger().info(f"{axis} Motor Jogging {jog_distance:.2f} mm...")
        
    def _execute_xy_jog(self, x_distance, y_distance):
        """Helper to format and send simultaneous raw G-code."""
        if abs(x_distance) < 0.1 and abs(y_distance) < 0.1:
            self.get_logger().info("Jog distances too small, ignoring.")
            return
            
        feed_rate = (self.jog_velocity_mm_s * 60.0) * self.grbl_scale
        scaled_x = x_distance * self.grbl_scale
        scaled_y = y_distance * self.grbl_scale
        
        self.send_gcode(f"$J=G21G91X{scaled_x:.3f}Y{scaled_y:.3f}F{feed_rate:.1f}")
        self.get_logger().info(f"XY Motors Jogging X:{x_distance:.2f} mm, Y:{y_distance:.2f} mm...")

    def xy_relative_jog_callback(self, msg):
        if not self._pre_move_checks_pass(): return
        if not self.position_initialized:
            self.get_logger().warn("Cannot verify failsafe: Waiting for initial position read from Pico.")
            return
            
        target_x = self.current_position_x_mm + msg.x
        target_y = self.current_position_y_mm + msg.y
        
        if target_x > target_y - 150.0:
            self.get_logger().error("x-y collusion")
            return
            
        self.get_logger().info(f"Received XY Relative Command: X:{msg.x:.2f}, Y:{msg.y:.2f} mm")
        self._execute_xy_jog(msg.x, msg.y)

    def xy_absolute_target_callback(self, msg):
        if not self._pre_move_checks_pass(): return
        if not self.position_initialized:
            self.get_logger().warn("Cannot calculate absolute target: Waiting for initial position read from Pico.")
            return

        target_x = msg.x
        target_y = msg.y
        
        if target_x > target_y - 150.0:
            self.get_logger().error("x-y collusion")
            return
            
        jog_x = target_x - self.current_position_x_mm
        jog_y = target_y - self.current_position_y_mm
        
        self.get_logger().info(f"Received XY Absolute Target: X:{target_x}, Y:{target_y} mm")
        self._execute_xy_jog(jog_x, jog_y)

    def x_relative_jog_callback(self, msg):
        if not self._pre_move_checks_pass(): return
        if not self.position_initialized:
            self.get_logger().warn("Cannot verify failsafe: Waiting for initial position read from Pico.")
            return
            
        target_x = self.current_position_x_mm + msg.data
        if target_x > self.current_position_y_mm - 150.0:
            self.get_logger().error("x-y collusion")
            return
            
        self.get_logger().info(f"Received X Relative Command: {msg.data:.2f} mm")
        self._execute_jog('X', msg.data)

    def y_relative_jog_callback(self, msg):
        if not self._pre_move_checks_pass(): return
        if not self.position_initialized:
            self.get_logger().warn("Cannot verify failsafe: Waiting for initial position read from Pico.")
            return
            
        target_y = self.current_position_y_mm + msg.data
        if target_y < self.current_position_x_mm + 150.0:
            self.get_logger().error("x-y collusion")
            return
            
        self.get_logger().info(f"Received Y Relative Command: {msg.data:.2f} mm")
        self._execute_jog('Y', msg.data)

    def x_absolute_target_callback(self, msg):
        if not self._pre_move_checks_pass(): return
        if not self.position_initialized:
            self.get_logger().warn("Cannot calculate absolute target: Waiting for initial position read from Pico.")
            return

        target_mm = msg.data
        if target_mm > self.current_position_y_mm - 150.0:
            self.get_logger().error("x-y collusion")
            return
            
        jog_distance = target_mm - self.current_position_x_mm
        self.get_logger().info(f"Received X Absolute Target: {target_mm} mm. Calculated difference: {jog_distance:.2f} mm")
        self._execute_jog('X', jog_distance)

    def y_absolute_target_callback(self, msg):
        if not self._pre_move_checks_pass(): return
        if not self.position_initialized:
            self.get_logger().warn("Cannot calculate absolute target: Waiting for initial position read from Pico.")
            return

        target_mm = msg.data
        if target_mm < self.current_position_x_mm + 150.0:
            self.get_logger().error("x-y collusion")
            return
            
        jog_distance = target_mm - self.current_position_y_mm
        self.get_logger().info(f"Received Y Absolute Target: {target_mm} mm. Calculated difference: {jog_distance:.2f} mm")
        self._execute_jog('Y', jog_distance)

    def x_set_current_position_callback(self, msg):
        if not self._pre_move_checks_pass(): return
        new_position = msg.data
        if self.position_initialized and new_position > self.current_position_y_mm - 150.0:
            self.get_logger().error("x-y collusion")
            return
            
        self.get_logger().info(f"Setting current GRBL X-axis position to: {new_position:.2f} mm")
        scaled_pos = new_position * self.grbl_scale
        self.send_gcode(f"G92X{scaled_pos:.3f}")
        self.current_position_x_mm = new_position
        self.position_initialized = True

    def y_set_current_position_callback(self, msg):
        if not self._pre_move_checks_pass(): return
        new_position = msg.data
        if self.position_initialized and new_position < self.current_position_x_mm + 150.0:
            self.get_logger().error("x-y collusion")
            return
            
        self.get_logger().info(f"Setting current GRBL Y-axis position to: {new_position:.2f} mm")
        scaled_pos = new_position * self.grbl_scale
        self.send_gcode(f"G92Y{scaled_pos:.3f}")
        self.current_position_y_mm = new_position
        self.position_initialized = True

    def evaluate_movement_state(self):
        if not self.position_initialized:
            return
        
        diff_x = abs(self.current_position_x_mm - self.last_position_x_mm)
        diff_y = abs(self.current_position_y_mm - self.last_position_y_mm)
        
        x_currently_moving = diff_x > 0.05
        y_currently_moving = diff_y > 0.05
        
        if x_currently_moving != self.x_is_moving:
            self.x_is_moving = x_currently_moving
            state_str = "MOVING" if self.x_is_moving else "STOPPED"
            self.get_logger().info(f"X Axis physical state changed: {state_str}")
            
        if y_currently_moving != self.y_is_moving:
            self.y_is_moving = y_currently_moving
            state_str = "MOVING" if self.y_is_moving else "STOPPED"
            self.get_logger().info(f"Y Axis physical state changed: {state_str}")
            
        x_moving_msg = Bool()
        x_moving_msg.data = bool(self.x_is_moving)
        self.x_moving_pub.publish(x_moving_msg)
        
        y_moving_msg = Bool()
        y_moving_msg.data = bool(self.y_is_moving)
        self.y_moving_pub.publish(y_moving_msg)
        
        alarm_msg = Bool()
        alarm_msg.data = bool(self.in_alarm)
        self.alarm_pub.publish(alarm_msg)
        
        self.last_position_x_mm = self.current_position_x_mm
        self.last_position_y_mm = self.current_position_y_mm

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
        self.get_logger().info("User manually cleared ALARM. Pico Unlocked.")
        
        response.success = True
        response.message = "Alarm cleared."
        return response

    def home_all_callback(self, request, response):
        """Triggers the GRBL built-in homing sequence for all configured axes ($H)."""
        if not self.is_connected:
            response.success = False
            response.message = "Pico not connected."
            return response
            
        self.get_logger().info("Setting mask $44=3 and Sending Homing Command ($H) to Pico...")
        self.send_gcode("$44=3")
        time.sleep(0.05)
        self.send_gcode("$H") 
        
        self.homing_state = 1
        self.pending_x_offset = 5.0 * self.grbl_scale
        self.pending_y_offset = 2790.0 * self.grbl_scale
        
        response.success = True
        response.message = "Full homing sequence initiated. Position will be set to X=5, Y=2790."
        return response

    def x_home_callback(self, request, response):
        """Homes ONLY the X axis."""
        if not self.is_connected:
            response.success = False
            response.message = "Pico not connected."
            return response
            
        self.get_logger().info("Setting mask $44=1 and Sending Homing Command ($H) to Pico...")
        self.send_gcode("$44=1")
        time.sleep(0.05)
        self.send_gcode("$H") 
        
        self.homing_state = 1
        self.pending_x_offset = 5.0 * self.grbl_scale
        self.pending_y_offset = None
        
        response.success = True
        response.message = "X-axis homing sequence initiated. Position will be set to X=5."
        return response

    def y_home_callback(self, request, response):
        """Homes ONLY the Y axis."""
        if not self.is_connected:
            response.success = False
            response.message = "Pico not connected."
            return response
            
        self.get_logger().info("Setting mask $44=2 and Sending Homing Command ($H) to Pico...")
        self.send_gcode("$44=2")
        time.sleep(0.05)
        self.send_gcode("$H") 
        
        self.homing_state = 1
        self.pending_x_offset = None
        self.pending_y_offset = 2790.0 * self.grbl_scale
        
        response.success = True
        response.message = "Y-axis homing sequence initiated. Position will be set to Y=2790."
        return response

    def flash_config_callback(self, request, response):
        if not self.is_connected:
            response.success = False
            response.message = "Cannot flash config: Pico not connected."
            return response

        self.get_logger().info("Flashing Pico EEPROM with UGS matching settings...")
        max_vel = self.max_velocity_mm_s * 60.0 

        config_commands = [
            "$0=5.0", "$1=25", "$2=0", "$3=3", 
            "$4=1", "$5=4", "$6=1", "$10=510", "$14=70", 
            "$20=1", "$21=0", "$22=1", "$23=1",
            "$24=100.0", "$25=500.0", # Homing speeds (mm/min)
            "$44=3", "$45=0", "$46=0",
            f"$100={self.steps_per_mm}", f"$110={max_vel}", 
            f"$120={self.max_acceleration_mm_s2}", f"$130={self.rail_length_mm}",
            f"$131={self.rail_length_mm}" # Y-axis max travel
        ]

        for cmd in config_commands:
            self.send_gcode(cmd)
            time.sleep(0.05)
            
        self.get_logger().info("Resetting Pico to apply configuration...")
        self.send_gcode("\x18") 
        time.sleep(1.0)
        self.send_gcode("$X")
        time.sleep(0.2)
        
        response.success = True
        response.message = "Pico configured successfully."
        return response

    # ==========================================
    # HARDWARE COMMUNICATION
    # ==========================================

    def connect_to_pico(self):
        try:
            self.pico_serial = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            self.is_connected = True
            time.sleep(1.0)
            self.send_gcode("$X") 
            self.get_logger().info("Successfully connected to GRBL Pico.")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
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
                                self.get_logger().error("HARD LIMIT OR CRASH! Rail locked.")
                                self.in_alarm = True
                        self.parse_and_update_status(line)
                    elif line.startswith('error:'):
                        self.get_logger().error(f"GRBL REJECTED COMMAND: {line} (You may be hitting a soft limit!)")
                    elif "ALARM" in line.upper():
                        if not self.in_alarm:
                            self.get_logger().error(f"HARD LIMIT OR CRASH! {line}")
                            self.in_alarm = True
            except Exception:
                self.is_connected = False
                self.position_initialized = False
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

    def parse_and_update_status(self, status_line):
        # Robust state machine: Wait for it to leave Idle, then wait for it to return to Idle.
        if self.homing_state == 1 and not ("<Idle" in status_line):
            self.homing_state = 2
            self.get_logger().info(f"Homing in progress... (Detected state: {status_line.split('|')[0]})")
        elif self.homing_state == 2 and ("<Idle" in status_line or "<Alarm" in status_line):
            self.homing_state = 0
            if "<Idle" in status_line:
                cmd = "G92"
                if self.pending_x_offset is not None:
                    cmd += f" X{self.pending_x_offset:.3f}"
                    self.pending_x_offset = None
                if self.pending_y_offset is not None:
                    cmd += f" Y{self.pending_y_offset:.3f}"
                    self.pending_y_offset = None
                    
                if cmd != "G92":
                    self.get_logger().info(f"Homing complete. Applying offsets: {cmd}")
                    self.send_gcode(cmd)

        match = re.search(r'(?:MPos|WPos):([-\d.]+)(?:,([-\d.]+))?', status_line)
        if match:
            self.current_position_x_mm = float(match.group(1)) / self.grbl_scale
            if match.group(2) is not None:
                self.current_position_y_mm = float(match.group(2)) / self.grbl_scale
            else:
                self.current_position_y_mm = self.current_position_x_mm
            self.position_initialized = True
            
            x_msg = Float64()
            x_msg.data = self.current_position_x_mm
            self.x_pos_pub.publish(x_msg)
            
            y_msg = Float64()
            y_msg.data = self.current_position_y_mm
            self.y_pos_pub.publish(y_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MonolithicRailDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.keep_running = False
        rclpy.shutdown()

if __name__ == '__main__':
    main()
