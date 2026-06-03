import rclpy
from rclpy.node import Node
import serial
import time
import threading
import re
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Trigger

class MonolithicRailDemo(Node):
    def __init__(self):
        super().__init__('standalone_rail_demo')

        # --- HARDWARE CONNECTION STATES ---
        self.serial_lock = threading.Lock()
        self.is_connected = False
        self.in_alarm = False
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200

        # --- RAIL PARAMETERS ---
        self.jog_velocity_mm_s = 50.0
        
        # --- DEMO STATES ---
        self.current_position_mm = 0.0
        self.last_position_mm = 0.0
        self.is_moving = False
        self.position_initialized = False

        # --- ROS 2 INTERFACE (DUAL MODE) ---
        # 1. Listen for Relative commands
        self.rel_sub = self.create_subscription(
            Float64, '/demo/relative_jog', self.relative_jog_callback, 10)
            
        # 2. Listen for Absolute commands
        self.abs_sub = self.create_subscription(
            Float64, '/demo/absolute_target', self.absolute_target_callback, 10)
        
        # 3. Publish current state
        self.pos_pub = self.create_publisher(Float64, '/demo/current_position_mm', 10)
        self.moving_pub = self.create_publisher(Bool, '/demo/is_moving', 10)
        
        # 4. Safety service
        self.clear_alarm_srv = self.create_service(Trigger, '~/clear_alarm', self.clear_alarm_callback)

        # --- HARDWARE THREADS & TIMERS ---
        self.keep_running = True
        
        self.watchdog_thread = threading.Thread(target=self.connection_watchdog_loop, daemon=True)
        self.watchdog_thread.start()

        self.poll_timer = self.create_timer(1.0 / 50.0, self.poll_pico_status)
        self.state_timer = self.create_timer(0.1, self.evaluate_movement_state)

        self.get_logger().info("Dual-Mode Demo initialized. Connecting directly to Pico...")
        self.get_logger().info("Listening on /demo/relative_jog AND /demo/absolute_target")

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

    def _execute_jog(self, jog_distance):
        """Helper to format and send the raw G-code."""
        if abs(jog_distance) < 0.1:
            self.get_logger().info("Jog distance too small, ignoring.")
            return

        # Format and send the GRBL Jog Command directly to hardware
        feed_rate = self.jog_velocity_mm_s * 60.0 
        self.send_gcode(f"$J=G21G91X{jog_distance}F{feed_rate}")
        self.get_logger().info(f"Motor Jogging {jog_distance:.2f} mm...")

    def relative_jog_callback(self, msg):
        """Handles raw relative inputs (e.g., move +10mm from current pos)"""
        if not self._pre_move_checks_pass(): return
        
        self.get_logger().info(f"Received Relative Command: {msg.data:.2f} mm")
        self._execute_jog(msg.data)

    def absolute_target_callback(self, msg):
        """Handles absolute inputs (e.g., go to the 150mm mark on the rail)"""
        if not self._pre_move_checks_pass(): return
        
        if not self.position_initialized:
            self.get_logger().warn("Cannot calculate absolute target: Waiting for initial position read from Pico.")
            return

        target_mm = msg.data
        jog_distance = target_mm - self.current_position_mm
        
        self.get_logger().info(f"Received Absolute Target: {target_mm} mm. Calculated difference: {jog_distance:.2f} mm")
        self._execute_jog(jog_distance)

    def evaluate_movement_state(self):
        if not self.position_initialized:
            return
        
        position_diff = abs(self.current_position_mm - self.last_position_mm)
        currently_moving = position_diff > 0.05
        
        if currently_moving != self.is_moving:
            self.is_moving = currently_moving
            state_str = "MOVING" if self.is_moving else "STOPPED"
            self.get_logger().info(f"Rail physical state changed: {state_str}")
            
        moving_msg = Bool()
        moving_msg.data = bool(self.is_moving)
        self.moving_pub.publish(moving_msg)
        
        self.last_position_mm = self.current_position_mm

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
                                self.get_logger().error("HARD LIMIT OR CRASH! Rail locked.")
                                self.in_alarm = True
                        self.parse_and_update_status(line)
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
        match = re.search(r'(?:MPos|WPos):([-\d.]+)', status_line)
        if match:
            self.current_position_mm = float(match.group(1))
            self.position_initialized = True
            
            pos_msg = Float64()
            pos_msg.data = self.current_position_mm
            self.pos_pub.publish(pos_msg)

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
