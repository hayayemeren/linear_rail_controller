import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('rail_jogger_remote')
    
    # Create publisher to the main node's jog topic
    pub = node.create_publisher(Float64, '/absolute_homer_left/jog_distance', 10)

    print("========================================")
    print("      MANUAL RAIL JOGGER UTILITY        ")
    print("========================================")
    print(" Instructions:")
    print("  - Type a positive number to move right (e.g.,  10)")
    print("  - Type a negative number to move left  (e.g., -10)")
    print("  - Type 'q' to quit.")
    print("========================================\n")

    try:
        while rclpy.ok():
            user_input = input("Enter distance in mm: ")
            
            if user_input.lower() == 'q':
                print("Exiting...")
                break
                
            try:
                distance = float(user_input)
                msg = Float64()
                msg.data = distance
                pub.publish(msg)
                print(f"--> Sent command: Jog {distance} mm")
            except ValueError:
                print("Invalid input. Please enter a number.")
                
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
