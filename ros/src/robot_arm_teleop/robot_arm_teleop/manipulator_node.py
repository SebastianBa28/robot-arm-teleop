import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import numpy as np

class ManipulatorNode(Node):
    def __init__(self):
        super().__init__('manipulator_node')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Timer for publishing commands
        self.dt = 0.033 # ~30Hz
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.t = 0.0
        self.get_logger().info('Manipulator Node initialized (Mock generator)')

    def timer_callback(self):
        msg = Twist()
        
        # Simple test signal: oscillate linear X
        msg.linear.x = 0.1 * np.sin(self.t)
        msg.linear.y = 0.05 * np.cos(self.t)
        msg.linear.z = 0.02 * np.sin(2 * self.t)
        
        self.cmd_pub.publish(msg)
        self.t += self.dt

def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
