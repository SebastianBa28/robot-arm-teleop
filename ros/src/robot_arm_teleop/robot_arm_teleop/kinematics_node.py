import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        
        # Parameters
        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        
        if not config_path:
            # Default path
            package_share = get_package_share_directory('robot_arm_teleop')
            config_path = os.path.join(package_share, 'ur10.yaml')
        
        self.load_config(config_path)
        
        # State
        self.q = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0]) # Home position
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Subscriptions
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        # Timer for integration and publishing
        self.dt = 0.033 # ~30Hz
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        self.current_twist = np.zeros(6)
        self.get_logger().info('Kinematics Node initialized')

    def load_config(self, path):
        with open(path, 'r') as f:
            config = yaml.safe_load(f)
        
        self.dh = []
        for i in range(1, 7):
            self.dh.append(config['dh_parameters'][f'joint_{i}'])
        self.dh = np.array(self.dh)
        self.get_logger().info(f'Loaded DH parameters from {path}')

    def cmd_vel_callback(self, msg):
        self.current_twist = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        ])

    def get_transform(self, theta, d, a, alpha):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d],
            [0,   0,      0,     1]
        ])

    def compute_jacobian(self, q):
        T = np.eye(4)
        transforms = []
        for i in range(6):
            theta_offset, d, a, alpha = self.dh[i]
            T_local = self.get_transform(q[i] + theta_offset, d, a, alpha)
            T = T @ T_local
            transforms.append(T)
        
        P_ee = transforms[-1][:3, 3]
        J = np.zeros((6, 6))
        
        # Base frame to first joint
        z_prev = np.array([0, 0, 1])
        p_prev = np.array([0, 0, 0])
        
        for i in range(6):
            J[:3, i] = np.cross(z_prev, P_ee - p_prev)
            J[3:, i] = z_prev
            
            z_prev = transforms[i][:3, 2]
            p_prev = transforms[i][:3, 3]
            
        return J

    def timer_callback(self):
        J = self.compute_jacobian(self.q)
        
        # Damped Least Squares inverse
        lambda_val = 0.01
        J_inv = J.T @ np.linalg.inv(J @ J.T + lambda_val**2 * np.eye(6))
        
        q_dot = J_inv @ self.current_twist
        
        # Clamping joint velocity
        max_vel = 1.0
        q_dot = np.clip(q_dot, -max_vel, max_vel)
        
        # Integration
        self.q += q_dot * self.dt
        
        # Publish
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        # msg.position = self.q.tolist()
        msg.position = np.zeros(6)
        msg.velocity = np.zeros(6)
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
