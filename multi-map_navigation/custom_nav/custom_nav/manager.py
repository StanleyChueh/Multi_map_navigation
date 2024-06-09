import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import os
import time

class AutomatedNavigation(Node):
    def __init__(self):
        super().__init__('manager')
        # Navigation initialization
        self.initial_pose_received = False
        self.navigation_succeeded = False
        self.last_amcl_pose_time = None
        self.pose_update_timeout = 5  # seconds
        self.monitoring_amcl = False

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10)
        
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amclpose_callback,
            10)

        self.run_sequence()

    def initialpose_callback(self, msg):
        self.initial_pose_received = True
        self.get_logger().info('Initial pose received')

    def amclpose_callback(self, msg):
        self.last_amcl_pose_time = time.time()
        if self.monitoring_amcl:
            self.get_logger().info('Received amcl_pose message')

    def run_sequence(self):
        # Run client 1 0
        self.get_logger().info('Start!')
        os.system('ros2 run custom_nav client 1 0')
        self.wait_for_initial_pose()
        
        # Run client 2 0
        time.sleep(1)
        self.get_logger().info('Start first navigation')
        os.system('ros2 run custom_nav client 2 0')
        self.start_amcl_monitoring()

        if self.navigation_succeeded:
            self.get_logger().info('Ready to load map2')
            os.system('ros2 run custom_nav client 1 1')  
            self.get_logger().info('Start second navigation')
            os.system('ros2 run custom_nav client 2 1')
            self.start_amcl_monitoring()

    def start_amcl_monitoring(self):
        self.monitoring_amcl = True
        self.last_amcl_pose_time = time.time()
        self.wait_for_navigation_success()
        self.monitoring_amcl = False

    def wait_for_initial_pose(self):
        self.get_logger().info('Waiting for initial pose...')
        while not self.initial_pose_received:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.initial_pose_received = False
        self.get_logger().info('Initial pose processed')

    def wait_for_navigation_success(self):
        self.get_logger().info('Waiting for navigation success...')
        last_log_time = self.last_amcl_pose_time

        while True:
            rclpy.spin_once(self, timeout_sec=0.1)
            current_time = time.time()
            if current_time - self.last_amcl_pose_time > self.pose_update_timeout:
                self.get_logger().info('No update in amcl_pose for 5 seconds, considering navigation success.')
                self.navigation_succeeded = True
                break
            elif current_time - last_log_time > 1:  # Log every 1 second
                self.get_logger().info('Still waiting for amcl_pose update...')
                last_log_time = current_time
            time.sleep(0.1)  # Use a smaller sleep interval for better responsiveness

        if self.navigation_succeeded:
            self.get_logger().info('Navigation succeeded')
        else:
            self.get_logger().info('Navigation failed')

def main(args=None):
    rclpy.init(args=args)
    node = AutomatedNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

