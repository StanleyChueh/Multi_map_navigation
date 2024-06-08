import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from load_map.sample_navigator import BasicNavigator
import os
import time

class AutomatedNavigation(Node):
    def __init__(self):
        super().__init__('automated_navigation')
        # Navigation initialization
        self.navigator = BasicNavigator()
        self.initial_pose_received = False
        self.navigation_succeeded = False

        self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            10)
        
        self.run_sequence()

    def initialpose_callback(self, msg):
        self.initial_pose_received = True
        self.get_logger().info("Initial pose received: {}".format(self.initial_pose_received))

    def run_sequence(self):
        # Run client 1 0
        self.get_logger().info('Start !!')
        self.get_logger().info("Initial pose received: {}".format(self.initial_pose_received))
        self.get_logger().info("Navigation pose received: {}".format(self.navigation_succeeded))
        os.system('ros2 run custom_nav client 1 0')
        self.wait_for_initial_pose()
        
        # Run client 2 0
        self.get_logger().info('Start first navigation !!')
        self.get_logger().info("Initial pose received: {}".format(self.initial_pose_received))
        self.get_logger().info("Navigation pose received: {}".format(self.navigation_succeeded))
        os.system('ros2 run custom_nav client 2 0')
        self.wait_for_navigation_success()
        
        if self.navigation_succeeded:
            self.get_logger().info('Ready to load map2!')
            self.get_logger().info("Initial pose received: {}".format(self.initial_pose_received))
            self.get_logger().info("Navigation pose received: {}".format(self.navigation_succeeded))

            # Run client 1 1
            os.system('ros2 run custom_nav client 1 1')            
            
        # Run client 2 1
        #os.system('ros2 run custom_nav client 2 1')
        
    def wait_for_initial_pose(self):
        self.get_logger().info('Waiting for initial pose...')
        while not self.initial_pose_received:
            rclpy.spin_once(self)
        self.initial_pose_received = False
        self.get_logger().info("Initial pose received: {}".format(self.initial_pose_received))

    def wait_for_navigation_success(self):
        self.get_logger().info('Waiting for navigation success...')
        # Add logic to wait for navigation success
        # For demonstration purposes, assuming it's succeeded after waiting for some time
        while self.navigator.isNavComplete():
            time.sleep(1)
            self.get_logger().info('Navigation fail!')
            self.get_logger().info("Navigation pose received: {}".format(self.navigation_succeeded))
            self.get_logger().info("Initial pose received: {}".format(self.initial_pose_received))
            
        self.navigation_succeeded = True
        self.initial_pose_received = False
        self.get_logger().info('Navigation done!')
        self.get_logger().info("Navigation pose received: {}".format(self.navigation_succeeded))
        self.get_logger().info("Initial pose received: {}".format(self.initial_pose_received))

def main(args=None):
    rclpy.init(args=args)
    node = AutomatedNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

